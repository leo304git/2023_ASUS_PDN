#include "FlowMILP.h"

// FlowLP::FlowLP(RGraph& rGraph, vector<double> vMediumLayerThickness, vector<double> vMetalLayerThickness, vector<double> vConductivity, double currentNorm)
//     : _model(_env), _rGraph(rGraph), _vMediumLayerThickness(vMediumLayerThickness), _vMetalLayerThickness(vMetalLayerThickness), _vConductivity(vConductivity), _currentNorm(currentNorm) {
FlowMILP::FlowMILP(DB& db, RGraph& rGraph)
    : _model(_env), _db(db), _rGraph(rGraph) {
    // _env.set("LogToConsole", 0);
    // _env.set("OutputFlag", 0);
    // _env.start();
    // _model = new GRBModel(_env);
    _model.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
    _area = 0;
    _overlap = 0;
    _numCapConstrs = 0;
    _numNetCapConstrs = 0;
    _beforeCost = 0;
    _afterCost = 0;
    _areaWeight = 0;
    _viaWeight = 0;
    _diffWeight = 0;
    _vPlaneLeftFlow = new GRBVar** [_rGraph.numNets()];
    _vPlaneRightFlow = new GRBVar** [_rGraph.numNets()];
    _vPlaneDiffFlow = new GRBVar** [_rGraph.numNets()];
    _vViaFlow = new GRBVar** [_rGraph.numNets()];
    _vMaxViaCost = new GRBVar* [_rGraph.numNets()];
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        _vPlaneLeftFlow[netId] = new GRBVar* [_rGraph.numLayers()];
        _vPlaneRightFlow[netId] = new GRBVar* [_rGraph.numLayers()];
        _vPlaneDiffFlow[netId] = new GRBVar* [_rGraph.numLayers()];
        _vViaFlow[netId] = new GRBVar* [_rGraph.numLayerPairs()];
        _vMaxViaCost[netId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            _vPlaneLeftFlow[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            _vPlaneRightFlow[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            _vPlaneDiffFlow[netId][layId] = new GRBVar [_rGraph.numPlaneOASGEdges(netId, layId)];
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                _vPlaneLeftFlow[netId][layId][pEdgeId] = _model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Fl_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));
                _vPlaneRightFlow[netId][layId][pEdgeId] = _model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Fr_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));
                _vPlaneDiffFlow[netId][layId][pEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Fd_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId));                                        
            }
        }
        for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
            _vViaFlow[netId][layPairId] = new GRBVar [_rGraph.numViaOASGEdges(netId)];
            for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                if (!_rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->redundant()) {
                    _vViaFlow[netId][layPairId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                            "Fv_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                }
            }     
        }
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            _vMaxViaCost[netId][vEdgeId] = _model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, 
                                                        "Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId));
        }
    }
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowMILP::setObjective(double areaWeight, double viaWeight, double diffWeight){
    _areaWeight = areaWeight;
    _viaWeight = viaWeight;
    _diffWeight = diffWeight;
    GRBLinExpr obj;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // add cost for horizontal flows
        // cerr << "add cost for horizontal flows" << endl;
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "planeEdge(net" << netId << " layer" << layId << " pEdge" << pEdgeId;
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // assert (e->sNode()->voltage() != e->tNode()->voltage());
                if (e->sNode()->voltage() == e->tNode()->voltage()) {
                    _model.addConstr(_vPlaneLeftFlow[netId][layId][pEdgeId] == 0);
                    _model.addConstr(_vPlaneRightFlow[netId][layId][pEdgeId] == 0);
                    _model.addConstr(_vPlaneDiffFlow[netId][layId][pEdgeId] == 0);
                } else {
                    if (e->sNode()->voltage() > e->tNode()->voltage()) {
                        _model.addConstr(_vPlaneLeftFlow[netId][layId][pEdgeId] >= 0);
                        _model.addConstr(_vPlaneRightFlow[netId][layId][pEdgeId] >= 0);
                    } else {
                        _model.addConstr(_vPlaneLeftFlow[netId][layId][pEdgeId] <= 0);
                        _model.addConstr(_vPlaneRightFlow[netId][layId][pEdgeId] <= 0);
                    }
                    // cost > 0 if flow & edge have same direction; cost < 0 if opposite
                    double cost = (areaWeight * pow(1E-3 * e->length(), 2)) / (_db.vMetalLayer(layId)->conductivity() * (e->sNode()->voltage()-e->tNode()->voltage()) * _db.vMetalLayer(layId)->thickness() * 1E-3);
                    // cerr << " sVolt = " << e->sNode()->voltage() << ", tVolt = " << e->tNode()->voltage() << ", thickness = " << _db.vMetalLayer(layId)->thickness();
                    // cerr << ", cost = " << cost << endl;
                    obj += cost * (_vPlaneLeftFlow[netId][layId][pEdgeId] + _vPlaneRightFlow[netId][layId][pEdgeId]);
                    // _beforeCost += cost * (e->currentLeft() + e->currentRight()) * 1E6;

                    // set diff flow
                    // 1116 Bug
                    _model.addConstr(_vPlaneDiffFlow[netId][layId][pEdgeId] >= _vPlaneLeftFlow[netId][layId][pEdgeId] - _vPlaneRightFlow[netId][layId][pEdgeId]);
                    _model.addConstr(_vPlaneDiffFlow[netId][layId][pEdgeId] >= _vPlaneRightFlow[netId][layId][pEdgeId] - _vPlaneLeftFlow[netId][layId][pEdgeId]);
                    obj += diffWeight * cost * _vPlaneDiffFlow[netId][layId][pEdgeId];
                    _beforeCost += diffWeight * cost * abs(e->currentLeft() - e->currentRight()) * 1E6;
                }
            }
        }
        // add cost for vertical flows
        // cerr << "add cost for vertical flows" << endl;
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            // double viaCost = 0;
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                if (!e->redundant()) {
                    double costNum = 1E-3 * (0.5*_db.vMetalLayer(layPairId)->thickness()+ _db.vMediumLayer(layPairId+1)->thickness()+0.5* _db.vMetalLayer(layPairId+1)->thickness());
                    double costDen = _db.vMetalLayer(0)->conductivity() * (e->sNode()->voltage() - e->tNode()->voltage()); // has not * via cross-sectional area
                    // cerr << "costDen=" << setprecision(15) << costDen << ", ";
                    // cerr << "sNode->voltage=" << setprecision(15) << e->sNode()->voltage() << ", ";
                    // cerr << "tNode->voltage=" << setprecision(15) << e->tNode()->voltage() << endl;
                    
                    //Bug
                    assert (e->sNode()->voltage() > e->tNode()->voltage());


                    if (e->sNode()->voltage() == e->tNode()->voltage()) {
                        _model.addConstr(_vViaFlow[netId][layPairId][vEdgeId] == 0);
                    } else {
                        double cost = costNum / costDen;
                        // cerr << "cost = " << cost << endl;
                        // _model.addConstr(_vViaFlow[netId][layPairId][vEdgeId] >= 0);
                        _model.addConstr(cost * _vViaFlow[netId][layPairId][vEdgeId] <= _vMaxViaCost[netId][vEdgeId], 
                                        "max_via_cost_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId));
                        _model.addConstr(cost * _vViaFlow[netId][layPairId][vEdgeId] * 1E6 >= _db.VIA16D8A24()->metalArea());
                        // if (cost * e->current() > viaCost) {
                        //     viaCost = cost * e->current();
                        // }
                    }
                    
                }
            }
            obj += viaWeight * _vMaxViaCost[netId][vEdgeId];
            // _beforeCost += viaWeight * viaCost * 1E6;
        }
    }
    // cerr << "_model.setObjective(obj, GRB_MINIMIZE)" << endl;
    _model.setObjective(obj, GRB_MINIMIZE);
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowMILP::setConserveConstraints(bool useDemandCurrent){
    // the input and output flows for port nodes
    // cerr << "the input and output flows for port nodes" << endl;
    vector<double> vInputFlow(_rGraph.numOASGNodes(), 0.0);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        OASGNode* sNode = _rGraph.sourceOASGNode(netId, 0);
        // vInputFlow[sNode->nodeId()] = sNode->port()->current() / _currentNorm;
        if (useDemandCurrent) {
            vInputFlow[sNode->nodeId()] = sNode->port()->current();
        } else {
            // vInputFlow[sNode->nodeId()] = sNode->current();
        }
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, 0);
            // vInputFlow[tNode->nodeId()] = - tNode->port()->current() / _currentNorm;
            vInputFlow[tNode->nodeId()] = - tNode->port()->current();
        }
    }
    // flow conservation constraints for all nodes
    // cerr << "flow conservation constraints for all nodes" << endl;
    for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
        GRBLinExpr outFlow;
        OASGNode* node = _rGraph.vOASGNode(nodeId);
        if (!node->redundant()) {
            // node->print();
            for (size_t outEdgeId = 0; outEdgeId < node->numOutEdges(); ++ outEdgeId) {
                OASGEdge* edge = _rGraph.vOASGEdge(node->outEdgeId(outEdgeId)); 
                if (!edge->redundant()) {
                    if (edge->viaEdge()) {
                        outFlow += _vViaFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    } else {
                        outFlow += _vPlaneLeftFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                        outFlow += _vPlaneRightFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    }
                }
            }
            for (size_t inEdgeId = 0; inEdgeId < node->numInEdges(); ++ inEdgeId) {
                OASGEdge* edge = _rGraph.vOASGEdge(node->inEdgeId(inEdgeId)); 
                if (!edge->redundant()) {
                    if (edge->viaEdge()) {
                        outFlow -= _vViaFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    } else {
                        outFlow -= _vPlaneLeftFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                        outFlow -= _vPlaneRightFlow[edge->netId()][edge->layId()][edge->typeEdgeId()];
                    }
                }
            }
            _model.addConstr(outFlow == vInputFlow[nodeId], "flow_conserve_n" + to_string(nodeId));
        }
    }
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

// void FlowMILP::setWidthConstraints() {
//     for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
//         for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
//             for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
//                 OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
//                 if (e->sNode()->voltage() != e->tNode()->voltage()) {
//                     GRBLinExpr widthRight;
//                     GRBLinExpr widthLeft;
//                     double widthWeight = (e->length()) / ( _db.vMetalLayer(e->layId())->conductivity() * (e->sNode()->voltage()-e->tNode()->voltage()) * _db.vMetalLayer(e->layId())->thickness());
//                     widthRight += widthWeight * _vPlaneRightFlow[netId][layId][pEdgeId];
//                     _model.addConstr(widthRight * 1E3 >= 0, "width_");
//                     widthLeft += widthWeight * _vPlaneLeftFlow[netId][layId][pEdgeId];
//                     _model.addConstr(widthLeft * 1E3 >= 0, "width_");
//                 } else {
//                     _model.addConstr(_vPlaneRightFlow[netId][layId][pEdgeId] == 0, "width_");
//                     _model.addConstr(_vPlaneLeftFlow[netId][layId][pEdgeId] == 0, "width_");
//                 }
                
//             }
//         }
//     }
// }

void FlowMILP::addViaAreaConstraints(size_t netId, size_t vEdgeId, double area) {
    // _model.addConstr(_vMaxViaCost[netId][vEdgeId] * _currentNorm * 1E6 / _vConductivity[0] <= area);
    _model.addConstr(_vMaxViaCost[netId][vEdgeId] * 1E6 <= area);
}

void FlowMILP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width){
    // printf("E%-2d_%c * %.2f + E%-2d_%c * %.2f <= %.3f\n", e1->edgeId(), right1? 'r': 'l', ratio1, e2->edgeId(), right2? 'r': 'l', ratio2, width);
    // width unit = milimeter
    GRBLinExpr totalWidth;
    double widthWeight1, widthWeight2;
    assert(!e1->viaEdge());
    assert(!e2->viaEdge());
    // assert(e1->sNode()->voltage() != e1->tNode()->voltage());
    if (e1->sNode()->voltage() == e1->tNode()->voltage()) {
        widthWeight1 = 0;
    } else {
        widthWeight1 = (e1->length()) / ( _db.vMetalLayer(e1->layId())->conductivity() * (e1->sNode()->voltage()-e1->tNode()->voltage()) * _db.vMetalLayer(e1->layId())->thickness());
    }
    // assert(e2->sNode()->voltage() != e2->tNode()->voltage());
    if (e2->sNode()->voltage() == e2->tNode()->voltage()) {
        widthWeight2 = 0;
    } else {
        widthWeight2 = (e2->length()) / ( _db.vMetalLayer(e2->layId())->conductivity() * (e2->sNode()->voltage()-e2->tNode()->voltage()) * _db.vMetalLayer(e2->layId())->thickness());
    }

    if (right1) {
        // totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    } else {
        // totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    }
    if (right2) {
        // totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * widthWeight2 * ratio2;
    } else {
        // totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numCapConstrs ++;
}

void FlowMILP::addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, double width) {
    // printf("E%-2d_%c * %.2f <= %.3f\n", e1->edgeId(), right1? 'r': 'l', ratio1, width);
    // width unit = millimeter
    GRBLinExpr e1Width;
    assert(!e1->viaEdge());
    double widthWeight1;
    if (e1->sNode()->voltage() == e1->tNode()->voltage()) {
        widthWeight1 = 0;
    } else {
        widthWeight1 = (e1->length()) / (_db.vMetalLayer(e1->layId())->conductivity() * (e1->sNode()->voltage()-e1->tNode()->voltage()) * _db.vMetalLayer(e1->layId())->thickness());
    }
 
    if (right1) {
        // e1Width += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        e1Width += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    } else {
        // e1Width += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        e1Width += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    }
    _model.addConstr(e1Width * 1E3 <= width, "single_capacity_" + to_string(_numCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    // _numCapConstrs ++;
}

void FlowMILP::addSameNetCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
    GRBLinExpr totalWidth;
    double widthWeight1, widthWeight2;
    assert(!e1->viaEdge());
    assert(!e2->viaEdge());
    assert(e1->netId() == e2->netId());
    // assert(e1->sNode()->voltage() != e1->tNode()->voltage());
    if (e1->sNode()->voltage() == e1->tNode()->voltage()) {
        widthWeight1 = 0;
    } else {
        widthWeight1 = (e1->length()) / ( _db.vMetalLayer(e1->layId())->conductivity() * (e1->sNode()->voltage()-e1->tNode()->voltage()) * _db.vMetalLayer(e1->layId())->thickness());
    }
    // assert(e2->sNode()->voltage() != e2->tNode()->voltage());
    if (e2->sNode()->voltage() == e2->tNode()->voltage()) {
        widthWeight2 = 0;
    } else {
        widthWeight2 = (e2->length()) / ( _db.vMetalLayer(e2->layId())->conductivity() * (e2->sNode()->voltage()-e2->tNode()->voltage()) * _db.vMetalLayer(e2->layId())->thickness());
    }

    if (right1) {
        // totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    } else {
        // totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * widthWeight1 * ratio1;
    }
    if (right2) {
        // totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * widthWeight2 * ratio2;
    } else {
        // totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * widthWeight2 * ratio2;
    }
    _model.addConstr(totalWidth * 1E3 <= width, "same_net_capacity_" + to_string(_numNetCapConstrs));
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
    _numNetCapConstrs ++;
}

void FlowMILP::relaxCapacityConstraints(vector<double> vLambda) {
    assert(vLambda.size() == _numCapConstrs);
    _model.update();
    _modelRelaxed = new GRBModel(_model);
    for (size_t capId = 0; capId < _numCapConstrs; ++ capId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("capacity_" + to_string(capId));
        // char sense = c->get ( GRB_CharAttr_Sense );
        // if ( sense != '>') {
        //     double coef = -1.0;
        //     _model.addVar (0.0 , GRB_INFINITY , 1.0 , GRB_CONTINUOUS , 1 , & c, & coef , " ArtN_ " + c-> get ( GRB_StringAttr_ConstrName ));
        // }
        // if ( sense != '<') {
        //     double coef = 1.0;
        //     _model.addVar (0.0 , GRB_INFINITY , 1.0 , GRB_CONTINUOUS , 1 ,& c , & coef , " ArtP_ " + c-> get ( GRB_StringAttr_ConstrName ));
        // }
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vLambda[capId] , GRB_CONTINUOUS, 1, &c, &coef , "lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
    // _modelRelaxed->update();
    // _modelRelaxed->write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowMILP::relaxCapacityConstraints(vector<double> vLambda, vector<double> vSameNetLambda) {
    assert(vLambda.size() == _numCapConstrs);
    assert(vSameNetLambda.size() == _numNetCapConstrs);
    _model.update();
    _modelRelaxed = new GRBModel(_model);
    for (size_t capId = 0; capId < _numCapConstrs; ++ capId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("capacity_" + to_string(capId));
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vLambda[capId] , GRB_CONTINUOUS, 1, &c, &coef , "lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
    for (size_t netCapId = 0; netCapId < _numNetCapConstrs; ++ netCapId) {
        const GRBConstr& c = _modelRelaxed->getConstrByName("same_net_capacity_" + to_string(netCapId));
        double coef = -1.0;
        _modelRelaxed->addVar(0.0 , GRB_INFINITY , vSameNetLambda[netCapId] , GRB_CONTINUOUS, 1, &c, &coef , "same_net_lambda_" + c.get ( GRB_StringAttr_ConstrName ));
    }
    // _modelRelaxed->update();
    // _modelRelaxed->write("/home/leotseng/2023_ASUS_PDN/exp/output/FlowLP_debug.lp");
}

void FlowMILP::solve() {
    _model.optimize();
}

void FlowMILP::collectResult(){
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                // cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // assert(e->sNode()->voltage() != e->tNode()->voltage());
                double widthWeight;
                if (e->sNode()->voltage() == e->tNode()->voltage()) {
                    widthWeight = 0;
                } else {
                    widthWeight = (e->length()) / (_db.vMetalLayer(e->layId())->conductivity() * (e->sNode()->voltage()-e->tNode()->voltage()) * _db.vMetalLayer(e->layId())->thickness());
                }
                // double leftFlow = _vPlaneLeftFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                double leftFlow = _vPlaneLeftFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X);
                // cerr << "leftFlow = " << leftFlow;
                // double rightFlow = _vPlaneRightFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                double rightFlow = _vPlaneRightFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X);
                // cerr << " rightFlow = " << rightFlow << endl;
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentRight(rightFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentLeft(leftFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(leftFlow * widthWeight * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(rightFlow * widthWeight * 1E3);
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            // double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X);
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                // cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    // double flow = _vViaFlow[netId][layPairId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                    double flow = _vViaFlow[netId][layPairId][vEdgeId].get(GRB_DoubleAttr_X);
                    // cerr << "flow = " << flow << endl;
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentRight(flow);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentLeft(0.0);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
        }
    }
}
void FlowMILP::printResult() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // print results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                cerr << "edgeId=" << e->edgeId() << ", current = " << e->current() << ", widthLeft = " << e->widthLeft();
                cerr << ", widthRight = " << e->widthRight() << endl;
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                cerr << "edgeId = " << e->edgeId() << " ";
                if (!e->redundant()) {
                    cerr << "current = " << e->current() << ", viaArea = " << e->viaArea() << endl;
                } else {
                    cerr << "redundant edge" << endl;
                }
            }
        }
    }
    // cerr << _model. << endl;
    // _model.printQuality();
}

void FlowMILP::solveRelaxed() {
    _modelRelaxed->optimize();
}

void FlowMILP::collectRelaxedResult() {
    _viaArea = 0;
    _beforeCost = 0;
    _afterCost = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        // collect results for horizontal flows
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                cerr << "vPEdge[" << netId << "][" << layId << "][" << pEdgeId << "]: ";
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // assert(e->sNode()->voltage() != e->tNode()->voltage());
                double widthWeight;
                if (e->sNode()->voltage() == e->tNode()->voltage()) {
                    widthWeight = 0;
                } else {
                    widthWeight = (e->length()) / (_db.vMetalLayer(e->layId())->conductivity() * (e->sNode()->voltage()-e->tNode()->voltage()) * _db.vMetalLayer(e->layId())->thickness());
                }
                // double widthWeight = (e->length()) / (_db.vMetalLayer(e->layId())->conductivity() * abs(e->sNode()->voltage()-e->tNode()->voltage()) * _db.vMetalLayer(e->layId())->thickness());
                // double leftFlow = _vPlaneLeftFlow[netId][layId][pEdgeId].get(GRB_DoubleAttr_X) * _currentNorm;
                // double leftFlow = _modelRelaxed->getVarByName("Fl_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                double leftFlow = _modelRelaxed->getVarByName("Fl_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X);
                if (abs(leftFlow) < 1E-3) { leftFlow = 0; }
                cerr << "leftFlow = " << leftFlow;
                // double rightFlow = _modelRelaxed->getVarByName("Fr_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                double rightFlow = _modelRelaxed->getVarByName("Fr_n" + to_string(netId) + "_l_" + to_string(layId) + "_i_" + to_string(pEdgeId)).get(GRB_DoubleAttr_X);
                if (abs(rightFlow) < 1E-3) { rightFlow = 0; }
                cerr << " rightFlow = " << rightFlow << endl;

                // set before cost
                _beforeCost += _areaWeight * widthWeight * 1E3 * e->length() * e->current();

                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentRight(rightFlow);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setCurrentLeft(leftFlow);
                assert(leftFlow * widthWeight >= 0);
                assert(rightFlow * widthWeight >= 0);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthLeft(leftFlow * widthWeight * 1E3);
                _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->setWidthRight(rightFlow * widthWeight * 1E3);

                // set after cost
                _afterCost += _areaWeight * _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->length() * (_rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->widthLeft() + _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->widthRight());
                // _afterCost += _areaWeight * _diffWeight * _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->length() * (_rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->widthLeft() + _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId)->widthRight());
            }
        }
        // collect results for vertical flows
        for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
            double viaCost = 0;
            // double viaArea = _vMaxViaCost[netId][vEdgeId].get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            // double viaArea = _modelRelaxed->getVarByName("Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm / _vConductivity[0];
            double viaArea = _modelRelaxed->getVarByName("Cv_max_n" + to_string(netId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X);
            _viaArea += viaArea * 1E6;
            assert(viaArea * 1E6 > 0);
            for (size_t layPairId = 0; layPairId < _rGraph.numLayerPairs(); ++ layPairId) {
                cerr << "vVEdge[" << netId << "][" << layPairId << "][" << vEdgeId << "]: ";
                if (! _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId) -> redundant()) {
                    // double flow = _modelRelaxed->getVarByName("Fv_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X) * _currentNorm;
                    double flow = _modelRelaxed->getVarByName("Fv_n" + to_string(netId) + "_l_" + to_string(layPairId) + "_i_" + to_string(vEdgeId)).get(GRB_DoubleAttr_X);
                    if (abs(flow) < 1E-3) { flow = 0; }
                    cerr << "flow = " << flow << endl;

                    // set before cost
                    OASGEdge* e = _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId);
                    if (e->sNode()->voltage() != e->tNode()->voltage()) {
                        assert (e->sNode()->voltage() > e->tNode()->voltage());
                        double costNum = 1E-3 * (0.5*_db.vMetalLayer(layPairId)->thickness()+ _db.vMediumLayer(layPairId+1)->thickness()+0.5* _db.vMetalLayer(layPairId+1)->thickness());
                        double costDen = _db.vMetalLayer(0)->conductivity() * (e->sNode()->voltage() - e->tNode()->voltage()); // has not * via cross-sectional area
                        double cost = costNum / costDen;
                        if (cost * e->current() > viaCost) {
                            viaCost = cost * e->current();
                        }
                    }

                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentRight(flow);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setCurrentLeft(0.0);
                    _rGraph.vViaOASGEdge(netId, layPairId, vEdgeId)->setViaArea(viaArea * 1E6);
                }
            }
            _beforeCost += _viaWeight * viaCost * 1E6;
            _afterCost += _viaWeight * viaArea * 1E6;
        }
    }

    // record area and overlapped width
    _area = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _area += e->length() * (e->widthLeft() + e->widthRight());
            }
        }
    }
    // _overlap = 0;
    // for (size_t capId = 0; capId < _numCapConstrs; ++capId) {
    //     _vOverlap.push_back(_modelRelaxed->getVarByName("lambda_capacity_" + to_string(capId)).get(GRB_DoubleAttr_X));
    //     _overlap += _vOverlap[capId];
    // }
    // _sameNetOverlap = 0;
    // for (size_t netCapId = 0; netCapId < _numNetCapConstrs; ++ netCapId) {
    //     _vSameNetOverlap.push_back(_modelRelaxed->getVarByName("same_net_lambda_same_net_capacity_" + to_string(netCapId)).get(GRB_DoubleAttr_X));
    //     _sameNetOverlap += _vSameNetOverlap[netCapId];
    // }
}

void FlowMILP::printRelaxedResult() {
    double violation = 0;
    for (size_t capId = 0; capId < _numCapConstrs; ++capId) {
        violation += _modelRelaxed->getVarByName("lambda_capacity_" + to_string(capId)).get(GRB_DoubleAttr_X);
    }
    cerr << "violation = " << violation << endl;
    double planeArea = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                planeArea += e->length() * (e->widthLeft() + e->widthRight());
            }
        }
    }
    cerr << "planeArea = " << planeArea << endl;
}

void FlowMILP::addCapacityOverlap(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width, bool before) {
    double totalWidth = 0;
    double widthWeight1, widthWeight2;
    assert(!e1->viaEdge());
    assert(!e2->viaEdge());
    // assert(e1->sNode()->voltage() != e1->tNode()->voltage());
    if (e1->sNode()->voltage() == e1->tNode()->voltage()) {
        widthWeight1 = 0;
    } else {
        widthWeight1 = (e1->length()) / ( _db.vMetalLayer(e1->layId())->conductivity() * (e1->sNode()->voltage()-e1->tNode()->voltage()) * _db.vMetalLayer(e1->layId())->thickness());
    }
    // assert(e2->sNode()->voltage() != e2->tNode()->voltage());
    if (e2->sNode()->voltage() == e2->tNode()->voltage()) {
        widthWeight2 = 0;
    } else {
        widthWeight2 = (e2->length()) / ( _db.vMetalLayer(e2->layId())->conductivity() * (e2->sNode()->voltage()-e2->tNode()->voltage()) * _db.vMetalLayer(e2->layId())->thickness());
    }

    if (right1) {
        // totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += e1->currentRight() * widthWeight1 * ratio1;

    } else {
        // totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += e1->currentLeft() * widthWeight1 * ratio1;
    }
    if (right2) {
        // totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += e2->currentRight() * widthWeight2 * ratio2;
    } else {
        // totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += e2->currentLeft() * widthWeight2 * ratio2;
    }
    if (totalWidth * 1E3 > width) {
        if (before) {
            _vBeforeOverlap.push_back(totalWidth * 1E3 - width);
        } else {
            _vAfterOverlap.push_back(totalWidth * 1E3 - width);
        }
    } else {
        if (before) {
            _vBeforeOverlap.push_back(0);
        } else {
            _vAfterOverlap.push_back(0);
        }
    }
}

void FlowMILP::addSameNetCapacityOverlap(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width, bool before) {
    double totalWidth = 0;
    double widthWeight1, widthWeight2;
    assert(!e1->viaEdge());
    assert(!e2->viaEdge());
    // assert(e1->sNode()->voltage() != e1->tNode()->voltage());
    if (e1->sNode()->voltage() == e1->tNode()->voltage()) {
        widthWeight1 = 0;
    } else {
        widthWeight1 = (e1->length()) / ( _db.vMetalLayer(e1->layId())->conductivity() * (e1->sNode()->voltage()-e1->tNode()->voltage()) * _db.vMetalLayer(e1->layId())->thickness());
    }
    // assert(e2->sNode()->voltage() != e2->tNode()->voltage());
    if (e2->sNode()->voltage() == e2->tNode()->voltage()) {
        widthWeight2 = 0;
    } else {
        widthWeight2 = (e2->length()) / ( _db.vMetalLayer(e2->layId())->conductivity() * (e2->sNode()->voltage()-e2->tNode()->voltage()) * _db.vMetalLayer(e2->layId())->thickness());
    }

    if (right1) {
        // totalWidth += _vPlaneRightFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += e1->currentRight() * widthWeight1 * ratio1;

    } else {
        // totalWidth += _vPlaneLeftFlow[e1->netId()][e1->layId()][e1->typeEdgeId()] * _currentNorm * widthWeight1 * ratio1;
        totalWidth += e1->currentLeft() * widthWeight1 * ratio1;
    }
    if (right2) {
        // totalWidth += _vPlaneRightFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += e2->currentRight() * widthWeight2 * ratio2;
    } else {
        // totalWidth += _vPlaneLeftFlow[e2->netId()][e2->layId()][e2->typeEdgeId()] * _currentNorm * widthWeight2 * ratio2;
        totalWidth += e2->currentLeft() * widthWeight2 * ratio2;
    }
    if (totalWidth * 1E3 > width) {
        if (before) {
            _vBeforeSameOverlap.push_back(totalWidth * 1E3 - width);
        } else {
            _vAfterSameOverlap.push_back(totalWidth * 1E3 - width);
        }
    } else {
        if (before) {
            _vBeforeSameOverlap.push_back(0);
        } else {
            _vAfterSameOverlap.push_back(0);
        }
    }
}

void FlowMILP::calculateOverlapCost(vector<double> vLambda, vector<double> vNetLambda) {
    _beforeOverlapCost = 0;
    _afterOverlapCost = 0;
    _overlap = 0;
    _sameNetOverlap = 0;
    for (size_t capId = 0; capId < _numCapConstrs; ++capId) {
        _beforeOverlapCost += vLambda[capId] * _vBeforeOverlap[capId];
        _afterOverlapCost += vLambda[capId] * _vAfterOverlap[capId];
        _overlap += _vAfterOverlap[capId];
    }
    for (size_t netCapId = 0; netCapId < _numNetCapConstrs; ++ netCapId) {
        _beforeOverlapCost += vNetLambda[netCapId] * _vBeforeSameOverlap[netCapId];
        _afterOverlapCost += vNetLambda[netCapId] * _vAfterSameOverlap[netCapId];
        _sameNetOverlap += _vAfterSameOverlap[netCapId];
    }
    if (_overlap < 1E-3) {
        _overlap = 0;
    }
    if (_sameNetOverlap < 1E-3) {
        _sameNetOverlap = 0;
    }
    if (_beforeOverlapCost < 1E-3) {
        _beforeOverlapCost = 0;
    }
    if (_afterOverlapCost < 1E-3) {
        _afterOverlapCost = 0;
    }
}

void FlowMILP::clearVOverlap() {
    _vBeforeOverlap.clear();
    _vBeforeSameOverlap.clear();
    _vAfterOverlap.clear();
    _vAfterSameOverlap.clear();
}


