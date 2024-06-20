#include "TopologyMgr.h"
#include "LayerILP.h"
using namespace std;

void TopologyMgr::buildOASG(bool case5) {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t portId1 = 0; portId1 < _db.vNet(netId)->numTPorts(); ++ portId1) {
                for (size_t portId2 = portId1+1; portId2 < _db.vNet(netId)->numTPorts()+1; ++ portId2) {
                    OASGNode* sNode;
                    if (portId1 == 0) {
                        sNode = _rGraph.sourceOASGNode(netId, layId);
                    } else {
                        sNode = _rGraph.targetOASGNode(netId, portId1-1, layId);
                    }
                    OASGNode* tNode = _rGraph.targetOASGNode(netId, portId2-1, layId);
                    connectNodes(netId, layId, sNode, tNode);
                    // assert(portId1 != 1 || portId2 != 2);
                }
            }

        }
    }
    _rGraph.print();
}

void TopologyMgr::connectNodes(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode) {
    auto orientation = [](pair<double, double> a, pair<double, double> b, pair<double, double> c) -> int { 
        double res = (b.second-a.second)*(c.first-b.first) - 
                (c.second-b.second)*(b.first-a.first); 
    
        if (res == 0) 
            return 0; 
        if (res > 0) 
            return 1; 
        return -1; 
    };
  
    // Returns the square of distance between two input points 
    auto sqDist = [](pair<double, double> p1, pair<double, double> p2) -> double { 
        return (p1.first-p2.first)*(p1.first-p2.first) + 
            (p1.second-p2.second)*(p1.second-p2.second); 
    };

    auto nearPointId = [&] (pair<double, double> point, vector<pair<double, double>> poly) -> size_t {
        size_t ind = 0; 
        for (size_t i = 1; i < poly.size(); ++ i) 
            if (sqDist(point, poly[i]) < sqDist(point, poly[ind])) 
                ind = i; 
        return ind;
    };

    auto upperTangentId = [&] (size_t startId, pair<double, double> point, vector<pair<double, double>> poly) -> size_t {
        size_t up = startId; 
        while (orientation(point, poly[up], poly[(up+1)%poly.size()])>=0) 
            up = (up + 1) % poly.size(); 
        return up;
    };

    auto lowerTangentId = [&] (size_t startId, pair<double, double> point, vector<pair<double, double>> poly) -> size_t {
        size_t low = startId; 
        while (orientation(point, poly[low], poly[(poly.size()+low-1)%poly.size()])<=0) 
            low = (poly.size()+low-1)%poly.size(); 
        return low;
    };

    auto legal = [&](pair<double, double> p) -> bool {
        return (p.first > 0 && p.first < _db.boardWidth() && p.second > 0 && p.second < _db.boardHeight());
    };

    auto intersect2Points = [] (Shape* shape, double x1, double y1, double x2, double y2) -> bool {
        vector<pair<double, double>> interPoint;
        shape->intersectPoints(x1, y1, x2, y2, interPoint);
        return interPoint.size() > 1;
    };

    Shape* nearestShape = NULL;
    double minDist = std::numeric_limits<double>::max();

    // find the nearest port polygon that sNode intersects
    for (size_t netId1 = 0; netId1 < _db.numNets(); ++ netId1) {
        if (netId1 != netId) {
            for (size_t portId = 0; portId < _db.vNet(netId1)->numTPorts()+1; ++ portId) {
                Shape* portShape;
                if (portId == 0) {
                    portShape = _db.vNet(netId1)->sourcePort()->boundPolygon();
                } else {
                    portShape = _db.vNet(netId1)->targetPort(portId-1)->boundPolygon();
                }
                if (intersect2Points(portShape, sNode->x(), sNode->y(), tNode->x(), tNode->y())){
                    vector<pair<double, double>> interPoint;
                    portShape->intersectPoints(sNode->x(), sNode->y(), tNode->x(), tNode->y(), interPoint);
                    for (size_t i = 0; i < interPoint.size(); ++ i) {
                        double dist = sqrt((interPoint[i].first-sNode->x())*(interPoint[i].first-sNode->x()) + (interPoint[i].second-sNode->y())*(interPoint[i].second-sNode->y()));
                        if (dist < minDist) {
                            minDist = dist;
                            nearestShape = portShape;
                        }
                    }
                }
            }
        }
    }

    // find the nearest obstacle polygon that sNode intersects
    for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
        Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
        if (intersect2Points(obs->vShape(0), sNode->x(), sNode->y(), tNode->x(), tNode->y())){
            vector<pair<double, double>> interPoint;
            obs->vShape(0)->intersectPoints(sNode->x(), sNode->y(), tNode->x(), tNode->y(), interPoint);
            for (size_t i = 0; i < interPoint.size(); ++ i) {
                double dist = sqrt((interPoint[i].first-sNode->x())*(interPoint[i].first-sNode->x()) + (interPoint[i].second-sNode->y())*(interPoint[i].second-sNode->y()));
                if (dist < minDist) {
                    minDist = dist;
                    nearestShape = obs->vShape(0);
                }
            }
        }
    }
    
    // detour around the nearest polygon
    if (nearestShape != NULL) {
        vector<pair<double, double>> polygon = nearestShape->bPolygon();
        pair<double, double> sPoint = make_pair(sNode->x(), sNode->y());
        pair<double, double> tPoint = make_pair(tNode->x(), tNode->y());
        // point having minimum distance from the point p 
        size_t sId = nearPointId(sPoint, polygon);
        size_t sUpperId = upperTangentId(sId, sPoint, polygon);
        size_t sLowerId = lowerTangentId(sId, sPoint, polygon);
        size_t tId = nearPointId(tPoint, polygon);
        size_t tUpperId = upperTangentId(tId, tPoint, polygon);
        size_t tLowerId = lowerTangentId(tId, tPoint, polygon);

        // construct the right detoured path
        bool rightLegal = true;
        for (size_t rId = sUpperId; rId != tLowerId; rId = (rId+1)%polygon.size()) {
            if (!legal(polygon[rId])) {
                rightLegal = false;
                break;
            }
        }
        if (rightLegal) {
            OASGNode* sUpperNode = _rGraph.addOASGNode(netId, layId, polygon[sUpperId].first, polygon[sUpperId].second, OASGNodeType::MIDDLE);
            _rGraph.addOASGEdge(netId, layId, sNode, sUpperNode, false);
            if (sUpperId != tLowerId) {
                OASGNode* lastNode = sUpperNode;
                size_t rId  = (sUpperId + 1) % polygon.size();
                while (rId != tLowerId) {
                    OASGNode* newNode = _rGraph.addOASGNode(netId, layId, polygon[rId].first, polygon[rId].second, OASGNodeType::MIDDLE);
                    _rGraph.addOASGEdge(netId, layId, lastNode, newNode, false);
                    lastNode = newNode;
                    rId = (rId + 1) % polygon.size();
                }
                OASGNode* tLowerNode = _rGraph.addOASGNode(netId, layId, polygon[tLowerId].first, polygon[tLowerId].second, OASGNodeType::MIDDLE);
                _rGraph.addOASGEdge(netId, layId, lastNode, tLowerNode, false);
                // _rGraph.addOASGEdge(netId, layId, tLowerNode, tNode, false);
                connectNodes(netId, layId, tLowerNode, tNode);
            } else {
                // _rGraph.addOASGEdge(netId, layId, sUpperNode, tNode, false);
                connectNodes(netId, layId, sUpperNode, tNode);
            }
        }

        // construct the left detoured path
        bool leftLegal = true;
        for (size_t lId = sLowerId; lId != tUpperId; lId = (lId+1)%polygon.size()) {
            if (!legal(polygon[lId])) {
                leftLegal = false;
                break;
            }
        }
        if (leftLegal) {
            OASGNode* sLowerNode = _rGraph.addOASGNode(netId, layId, polygon[sLowerId].first, polygon[sLowerId].second, OASGNodeType::MIDDLE);
            _rGraph.addOASGEdge(netId, layId, sNode, sLowerNode, false);
            if (sLowerId != tUpperId) {
                OASGNode* lastNode = sLowerNode;
                size_t lId  = (sLowerId + polygon.size() - 1) % polygon.size();
                while (lId != tUpperId) {
                    OASGNode* newNode = _rGraph.addOASGNode(netId, layId, polygon[lId].first, polygon[lId].second, OASGNodeType::MIDDLE);
                    _rGraph.addOASGEdge(netId, layId, lastNode, newNode, false);
                    lastNode = newNode;
                    lId = (lId + polygon.size() - 1) % polygon.size();
                }
                OASGNode* tUpperNode = _rGraph.addOASGNode(netId, layId, polygon[tUpperId].first, polygon[tUpperId].second, OASGNodeType::MIDDLE);
                _rGraph.addOASGEdge(netId, layId, lastNode, tUpperNode, false);
                // _rGraph.addOASGEdge(netId, layId, tUpperNode, tNode, false);
                connectNodes(netId, layId, tUpperNode, tNode);
            } else {
                // _rGraph.addOASGEdge(netId, layId, sLowerNode, tNode, false);
                connectNodes(netId, layId, sLowerNode, tNode);
            }
        }
    } else {
        _rGraph.addOASGEdge(netId, layId, sNode, tNode, false);
    }
    return;
}

void TopologyMgr::plotOASG() {
    //_plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                if (e->narrow()) {
                    _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), SVGPlotColor::black, layId,1.0);
                } else {
                    _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId,1.0);
                }
            }
        }
    }
}

void TopologyMgr::trimOASG() {
    auto legal = [&](OASGNode* node) -> bool {
        return (node->x() > 0 && node->x() < _db.boardWidth() && node->y() > 0 && node->y() < _db.boardHeight());
    };
    RGraph* legalOASG = new RGraph();
    legalOASG->initRGraph(_db);
    vector<int> vNewNodeId(_rGraph.numOASGNodes(), -1);
    vector<int> vNewEdgeId(_rGraph.numOASGEdges(), -1);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            OASGNode* sNode = _rGraph.sourceOASGNode(netId, layId);
            vNewNodeId[sNode->nodeId()] = sNode->nodeId();
            for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
                OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, layId);
                vNewNodeId[tNode->nodeId()] = tNode->nodeId();
            }
        }
    }

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                if (legal(e->sNode()) && legal(e->tNode())) {
                    if (vNewEdgeId[e->edgeId()] == -1) {
                        // find or create a new sNode for the new edge
                        OASGNode* sNode = e->sNode();
                        OASGNode* newSNode;
                        if (vNewNodeId[sNode->nodeId()] == -1) {
                            newSNode = legalOASG->addOASGNode(sNode->netId(), layId, sNode->x(), sNode->y(), sNode->nodeType());
                            vNewNodeId[sNode->nodeId()] = newSNode->nodeId();
                        } else {
                            newSNode = legalOASG->vOASGNode( vNewNodeId[sNode->nodeId()] );
                        }
                        // find or create a new tNode for the new edge
                        OASGNode* tNode = e->tNode();
                        OASGNode* newTNode;
                        if (vNewNodeId[tNode->nodeId()] == -1) {
                            newTNode = legalOASG->addOASGNode(tNode->netId(), layId, tNode->x(), tNode->y(), tNode->nodeType());
                            vNewNodeId[tNode->nodeId()] = newTNode->nodeId();
                        } else {
                            newTNode = legalOASG->vOASGNode( vNewNodeId[tNode->nodeId()] );
                        }
                        size_t newEdgeId = legalOASG->addOASGEdge(e->netId(), e->layId(), newSNode, newTNode, e->viaEdge());
                        vNewEdgeId[e->edgeId()] = newEdgeId;
                    }
                } 
            }
        }
    }
    cerr << "legalOASG: \n";
    // legalOASG->print();
    _rGraph = *legalOASG;
    _rGraph.print();
}

void TopologyMgr::layerDistribution() {
    // construct the routing graph
    _rGraph.constructRGraph();
    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // cerr << "constructRGraph DONE" << endl;
    // for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //             cerr << "_vRGEdge[" << twoPinNetId << "][" << layId << "][" << RGEdgeId << "] = ";
    //             _rGraph.vEdge(twoPinNetId,layId,RGEdgeId)->print();
    //         }
    //     }
    // }

    
    // calculate normalized current demand
    vector< vector<double> > vNetWeight;
    double totalCurrent = 0;
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            totalCurrent += tPort->current();
        }
    }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        vector<double> temp;
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            temp.push_back(tPort->current() / totalCurrent);
        }
        vNetWeight.push_back(temp);
    }

    // calculate the accumulated via length of each layer
    vector<double> vAccuViaLength;
    double accuViaLength = 0;
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
        vAccuViaLength.push_back(accuViaLength);
        accuViaLength += _db.vMetalLayer(layId)->thickness() + _db.vMediumLayer(layId)->thickness();
    }

    // distribute layers to RGEdges with an ILP solver
    try {
        LayerILP solver(_rGraph, vNetWeight, vAccuViaLength);
        solver.formulate();
        solver.solve();
        solver.collectResult();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
}

void TopologyMgr::plotRGraph() {
    // _plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), e->netId(), layId);
                    }
                }
            }
        }
    }
}

void TopologyMgr::buildNCOASG() {
    cerr << "buildTestNCOASG\n";
    cerr << "before nc" << endl;
    _rGraph.print();
    RGraph* NCOASG = new RGraph();
    NCOASG->initRGraph(_db);
    vector<int> vNewNodeId(_rGraph.numOASGNodes(), -1);
    vector<int> vNewEdgeId(_rGraph.numOASGEdges(), -1);
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            OASGNode* sNode = _rGraph.sourceOASGNode(netId, layId);
            vNewNodeId[sNode->nodeId()] = sNode->nodeId();
            for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
                OASGNode* tNode = _rGraph.targetOASGNode(netId, netTPortId, layId);
                vNewNodeId[tNode->nodeId()] = tNode->nodeId();
            }
        }
    }
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                // NCOASG->addRGEdge(rgEdge, twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        if (vNewEdgeId[e->edgeId()] == -1) {
                            // find or create a new sNode for the new edge
                            OASGNode* sNode = e->sNode();
                            OASGNode* newSNode;
                            if (vNewNodeId[sNode->nodeId()] == -1) {
                                newSNode = NCOASG->addOASGNode(sNode->netId(), sNode->layId(), sNode->x(), sNode->y(), sNode->nodeType());
                                vNewNodeId[sNode->nodeId()] = newSNode->nodeId();
                            } else {
                                newSNode = NCOASG->vOASGNode( vNewNodeId[sNode->nodeId()] );
                            }
                            // find or create a new tNode for the new edge
                            OASGNode* tNode = e->tNode();
                            OASGNode* newTNode;
                            if (vNewNodeId[tNode->nodeId()] == -1) {
                                newTNode = NCOASG->addOASGNode(tNode->netId(), tNode->layId(), tNode->x(), tNode->y(), tNode->nodeType());
                                vNewNodeId[tNode->nodeId()] = newTNode->nodeId();
                            } else {
                                newTNode = NCOASG->vOASGNode( vNewNodeId[tNode->nodeId()] );
                            }
                            size_t newEdgeId = NCOASG->addOASGEdge(e->netId(), e->layId(), newSNode, newTNode, e->viaEdge());
                            vNewEdgeId[e->edgeId()] = newEdgeId;
                        }
                    }
                }
            }
        }
    }
    cerr << "NCOASG:" << endl;
    NCOASG->print();

    // set redundant nodes and edges
    for (size_t netId = 0; netId < NCOASG->numNets(); ++ netId) {
        for (int layId = NCOASG->numLayers()-1; layId >= 0; -- layId) {
            // set redundant source nodes and edges
            OASGNode* sNode = NCOASG->sourceOASGNode(netId, layId);
            if (sNode->numOutEdges() == 0) {
                sNode->setRedundant();
                assert(sNode->numInEdges() > 0);
                NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
            } else if (sNode->numOutEdges() == 1) {
                if (NCOASG->vOASGEdge(sNode->outEdgeId(0))->redundant()) {
                    sNode->setRedundant();
                    assert(sNode->numInEdges() > 0);
                    NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
                }
            }
            // set redundant target nodes and edges
            for (size_t tPortId = 0; tPortId < NCOASG->numTPorts(netId); ++ tPortId) {
                OASGNode* tNode = NCOASG->targetOASGNode(netId, tPortId, layId);
                if (tNode->numInEdges() == 0) {
                    tNode->setRedundant();
                    assert(tNode->numOutEdges() > 0);
                    NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                } else if (tNode->numInEdges() == 1) {
                    if (NCOASG->vOASGEdge(tNode->inEdgeId(0))->redundant()) {
                        tNode->setRedundant();
                        assert(tNode->numOutEdges() > 0);
                        NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                    }
                }
            }
        }
    }
    _rGraph = *NCOASG;
}

void TopologyMgr::plotNCOASG() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId, 1.0);
            }
        }
    }
}