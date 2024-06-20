#include "LayerILP.h"
using namespace std;

LayerILP::LayerILP(RGraph& rGraph, vector< vector<double> > vNetWeight, vector<double> vAccuViaLength)
        : _model(_env), _rGraph(rGraph), _vNetWeight(vNetWeight), _vAccuViaLength(vAccuViaLength) {
    // _model.getEnv().set(GRB_DoubleParam_TimeLimit, 400);
    _model.set(GRB_DoubleParam_FeasibilityTol, 1e-9);
    _vFlow = new GRBVar** [_rGraph.num2PinNets()];
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        _vFlow[twoPinNetId] = new GRBVar* [ _rGraph.numLayers() ];
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            _vFlow[twoPinNetId][layId] = new GRBVar [ _rGraph.numRGEdges(twoPinNetId, layId) ];
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                _vFlow[twoPinNetId][layId][RGEdgeId] = _model.addVar(0.0, 1.0, 0.0, GRB_BINARY, 
                                                        "F_n" + to_string(twoPinNetId) + "_l_" + to_string(layId) + "_i_" + to_string(RGEdgeId));
            }
        }
    }
    _currentLB = _model.addVar(-100000.0, 100000.0, 0.0, GRB_CONTINUOUS, "_currentLB");
    // _model.update();
    // _model.write("exp/output/LayerILP_debug.lp");
}

void LayerILP::formulate() {
    setObjective();
    setConflictConstraints();
    // _model.update();
    // _model.write("/home/leotseng/2023_ASUS_PDN/exp/output/LayerILP_debug.lp");
}

void LayerILP::solve() {
    _model.optimize();
}

void LayerILP::collectResult() {
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                if (_vFlow[twoPinNetId][layId][RGEdgeId].get(GRB_DoubleAttr_X) != 0) {
                    _rGraph.vEdge(twoPinNetId, layId, RGEdgeId)->select();
                }
            }
        }
    }
}

void LayerILP::setObjective() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Port* sPort = _rGraph.sPort(netId);
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            GRBLinExpr netCurrent;
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
            for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
                // add the sum of currents from the source port
                for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                    RGEdge* e = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                    netCurrent += ((sPort->voltage() - tPort->voltage()) / (e->length() + 2.0*_vAccuViaLength[layId])) * _vFlow[twoPinNetId][layId][RGEdgeId];
                }
                // add the sum of currents from other higher-voltage target ports
                for (size_t netTPortId1 = 0; netTPortId1 < netTPortId; ++ netTPortId1) {
                    Port* tPort1 = _rGraph.tPort(netId, netTPortId1);
                    size_t twoPinNetId1 = _rGraph.twoPinNetId(tPort1->portId(), tPort->portId());
                    for (size_t RGEdgeId1 = 0; RGEdgeId1 < _rGraph.numRGEdges(twoPinNetId1, layId); ++ RGEdgeId1) {
                        RGEdge* e1 = _rGraph.vEdge(twoPinNetId1, layId, RGEdgeId1);
                        netCurrent += ((tPort1->voltage() - tPort->voltage()) / (e1->length() + 2.0*_vAccuViaLength[layId])) * _vFlow[twoPinNetId1][layId][RGEdgeId1];
                    }
                }
                // subtract the sum of currents to other lower-voltage target ports 
                for (size_t netTPortId2 = netTPortId+1; netTPortId2 < _rGraph.numTPorts(netId); ++ netTPortId2) {
                    Port* tPort2 = _rGraph.tPort(netId, netTPortId2);
                    size_t twoPinNetId2 = _rGraph.twoPinNetId(tPort->portId(), tPort2->portId());
                    for (size_t RGEdgeId2 = 0; RGEdgeId2 < _rGraph.numRGEdges(twoPinNetId2, layId); ++ RGEdgeId2) {
                        RGEdge* e2 = _rGraph.vEdge(twoPinNetId2, layId, RGEdgeId2);
                        netCurrent -= ((tPort->voltage() - tPort2->voltage()) / (e2->length() + 2.0*_vAccuViaLength[layId])) * _vFlow[twoPinNetId2][layId][RGEdgeId2];
                    }
                }
            }
            netCurrent *= _vNetWeight[netId][netTPortId];
            _model.addConstr(netCurrent >= _currentLB, "obj_constr_n" + to_string(netId) + "_t" + to_string(netTPortId));
        }
    }

    GRBLinExpr obj;
    obj += _currentLB;
    _model.setObjective(obj, GRB_MAXIMIZE);
}

void LayerILP::setConflictConstraints() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Port* sPort = _rGraph.sPort(netId);
        for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {

            // the two pin net between the target port and the source port of the net
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
            addConflictConstraint(netId, twoPinNetId);

            // the two pin net between the target port and other target ports of the net
            for (size_t netTPortId2 = netTPortId+1; netTPortId2 < _rGraph.numTPorts(netId); ++ netTPortId2) {
                Port* tPort2 = _rGraph.tPort(netId, netTPortId2);
                size_t twoPinNetId2 = _rGraph.twoPinNetId(tPort->portId(), tPort2->portId());
                addConflictConstraint(netId, twoPinNetId2);
            }

        }
    }
}

void LayerILP::addConflictConstraint(size_t netId, size_t twoPinNetId) {
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
        for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
            RGEdge* e = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
            if (e->narrow()) {
                _model.addConstr(_vFlow[twoPinNetId][layId][RGEdgeId] <= 0);
            }

            // the RGEdges from other nets
            for (size_t netId1 = netId+1; netId1 < _rGraph.numNets(); ++ netId1) {
                Port* sPort1 = _rGraph.sPort(netId1);
                for (size_t netTPortId1 = 0; netTPortId1 < _rGraph.numTPorts(netId1); ++ netTPortId1) {

                    // the two pin net between the target port and the source port of net1
                    Port* tPort1 = _rGraph.tPort(netId1, netTPortId1);
                    size_t twoPinNetId1 = _rGraph.twoPinNetId(sPort1->portId(), tPort1->portId());
                    for (size_t RGEdgeId1 = 0; RGEdgeId1 < _rGraph.numRGEdges(twoPinNetId1, layId); ++ RGEdgeId1) {
                        RGEdge* e1 = _rGraph.vEdge(twoPinNetId1, layId, RGEdgeId1);
                        if (e->cross(e1)) {
                            _model.addConstr(_vFlow[twoPinNetId][layId][RGEdgeId] + _vFlow[twoPinNetId1][layId][RGEdgeId1] <= 1);
                        }
                    }

                    // the two pin net between the target port and other target ports of net1
                    for (size_t netTPortId11 = netTPortId1+1; netTPortId11 < _rGraph.numTPorts(netId1); ++ netTPortId11) {
                        Port* tPort11 = _rGraph.tPort(netId1, netTPortId11);
                        size_t twoPinNetId11 = _rGraph.twoPinNetId(tPort1->portId(), tPort11->portId());
                        for (size_t RGEdgeId11 = 0; RGEdgeId11 < _rGraph.numRGEdges(twoPinNetId11, layId); ++ RGEdgeId11) {
                            RGEdge* e11 = _rGraph.vEdge(twoPinNetId11, layId, RGEdgeId11);
                            if (e->cross(e11)) {
                                _model.addConstr(_vFlow[twoPinNetId][layId][RGEdgeId] + _vFlow[twoPinNetId11][layId][RGEdgeId11] <= 1);
                            }
                        }
                    }

                }
            }

        }
    }
}