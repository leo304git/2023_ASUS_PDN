#include "RGraph.h"

void RGraph::initRGraph(DB db) {
    // database info
    _type = RGraphType::CROSS;
    _numLayers = db.numLayers();
    _numNets = db.numNets();

    // OASG info (ports and nodes)
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        Net* net = db.vNet(netId);
        //construct non port nodes
        vector<OASGNode*> vNetNPortNode;
        _vNPortOASGNode.push_back(vNetNPortNode);
        // construct source ports
        _vSPort.push_back(net->sourcePort());
        // construct source nodes
        vector<OASGNode*> layerSNode;
        for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
            OASGNode* node;
            if (layId == 0) {
                // node = addOASGNode(netId, net->sourceViaCstr()->centerX(), net->sourceViaCstr()->centerY(), OASGNodeType::SOURCE, net->sourcePort(), false);
                node = addOASGNode(netId, net->sourcePort()->boundPolygon()->ctrX(), net->sourcePort()->boundPolygon()->ctrY(), OASGNodeType::SOURCE, net->sourcePort(), false);
            } else {
                // node = addOASGNode(netId, net->sourceViaCstr()->centerX(), net->sourceViaCstr()->centerY(), OASGNodeType::SOURCE, net->sourcePort(), true);
                node = addOASGNode(netId, net->sourcePort()->boundPolygon()->ctrX(), net->sourcePort()->boundPolygon()->ctrY(), OASGNodeType::SOURCE, net->sourcePort(), true);
            }
            
            // OASGNode* node = new OASGNode(net->sourceViaCstr()->centerX(), net->sourceViaCstr()->centerY(), OASGNodeType::SOURCE, net->sourcePort());
            layerSNode.push_back(node);
            // _vOASGNode.push_back(node);
        }
        _vSourceOASGNode.push_back(layerSNode);

        // construct target ports and nodes
        vector< vector<OASGNode*> > netLayerTNode;
        vector<Port*> netPort;
        for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
            netPort.push_back(net->targetPort(netTPortId));
            vector<OASGNode*> layerTNode;
            for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
                OASGNode* node;
                if (layId == 0) {
                    // node = addOASGNode(netId, net->vTargetViaCstr(netTPortId)->centerX(), net->vTargetViaCstr(netTPortId)->centerY(), OASGNodeType::TARGET, net->targetPort(netTPortId), false);
                    node = addOASGNode(netId, net->targetPort(netTPortId)->boundPolygon()->ctrX(), net->targetPort(netTPortId)->boundPolygon()->ctrY(), OASGNodeType::TARGET, net->targetPort(netTPortId), false);
                } else {
                    // node = addOASGNode(netId, net->vTargetViaCstr(netTPortId)->centerX(), net->vTargetViaCstr(netTPortId)->centerY(), OASGNodeType::TARGET, net->targetPort(netTPortId), true);
                    node = addOASGNode(netId, net->targetPort(netTPortId)->boundPolygon()->ctrX(), net->targetPort(netTPortId)->boundPolygon()->ctrY(), OASGNodeType::TARGET, net->targetPort(netTPortId), true);
                }
                
                // OASGNode* node = new OASGNode(net->vTargetViaCstr(netTPortId)->centerX(), net->vTargetViaCstr(netTPortId)->centerY(), OASGNodeType::TARGET, net->targetPort(netTPortId));
                layerTNode.push_back(node);
                // _vOASGNode.push_back(node);
            }
            netLayerTNode.push_back(layerTNode);
        }
        _vTPort.push_back(netPort);
        _vTargetOASGNode.push_back(netLayerTNode);
    }

    // cerr << "tartget OASG nodes:" << endl;
    // cerr << "numNets = " << db.numNets() << endl;
    // for (size_t netId = 0; netId < db.numNets(); ++ netId) {
    //     Net* net = db.vNet(netId);
    //     net->print();
    //     for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
    //         for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
    //             cerr << "_vTargetOASGNode[" << netId << "][" << netTPortId << "][" << layId << "] = " << _vTargetOASGNode[netId][netTPortId][layId]->x() << endl;
    //         }
    //     }
    // }

    // OASG info (edges)
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        vector< vector<OASGEdge*> > netLayerEdge;
        for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
            vector<OASGEdge*> layerEdge;
            netLayerEdge.push_back(layerEdge);
        }
        _vViaOASGEdge.push_back(netLayerEdge);
        _vPlaneOASGEdge.push_back(netLayerEdge);
    }
    for (size_t netId = 0; netId < db.numNets(); ++ netId) {
        Net* net = db.vNet(netId);
        // construct source and target edges
        Polygon* sPolygon = net->sourcePort()->boundPolygon();
        for (size_t layId = 0; layId < db.numLayers()-1; ++ layId) {
            addViaOASGEdge(netId, layId, _vSourceOASGNode[netId][layId], _vSourceOASGNode[netId][layId+1], sPolygon);
        }
        for (size_t netTPortId = 0; netTPortId < net->numTPorts(); ++ netTPortId) {
            Polygon* tPolygon = net->targetPort(netTPortId)->boundPolygon();
            for (size_t layId = 0; layId < db.numLayers()-1; ++ layId) {
                addViaOASGEdge(netId, layId, _vTargetOASGNode[netId][netTPortId][layId+1], _vTargetOASGNode[netId][netTPortId][layId], tPolygon);
            }
        }
    }

    // vector<Port*> _vSPort;  // index = [netId]
    // vector< vector< Port* > > _vTPort;     // index = [netId][netTPortId], the ports of the same net are sorted in descending order
    // vector<OASGNode*> _vOASGNode;   // all OASGNodes of all nets
    // vector< vector<OASGNode*> > _vSourceOASGNode;     // nodes of the source via clusters, index = [netId] [layId]
    // vector< vector< vector<OASGNode*> > > _vTargetOASGNode;     // nodes of the target via clusters index = [netId] [netTPortId] [layId]
    // // vector< vector<OASGNode*> > _vMiddleOASGNode;   // nodes other than the source or target ones, index = [layId] [middleNodeId]
    // vector< vector<OASGNode*> > _vNPortOASGNode;    // nodes that are not connected directly to ports, index = [netId] [nPortNodeId]

    // vector<OASGEdge*> _vOASGEdge;   // all OASGEdges of all nets
    // vector< vector< vector<OASGEdge*> > > _vPlaneOASGEdge;   // horizontal OASGEdges, index = [netId] [layId] [typeEdgeId]
    // vector< vector< vector<OASGEdge*> > > _vViaOASGEdge;   // vertical OASGEdges between Layer[layId, layId+1], index = [netId] [layId] [typeEdgeId]

    for (size_t netId = 0; netId <_vSPort.size(); ++ netId) {
        cerr << "_vSPort(net" << netId << "):" << endl;
        _vSPort[netId]->print();
        for (size_t tPortId = 0; tPortId < _vTPort[netId].size(); ++ tPortId) {
            cerr << "_vTPort(net" << netId << ", tPort" << tPortId << "):" << endl;
            _vTPort[netId][tPortId]->print();
        }
    }

    cerr << "_vOASGNode: " << endl;
    for (size_t nodeId = 0; nodeId < _vOASGNode.size(); ++ nodeId) {
        _vOASGNode[nodeId]->print();
    }
    for (size_t netId = 0; netId < numNets(); ++ netId) {
        for (size_t layId = 0; layId < numLayers(); ++ layId) {
            cerr << "_vSourceOASGNode(net" << netId << ", layer" << layId << "):" << endl;
            _vSourceOASGNode[netId][layId]->print(); 
        }
        for (size_t tPortId = 0; tPortId < _vTPort[netId].size(); ++ tPortId) {
            for (size_t layId = 0; layId < numLayers(); ++ layId) {
                cerr << "_vTargetOASGNode(net" << netId << ", tPort" << tPortId << ", layer" << layId << "):" << endl;
                _vTargetOASGNode[netId][tPortId][layId]->print();
            }
        }
        cerr << "_vNPortOASGNode(net" << netId << "):" << endl;
        for (size_t nPortId = 0; nPortId < _vNPortOASGNode[netId].size(); ++ nPortId) {
            _vNPortOASGNode[netId][nPortId]->print();
        }
    }

    cerr << "_vOASGEdge:" << endl;
    for (size_t edgeId = 0; edgeId < _vOASGEdge.size(); ++ edgeId) {
        _vOASGEdge[edgeId]->print();
    }

    for (size_t netId = 0; netId < numNets(); ++ netId) {
        for (size_t layId = 0; layId < numLayers(); ++ layId) {
            cerr << "_vViaOASGEdge(net" << netId << ", layer" << layId << "):" << endl;
            for (size_t viaEdgeId = 0; viaEdgeId <_vViaOASGEdge[netId][layId].size(); ++ viaEdgeId) {
                _vViaOASGEdge[netId][layId][viaEdgeId]->print();
            }
        }
    }
}

OASGNode* RGraph::addOASGNode(size_t netId, double x, double y, OASGNodeType type, Port* port, bool nPort){
    // _vOutEdgeId and _vInEdgeId will be set later through addOASGEdge()
    // _voltage will be set in the preporcessing of currentDistribution()
    OASGNode* node = new OASGNode(_vOASGNode.size(), _vNPortOASGNode[netId].size(), netId, x, y, type, port, nPort);
    _vOASGNode.push_back(node);
    if (nPort) {
        _vNPortOASGNode[netId].push_back(node);
    }
    return node;
}

size_t RGraph::addOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, bool viaEdge){
    // if viaEdge, layId is layPairId
    size_t OASGEdgeId = _vOASGEdge.size();
    size_t typeEdgeId = viaEdge ? _vViaOASGEdge[netId][layId].size() : _vPlaneOASGEdge[netId][layId].size();
    OASGEdge* edge = new OASGEdge(OASGEdgeId, netId, layId, typeEdgeId, sNode, tNode, viaEdge);
    sNode->addOutEdge(OASGEdgeId);
    tNode->addInEdge(OASGEdgeId);
    _vOASGEdge.push_back(edge);
    if (viaEdge) {
        _vViaOASGEdge[netId][layId].push_back(edge);
    } else {
        _vPlaneOASGEdge[netId][layId].push_back(edge);
    }
    return OASGEdgeId;
}

size_t RGraph::addViaOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, Polygon* boundPolygon){
    // if viaEdge, layId is layPairId
    size_t OASGEdgeId = _vOASGEdge.size();
    size_t typeEdgeId = _vViaOASGEdge[netId][layId].size();
    OASGEdge* edge = new OASGEdge(OASGEdgeId, netId, layId, typeEdgeId, sNode, tNode, true);
    edge->setBoundPolygon(boundPolygon);
    sNode->addOutEdge(OASGEdgeId);
    tNode->addInEdge(OASGEdgeId);
    _vOASGEdge.push_back(edge);
    _vViaOASGEdge[netId][layId].push_back(edge);
    return OASGEdgeId;
}

void RGraph::swapST(OASGEdge* edge) {
    OASGNode* sNode = edge->sNode();
    OASGNode* tNode = edge->tNode();
    size_t edgeId = edge->edgeId();
    sNode->deleteOutEdge(edgeId);
    tNode->deleteInEdge(edgeId);
    sNode->addInEdge(edgeId);
    tNode->addOutEdge(edgeId);
    edge->setNode(tNode, sNode);
}

void RGraph::constructRGraph() {
    // build _vRGEdge and _portPair2Edge
    size_t twoPinNetId = 0;
    for (size_t netId = 0; netId < numNets(); ++ netId) {
        // for each layer, apply DFS
        for (size_t layId = 0; layId < numLayers(); ++ layId) {
            // build RGEdges and the map between sPort and tPort
            OASGNode* sNode = sourceOASGNode(netId, layId);
            size_t sPortId = sNode->port()->portId();
            vector< vector<OASGEdge*> > paths;
            paths = DFS(sNode, netId);
            for (size_t pathId = 0; pathId < paths.size(); ++ pathId) {
                size_t tPortId = paths[pathId][0]->tNode()->port()->portId();
                pair<size_t, size_t> portPairId = make_pair(sPortId, tPortId);
                RGEdge* e = new RGEdge(paths[pathId]);
                e->updateLength();
                if (_portPair2Edge.count(portPairId) == 0) {
                    _portPair2Edge[portPairId] = twoPinNetId;
                    vector<RGEdge*> temp;
                    vector< vector< RGEdge* > > temp2(numLayers(), temp);
                    temp2[layId].push_back(e);
                    _vRGEdge.push_back(temp2);
                    twoPinNetId ++;
                } else {
                    _vRGEdge[_portPair2Edge[portPairId]][layId].push_back(e);
                }
            }
            for (size_t tPortId = 0; tPortId < numTPorts(netId); ++ tPortId) {
                Port* tPort = _vTPort[netId][tPortId];
                pair<size_t, size_t> portPairId = make_pair(sPortId, tPort->portId());
                if (_portPair2Edge.count(portPairId) == 0) {
                    _portPair2Edge[portPairId] = twoPinNetId;
                    vector<RGEdge*> temp;
                    vector< vector< RGEdge* > > temp2(numLayers(), temp);
                    _vRGEdge.push_back(temp2);
                    twoPinNetId ++;
                }
            }
            // build RGEdges and the map between tPorts
            for (size_t netTPortId = 0; netTPortId < _vTargetOASGNode[netId].size(); ++ netTPortId) {
                OASGNode* tNode = targetOASGNode(netId, netTPortId, layId);
                size_t tPortId = tNode->port()->portId();
                paths = DFS(tNode, netId);
                for (size_t pathId = 0; pathId < paths.size(); ++ pathId) {
                    size_t tPort1Id = paths[pathId][0]->tNode()->port()->portId();
                    pair<size_t, size_t> portPairId = make_pair(tPortId, tPort1Id);
                    RGEdge* e = new RGEdge(paths[pathId]);
                    e->updateLength();
                    if (_portPair2Edge.count(portPairId) == 0) {
                        _portPair2Edge[portPairId] = twoPinNetId;
                        vector<RGEdge*> temp;
                        vector< vector< RGEdge* > > temp2(numLayers(), temp);
                        temp2[layId].push_back(e);
                        _vRGEdge.push_back(temp2);
                        twoPinNetId ++;
                    } else {
                        _vRGEdge[_portPair2Edge[portPairId]][layId].push_back(e);
                    }
                }
            }
        }
    }
    _num2PinNets = twoPinNetId;
}

vector< vector<OASGEdge*> > RGraph::DFS(OASGNode* node, size_t netId) {
    vector< vector<OASGEdge*> > paths;
    for (size_t edgeId = 0; edgeId < node->numOutEdges(); ++ edgeId) {
        OASGEdge* succEdge = vOASGEdge(node->outEdgeId(edgeId));
        if (succEdge->viaEdge() == false && succEdge->netId() == netId) {
            if (succEdge->tNode()->nodeType() == OASGNodeType::TARGET) {
                vector<OASGEdge*> temp;
                temp.push_back(succEdge);
                paths.push_back(temp);
            }
            else {
                vector< vector<OASGEdge*> > tempPaths;
                tempPaths = DFS(succEdge->tNode(), netId);
                for (size_t tempPathId = 0; tempPathId < tempPaths.size(); ++ tempPathId) {
                    tempPaths[tempPathId].push_back(succEdge);
                    paths.push_back(tempPaths[tempPathId]);
                }
            }
        }
    }
    return paths;
}

void RGraph::newDFS(OASGNode* node, size_t netId, vector< vector<OASGEdge*> >& paths) {
    for (size_t edgeId = 0; edgeId < node->numOutEdges(); ++ edgeId) {
        OASGEdge* succEdge = vOASGEdge(node->outEdgeId(edgeId));
        if (succEdge->viaEdge() == false && succEdge->netId() == netId) {
            if (succEdge->tNode()->nodeType() == OASGNodeType::TARGET) {
                vector<OASGEdge*> temp;
                temp.push_back(succEdge);
                paths.push_back(temp);
            }
            else {
                vector< vector<OASGEdge*> > tempPaths;
                tempPaths = DFS(succEdge->tNode(), netId);
                for (size_t tempPathId = 0; tempPathId < tempPaths.size(); ++ tempPathId) {
                    tempPaths[tempPathId].push_back(succEdge);
                    paths.push_back(tempPaths[tempPathId]);
                }
            }
        }
    }
}
