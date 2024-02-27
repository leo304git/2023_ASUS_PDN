#ifndef RGRAPH_H
#define RGRAPH_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "../base/Net.h"
// #include "OASG.h"
using namespace std;

//         topological relation        physical relation
// RGraph <--------------------> OASG <-----------------> DB

enum OASGNodeType {
    SOURCE,
    TARGET,
    MIDDLE
};

class OASGNode {
    public:
        OASGNode(size_t nodeId, size_t nPortNodeId, size_t netId, double x, double y, OASGNodeType type, Port* port = NULL, bool nPort = true)
        : _nodeId(nodeId), _nPortNodeId(nPortNodeId), _netId(netId), _x(x), _y(y), _nodeType(type), _port(port), _nPort(nPort) {
            _redundant = false;
            _vOutEdgeId.clear();
            _vInEdgeId.clear();
            // assert(false);
        }
        ~OASGNode() {}

        double x() const { return _x; }
        double y() const { return _y; }
        double voltage() const { return _voltage; }
        OASGNodeType nodeType() const { return _nodeType; }
        size_t outEdgeId(size_t i) const { return _vOutEdgeId[i]; }
        size_t inEdgeId(size_t i) const { return _vInEdgeId[i]; }
        size_t numOutEdges() const {return _vOutEdgeId.size(); }
        size_t numInEdges() const { return _vInEdgeId.size(); }
        Port* port() const { return _port; }
        size_t nodeId() const { return _nodeId; }
        size_t nPortNodeId() const { return _nPortNodeId; }
        size_t netId() const { return _netId; }
        bool nPort() const { return _nPort; }
        bool redundant() const { return _redundant; }

        void addOutEdge(size_t OASGEdgeId) { _vOutEdgeId.push_back(OASGEdgeId); }
        void addInEdge(size_t OASGEdgeId) { _vInEdgeId.push_back(OASGEdgeId); }
        void deleteOutEdge(size_t OASGEdgeId) {
            std::vector<size_t>::iterator it;
            it = find(_vOutEdgeId.begin(), _vOutEdgeId.end(), OASGEdgeId);
            assert(it != _vOutEdgeId.end());
            _vOutEdgeId.erase(it);
        }
        void deleteInEdge(size_t OASGEdgeId) {
            std::vector<size_t>::iterator it;
            it = find(_vInEdgeId.begin(), _vInEdgeId.end(), OASGEdgeId);
            assert(it != _vInEdgeId.end());
            _vInEdgeId.erase(it);
        }
        void setVoltage(double voltage) { _voltage = voltage; }
        void setRedundant() { _redundant = true; }
        void print() {
            cerr << "OASGNode {" << endl;
            cerr << "   (x,y) = (" << _x << "," << _y << ")";
            cerr << "   nodeId = " << _nodeId;
            cerr << "   nPortId = " << _nPortNodeId;
            cerr << "   netId = " << _netId;
            cerr << "   nPort = " << _nPort;
            cerr << "   redundant = " << _redundant;
            cerr << "   outEdge = {";
            for (size_t outId = 0; outId < _vOutEdgeId.size(); ++ outId) {
                cerr << _vOutEdgeId[outId] << ", ";
            }
            cerr << "}";
            cerr << "   inEdge = {";
            for (size_t inId = 0; inId < _vInEdgeId.size(); ++ inId) {
                cerr << _vInEdgeId[inId] << ", ";
            }
            cerr << "} }" << endl;
        }
    private:
        double _x;
        double _y;
        double _voltage;
        OASGNodeType _nodeType;
        vector<size_t> _vOutEdgeId;    // the OASGEdges starting from this node
        vector<size_t> _vInEdgeId;    // the OASGEdges going to this node
        Port* _port;                     // the port of the via cluster to which this node belongs, NULL if no via cluster
        size_t _nodeId;
        size_t _nPortNodeId;   // non-port node index in RGraph._vNPortOASGNode[netId]
        size_t _netId;
        bool _nPort;        // true if this node is not on layer0 of a source/target via cluster (i.e. not connected directly to a port)
        bool _redundant;    // true if this node has no current passing through
};

class OASGEdge {
    public:
        OASGEdge(size_t edgeId, size_t netId, size_t layId, size_t typeEdgeId, OASGNode* sNode, OASGNode* tNode, bool viaEdge)
        : _OASGEdgeId(edgeId), _netId(netId), _layId(layId), _typeEdgeId(typeEdgeId), _sNode(sNode), _tNode(tNode), _viaEdge(viaEdge) {
            _length = sqrt( pow(_sNode->x() - _tNode->x(), 2) + pow(_sNode->y() - _tNode->y(), 2) );
            _redundant = false;
            _widthLeft = numeric_limits<double>::infinity();
            _widthRight = numeric_limits<double>:: infinity();
        }
        ~OASGEdge() {}

        OASGNode* sNode() { return _sNode; }
        OASGNode* tNode() { return _tNode; }

        double length() const { return _length; }   // horizontal distance between the source and target node
        bool viaEdge() const { return _viaEdge; }
        size_t layId() const { return _layId; }
        size_t netId() const { return _netId; }
        size_t typeEdgeId() const { return _typeEdgeId; }
        size_t edgeId() const { return _OASGEdgeId; }
        double current() { return _currentRight + _currentLeft; }
        double currentRight() const { return _currentRight; }
        double currentLeft() const { return _currentLeft; }
        double widthLeft() const { return _widthLeft; }
        double widthRight() const { return _widthRight; }
        double viaArea() const { return _viaArea; }
        bool redundant() const { return _redundant; }
        Polygon* boundPolygon() { return _boundPolygon; }

        bool cross(OASGEdge* e) {
            // reference: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
            
            auto onSegment = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> bool {
                if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) &&
                    q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y()))
                return true;
            
                return false;
            };
            auto orientation = [] (OASGNode* p, OASGNode* q, OASGNode* r) -> int {
                int val = (q->y() - p->y()) * (r->x() - q->x()) -
                        (q->x() - p->x()) * (r->y() - q->y());
            
                if (val == 0) return 0;  // collinear
            
                return (val > 0)? 1: 2; // clock or counterclock wise
            };

            OASGNode* p1 = _sNode;
            OASGNode* q1 = _tNode;
            OASGNode* p2 = e->sNode();
            OASGNode* q2 = e->tNode();
            int o1 = orientation(p1, q1, p2);
            int o2 = orientation(p1, q1, q2);
            int o3 = orientation(p2, q2, p1);
            int o4 = orientation(p2, q2, q1);
        
            // General case
            if (o1 != o2 && o3 != o4)
                return true;
        
            // Special Cases
            // p1, q1 and p2 are collinear and p2 lies on segment p1q1
            if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        
            // p1, q1 and q2 are collinear and q2 lies on segment p1q1
            if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        
            // p2, q2 and p1 are collinear and p1 lies on segment p2q2
            if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        
            // p2, q2 and q1 are collinear and q1 lies on segment p2q2
            if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        
            return false; // Doesn't fall in any of the above cases
        }

        void setNode(OASGNode* sNode, OASGNode* tNode) {
            _sNode = sNode;
            _tNode = tNode;
            _length = sqrt( pow(_sNode->x() - _tNode->x(), 2) + pow(_sNode->y() - _tNode->y(), 2) );
        }

        void setViaEdge(bool viaEdge) { _viaEdge = viaEdge; }

        // void setCurrent(double current) { _current = current; }
        void setCurrentRight(double current) { _currentRight = current; }
        void setCurrentLeft(double current) { _currentLeft = current; }

        void setWidthLeft(double widthLeft) { _widthLeft = widthLeft; }

        void setWidthRight(double widthRight) { _widthRight = widthRight; }

        void setViaArea(double viaArea) { _viaArea = viaArea; }

        void setRedundant() { _redundant = true; }

        void setBoundPolygon(Polygon* boundPolygon) {
            assert(_viaEdge);
            _boundPolygon = boundPolygon; 
        }

        void print() {
            cerr << "OASGEdge[" << _OASGEdgeId << "], length=" << _length << ", (" << _sNode->x()/40 << " " << _sNode->y()/40 << ") -> (" << _tNode->x()/40 << " " << _tNode->y()/40 << ")" << endl;
            if (_viaEdge) {
            cerr << "bPolygon = ";
            _boundPolygon->print();
            }
        }

        // void setNode(size_t sNodeId, size_t tNodeId) { _sNodeId = sNodeId; _tNodeId = tNodeId; }
        // void setLength(double length) { _length = length; }
    private:
        double _length;     // horizontal distance between the source and target node
        OASGNode* _sNode;   // the node with higher voltage
        OASGNode* _tNode;   // the node with lower voltage
        size_t _OASGEdgeId;
        bool _viaEdge;
        size_t _layId;
        size_t _netId;
        size_t _typeEdgeId;
        // double _current;    // the current flowing from _sNode to _tNode, assigned in GlobalMgr::currentDistribution()
        double _currentRight;
        double _currentLeft; 
        double _widthLeft;  // the width to the left of the edge center line, assigned in GlobalMgr::currentDistribution() 
        double _widthRight; // the width to the left of the edge center line, assigned in GlobalMgr::currentDistribution()
        double _viaArea;    // the cross-sectional area of the via cluster of the edge, assigned in GlobalMgr::currentDistribution()
        bool _redundant;    // true if this edge is not in the current loop (usually occurs with via edges on upper layers)
        Polygon* _boundPolygon;     // bounding polygon of the via edge

        // size_t _sNodeId;    // the node with higher voltage
        // size_t _tNodeId;    // the node with lower voltage
        // pair<double, double> _sPos;
        // pair<double, double> _tPos; 

        
};

class RGEdge {
    public:
        RGEdge(vector<OASGEdge*> vEdge) : _vEdge(vEdge) { _selected = false; updateLength(); }
        ~RGEdge() {}

        size_t numEdges() const { return _vEdge.size(); }
        OASGEdge* vEdge(size_t edgeId) const { return _vEdge[edgeId]; }
        double length() const { return _length; }
        bool selected() const { return _selected; }
        bool cross(RGEdge* e) {
            for (size_t edgeId1 = 0; edgeId1 < _vEdge.size(); ++ edgeId1) {
                for (size_t edgeId2 = 0; edgeId2 < e->numEdges(); ++ edgeId2) {
                    if (_vEdge[edgeId1]->cross(e->vEdge(edgeId2))) {
                        return true;
                    }
                }
            }
            return false;
        }
        void updateLength() {
            for (size_t edgeId = 0; edgeId < _vEdge.size(); ++ edgeId) {
                _length += _vEdge[edgeId] -> length();
            }
        }
        void addEdge(OASGEdge* e) { _vEdge.push_back(e); } 
        void select() { _selected = true; }
        void print() {
            cerr << "RGEdge { _length=" << _length << endl;
            cerr << "sPort = ";
            _vEdge[0]->sNode()->port()->print();
            cerr << "tPort = ";
            _vEdge[_vEdge.size()-1]->tNode()->port()->print();
            cerr << endl;
            cerr << ", _vEdge = " << endl;
            for (size_t edgeId = 0; edgeId < _vEdge.size(); ++ edgeId) {
                _vEdge[edgeId]->print();
            }
            cerr << "}" << endl;
        }

    private:
        vector<OASGEdge*> _vEdge;
        double _length;     // exclude the length of vias
        bool _selected;
};

enum RGraphType {
    CROSS,      // merge edges from the OASG
    NO_CROSS,   // after layer distribution, before pseudo routing
    DETOURED    // after pseudo routing, before current distribution
};

class RGraph {
    public:
        // RGraph(DB& db);
        RGraph() {}
        ~RGraph() {}

        // get functions
        size_t num2PinNets() const { return _num2PinNets; }
        size_t numLayers() const { return _numLayers; }
        size_t numLayerPairs() const { return _numLayers - 1; }
        size_t numRGEdges(size_t twoPinNetId, size_t layId) const { return _vRGEdge[twoPinNetId][layId].size(); }
        RGEdge* vEdge(size_t twoPinNetId, size_t layId, size_t RGEdgeId) { return _vRGEdge[twoPinNetId][layId][RGEdgeId]; }
        size_t twoPinNetId(size_t sPortId, size_t tPortId) {
            assert(_portPair2Edge.count(make_pair(sPortId, tPortId)) > 0);
            return _portPair2Edge[make_pair(sPortId, tPortId)]; 
        } // tPortId1 < tPortId
        // size_t sNodeId(size_t netId) const { return _vSNodeId[netId]; }
        // size_t tNodeId(size_t netId, size_t netTNodeId) const { return _vTNodeId[netId][netTNodeId]; }
        // size_t numTNodes(size_t netId) const { return _vTNodeId[netId].size(); }
        Port* sPort(size_t sPortId) const { return _vSPort[sPortId]; }
        Port* tPort(size_t netId, size_t netTPortId) const { return _vTPort[netId][netTPortId]; }
        size_t numTPorts(size_t netId) const { return _vTPort[netId].size(); }
        size_t numNets() const { return _numNets; }
        size_t numOASGNodes() const { return _vOASGNode.size(); }
        size_t numNPortOASGNodes(size_t netId) const { return _vNPortOASGNode[netId].size(); }
        OASGNode* vOASGNode(size_t nodeId)       { return _vOASGNode[nodeId]; }
        OASGNode* sourceOASGNode(size_t netId, size_t layId) { return _vSourceOASGNode[netId][layId]; }
        OASGNode* targetOASGNode(size_t netId, size_t netTPortId, size_t layId) { return _vTargetOASGNode[netId][netTPortId][layId]; }
        OASGNode* vNPortOASGNode(size_t netId, size_t nPortNodeId) { return _vNPortOASGNode[netId][nPortNodeId]; }
        OASGEdge* vOASGEdge(size_t OASGEdgeId) { return _vOASGEdge[OASGEdgeId]; }
        OASGEdge* vPlaneOASGEdge(size_t netId, size_t layId, size_t planeEdgeId) { return _vPlaneOASGEdge[netId][layId][planeEdgeId]; }
        OASGEdge* vViaOASGEdge(size_t netId, size_t layPairId, size_t viaEdgeId) { return _vViaOASGEdge[netId][layPairId][viaEdgeId]; }
        size_t numPlaneOASGEdges(size_t netId, size_t layId) const { return _vPlaneOASGEdge[netId][layId].size(); }
        size_t numViaOASGEdges(size_t netId) const { return _vViaOASGEdge[netId][0].size(); }
        size_t numOASGEdges() const { return _vOASGEdge.size(); }
        vector< vector< vector<RGEdge*> > > vRGEdge() { return _vRGEdge; }

        // set functions
        void initRGraph(DB db);
        // after OASG coonstruction, before layer distribution
        void constructRGraph();
        vector< vector<OASGEdge*> > DFS(OASGNode* node, size_t netId);
        void newDFS(OASGNode* node, size_t netId, vector< vector<OASGEdge*> >& paths);
        OASGNode* addOASGNode(size_t netId, double x, double y, OASGNodeType type, Port* port = NULL, bool nPort = true);
        size_t addOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, bool viaEdge);
        size_t addViaOASGEdge(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode, Polygon* boundPolygon);
        void swapST(OASGEdge* edge);
        // void addRGEdge(RGEdge* edge, size_t twoPinNetId, size_t layId, size_t RGEdgeId) { _vRGEdge[twoPinNetId][layId][RGEdgeId] = edge; }
    private:
        RGraphType _type;
        // vector<size_t> _vSNodeId;   // index = [netId]
        // vector< vector<size_t> > _vTNodeId;     // index = [netId][netTPortId]
        vector<Port*> _vSPort;  // index = [netId]
        vector< vector< Port* > > _vTPort;     // index = [netId][netTPortId], the ports of the same net are sorted in descending order
        vector< vector< vector<RGEdge*> > > _vRGEdge;   // index = [twoPinNetId] [layId] [RGEdgeId]
        size_t _num2PinNets;
        size_t _numLayers;
        map< pair<size_t, size_t>, size_t > _portPair2Edge;
        size_t _numNets;

        vector<OASGNode*> _vOASGNode;   // all OASGNodes of all nets
        vector< vector<OASGNode*> > _vSourceOASGNode;     // nodes of the source via clusters, index = [netId] [layId]
        vector< vector< vector<OASGNode*> > > _vTargetOASGNode;     // nodes of the target via clusters index = [netId] [netTPortId] [layId]
        // vector< vector<OASGNode*> > _vMiddleOASGNode;   // nodes other than the source or target ones, index = [layId] [middleNodeId]
        vector< vector<OASGNode*> > _vNPortOASGNode;    // nodes that are not connected directly to ports, index = [netId] [nPortNodeId]

        vector<OASGEdge*> _vOASGEdge;   // all OASGEdges of all nets
        vector< vector< vector<OASGEdge*> > > _vPlaneOASGEdge;   // horizontal OASGEdges, index = [netId] [layId] [typeEdgeId]
        vector< vector< vector<OASGEdge*> > > _vViaOASGEdge;   // vertical OASGEdges between Layer[layId, layId+1], index = [netId] [layId] [typeEdgeId]
};

#endif