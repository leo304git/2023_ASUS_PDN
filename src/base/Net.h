#ifndef NET_H
#define NET_H

#include "Include.h"
#include "Via.h"
#include "Shape.h"
using namespace std;

class DBNode {
    public:
        DBNode(string name, Node* node, size_t layId) : _name(name), _node(node), _layId(layId) {
            _upViaEdge = NULL;
            _lowViaEdge = NULL;
        }
        ~DBNode() {}

        string name() const { return _name; }
        Node* node() { return _node; }
        ViaEdge* upViaEdge() { return _upViaEdge; }
        ViaEdge* lowViaEdge() { return _lowViaEdge; }
        size_t layId() const  { return _layId; }
        void setUpViaEdge(ViaEdge* upViaEdge) { _upViaEdge = upViaEdge; }
        void setLowViaEdge(ViaEdge* lowViaEdge) { _lowViaEdge = lowViaEdge; }
    private:
        string _name;
        Node* _node;
        ViaEdge* _upViaEdge;
        ViaEdge* _lowViaEdge;
        size_t _layId;
};

class Port {
    public:
        Port(size_t portId, double voltage, double current, ViaCluster* viaCstr)
        : _portId(portId), _voltage(voltage), _current(current), _viaCluster(viaCstr) {}
        Port(size_t portId, int netTPortId, double voltage, double current)
        : _portId(portId), _netTPortId(netTPortId), _voltage(voltage), _current(current) {
            _viaCluster = NULL;
            _viaArea = -1;
            _T2SDist = 0;
        }
        ~Port() {}
        size_t portId() const { return _portId; }
        int netTPortId() const { return _netTPortId; }
        double voltage() const { return _voltage; }
        double current() const { return _current; }
        ViaCluster* viaCluster() { return _viaCluster; }
        Polygon* boundPolygon() { return _boundPolygon; }
        double viaArea() const { return _viaArea; }
        double T2SDist() const { return _T2SDist; }
        void setBoundPolygon(Polygon* polygon) { _boundPolygon = polygon; }
        void setViaArea(double viaArea) { _viaArea = viaArea; }
        void setViaCluster(ViaCluster* viaCluster) { _viaCluster = viaCluster; }
        void setT2SDist(double dist) { _T2SDist = dist; }
        void setNetTPortId(int netTPortId) { _netTPortId = netTPortId; }
        void print() {
            cerr << "Port {portId=" << _portId << ", voltage=" << _voltage << ", current=" << _current << endl;
            cerr << ", netPortId=" << _netTPortId << ", boundPolygon=";
            _boundPolygon->print();
            // cerr << ", viaCluster=";
            // _viaCluster->print();
            cerr << "}" << endl;
        }
    private:
        size_t _portId;
        int _netTPortId;
        double _voltage;
        double _current;
        ViaCluster* _viaCluster;
        Polygon* _boundPolygon;
        double _viaArea;    // assigned in GlobalMgr::currVoltOpt()
        double _T2SDist;
};

// class TwoPinNet {
//     public:
//         TwoPinNet() {}
//         ~TwoPinNet() {}
//         size_t TwoPinNetId() const { return _TwoPinNetId; }
//     private:
//         size_t _TwoPinNetId;
//         ViaCluster* _sourceViaCstr;
//         ViaCluster* _targetViaCstr;
//         vector<Shape*> _vShape;
// };

class Segment {
    public:
        Segment(Trace* trace, pair<double, double> sPos, pair<double, double> tPos, double widthLeft, double widthRight, double sVoltage, double tVoltage, double current)
         : _trace(trace), _sPos(sPos), _tPos(tPos), _widthLeft(widthLeft), _widthRight(widthRight), _sVoltage(sVoltage), _tVoltage(tVoltage), _current(current) {
            _length = sqrt(pow(sX()-tX(), 2) + pow(sY()-tY(), 2));
            _width = _trace->width();
            assert(_width == _widthLeft + _widthRight);
        }
        ~Segment() {}

        Trace* trace() { return _trace; }
        // double sX() const { return _trace->sNode()->ctrX(); }
        // double sY() const { return _trace->sNode()->ctrY(); }
        // double tX() const { return _trace->tNode()->ctrX(); }
        // double tY() const { return _trace->tNode()->ctrY(); }
        double sX() const { return _sPos.first; }
        double sY() const { return _sPos.second; }
        double tX() const { return _tPos.first; }
        double tY() const { return _tPos.second; }
        double sVoltage() const { return _sVoltage; }
        double tVoltage() const { return _tVoltage; }
        double current() const { return _current; }
        double length() const { return _length; }
        double width() const { return _width; }

        void setSVoltage(double sVoltage) { _sVoltage = sVoltage; }
        void setTVoltage(double tVoltage) { _tVoltage = tVoltage; }
        void setCurrent(double current) { _current = current; }
        void setLength(double length) { _length = length; }
        void setWidth(double width) { _width = width; }
        void setSPos(double sX, double sY) { _sPos = make_pair(sX, sY); }
        void setTPos(double tX, double tY) { _tPos = make_pair(tX, tY); }
        void setWidthLeft(double widthLeft) { _widthLeft = widthLeft; }
        void setWidthRight(double widthRight) { _widthRight = widthRight; }
        void plot(size_t netId, size_t layId) { _trace->plot(netId, layId); }

    private:
        Trace* _trace;
        Shape* _shape;
        double _sVoltage;
        double _tVoltage;
        double _current;
        double _length;     // length of the (detoured) segment
        double _width;      // width of the (detoured) segment
        double _widthLeft;      // left width of the (detoured) segment
        double _widthRight;     // right width of the (detoured) segment
        pair<double, double> _sPos;     // the source position of the spine (not center)
        pair<double, double> _tPos;     // the target position of the spine (not center)
};

class Net {
    public:
        Net(size_t numLayers) {
            vector<Segment*> tempSegment;
            vector<Shape*> tempShape;
            for (size_t layId = 0; layId < numLayers; ++ layId) {
                _vSegment.push_back(tempSegment);
                _vShape.push_back(tempShape);
            }
        }
        ~Net() {}
        ViaCluster* sourceViaCstr()                   { return _sourcePort->viaCluster(); }
        ViaCluster* vTargetViaCstr(size_t netTPortId) { return _vTargetPort[netTPortId]->viaCluster(); }
        ViaCluster* vAddedViaCstr(size_t aViaCstrIdx) { return _vAddedViaCstr[aViaCstrIdx]; }
        Port*       sourcePort()                      { return _sourcePort; }
        Port*       targetPort(size_t netTPortId)     { return _vTargetPort[netTPortId]; }
        // Trace*      vTrace(size_t layId, size_t traceId) { return _vTrace[layId][traceId]; }
        Segment*    vSegment(size_t layId, size_t segId) { return _vSegment[layId][segId]; }
        size_t      numTPorts() const                 { return _vTargetPort.size(); }
        // size_t      numTraces(size_t layId) const     { return _vTrace[layId].size(); }
        size_t      numSegments(size_t layId) const   { return _vSegment[layId].size(); }

        void addSPort(Port* port)                 { _sourcePort = port; }
        void addTPort(Port* port)                 { _vTargetPort.push_back(port); }
        void addAddedViaCstr(ViaCluster* viaCstr) { _vAddedViaCstr.push_back(viaCstr); }
        // void addTrace(Trace* trace, size_t layId) { _vTrace[layId].push_back(trace); }
        void addSegment(Segment* segment, size_t layId) { _vSegment[layId].push_back(segment); }
        void sortTPort() {
            // sort _vTargetPort by their target2source distances in an ascending order
            auto compareByDist = [] (Port* const& tPort1, Port* const& tPort2) -> bool {
                return tPort1->T2SDist() < tPort2->T2SDist();
            };
            std::sort(_vTargetPort.begin(), _vTargetPort.end(), compareByDist);
            for (size_t i = 0; i < _vTargetPort.size(); ++ i) {
                _vTargetPort[i]->setNetTPortId(i);
            }
        }

        void print() {
            cerr << "Net {" << endl;
            cerr << "sourcePort=";
            _sourcePort->print();
            cerr << "vTargetPort=" << endl;
            for (size_t tPortId = 0; tPortId < _vTargetPort.size(); ++ tPortId) {
                _vTargetPort[tPortId]->print();
            }
            cerr << "vAddedViaCstr=" << endl;
            for (size_t addedViaId = 0; addedViaId < _vAddedViaCstr.size(); ++ addedViaId) {
                _vAddedViaCstr[addedViaId] -> print();
            }
            cerr << "vShape=" << endl;
            for (size_t layId = 0; layId < _vShape.size(); ++ layId) {
                cerr << "layId=" << layId << "{" << endl;
                for (size_t shapeId = 0; shapeId < _vShape[layId].size(); ++ shapeId) {
                    _vShape[layId][shapeId]->print();
                }
                cerr << "}" << endl;
            }
            cerr << "}" << endl;

        }

    private:
        Port*                    _sourcePort;
        vector<Port*>            _vTargetPort;
        // ViaCluster* _sourceViaCstr;
        // vector<ViaCluster*> _vTargetViaCstr;
        vector<ViaCluster*>      _vAddedViaCstr;
        vector< vector<Segment*> > _vSegment;   // index = [layId] [traceId], assigned in current distribution
        vector< vector<Shape*> > _vShape;   // index = [layId] [shapeId]
        // vector< vector<Trace*> > _vTrace;   // index = [layId] [traceId], assigned in current distribution
};


#endif