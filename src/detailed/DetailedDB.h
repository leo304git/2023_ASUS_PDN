#ifndef DETAILED_DB_H
#define DETAILED_DB_H

#include "../base/Include.h"
#include <cstddef>
using namespace std;

class Grid {
    public:
        Grid(size_t xId, size_t yId, size_t numNets) : _xId(xId), _yId(yId) {
            _congestion = 0;
            _congestCur = 0;
            _congestHis = 0;
            _hasObs = false;
            for (size_t netId = 0; netId < numNets; ++ netId) {
                _vVoltage.push_back(-1);
                _vCurrent.push_back(-1);
            }
            for (size_t netId = 0; netId < numNets; ++ netId) {
                _IsPort.push_back(false);
            }
        }
        ~Grid() {}

        // get function
        int congestion() const { return _congestion; }
        int congestCur() const { return _congestCur; }
        int congestHis() const { return _congestHis; }
        size_t vNetId(size_t i) const { return _vNetId[i]; }
        size_t numNets() const { return _vNetId.size(); }
        bool hasNet(size_t netId) {
            for (size_t i = 0; i<_vNetId.size(); ++ i) {
                if (_vNetId[i] == netId) return true;
            }
            return false;
        }
        int xId() { return _xId; }
        int yId() { return _yId; }
        double voltage(size_t netId) const { return _vVoltage[netId]; }
        double current(size_t netId) const { return _vCurrent[netId]; }
        bool hasObs() const { return _hasObs; }
        bool IsPort(size_t netId ) const { return _IsPort[netId]; }

        // set function
        void incCongestCur() { _congestCur += 10; _congestion += 10; }
        void decCongestCur() { _congestCur -= 10; _congestion -= 10; }
        void incCongestHis() { _congestHis += 14; _congestion += 14; }
        void setPort(size_t netId){ _IsPort[netId] = true; }
        // void decCongestHis() { _congestHis --; _congestion --; }
        void addCongestCur(int congestion) { _congestCur += congestion; _congestion += congestion; } 
        void addNet(size_t netId) { _vNetId.push_back(netId); }
        void removeNet(size_t netId) {
            for (vector<size_t>::iterator i = _vNetId.begin(); i != _vNetId.end(); ++ i) {
                if (*i == netId) {
                    _vNetId.erase(i);
                    return;
                }
            }
        }
        void setVoltage(size_t netId, double voltage) { _vVoltage[netId] = voltage; }
        void setCurrent(size_t netId, double current) { _vCurrent[netId] = current; }
        void setObs() { _hasObs = true; }

        // other function
        void print() {
            printf("(%d, %d), net: ", _xId, _yId);
            for(int i=0; i<_vNetId.size(); i++)
                printf("%d ", _vNetId[i]);
            printf("\n");
        }
        
    private:
        int _congestion;    // _congestHis + _congestCur
        int _congestCur;    // current congestion cost
        int _congestHis;    // history congestion cost
        bool _hasObs;
        vector<bool> _IsPort; // index = [netId], port of the net
        vector<size_t> _vNetId;
        size_t _xId;
        size_t _yId;
        vector<double> _vVoltage;    // index = [netId], the voltage of the grid center point, assigned in detailedMgr::buildMtx
        vector<double> _vCurrent;    // index = [netId], the current through the grid center point
};

enum GNodeStatus {
    Init,
    InQueue,
    InPath
};

class GNode {
    public:
        GNode(int xId, int yId) : _xId(xId), _yId(yId) {
            _status = GNodeStatus::Init;
            _parent = NULL;
            _cost = numeric_limits<double>::infinity();
            _curDist = numeric_limits<double>::infinity();
            _estDist = numeric_limits<double>::infinity();
            _congest = numeric_limits<double>::infinity();
        }
        ~GNode() {}

        // get function
        int xId() const { return _xId; }
        int yId() const { return _yId; }
        GNodeStatus status() const { return _status; }
        GNode* parent() { return _parent; }
        double cost() const { return _cost; }
        double curDist() const { return _curDist; }
        double estDist() const { return _estDist; }
        double congest() const { return _congest; }

        // set function
        void setStatus(GNodeStatus status) { _status = status; }
        void setParent(GNode* parentNode) { _parent = parentNode; }
        void setCost(double cost) { _cost = cost; }
        void setCurDist(double curDist) { _curDist = curDist; }
        void setEstDist(double estDist) { _estDist = estDist; }
        void setCongest(double congest) { _congest = congest; }

    private:
        int _xId;
        int _yId;
        GNodeStatus _status;
        GNode* _parent;
        double _cost;
        double _curDist;    // the current distance cost (from the source)
        double _estDist;    // the estimated distance cost (to the target)
        double _congest;    // the (accumulated) congestion cost
};

#endif