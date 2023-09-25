#ifndef VIA_H
#define VIA_H

#include "Include.h"
#include "Shape.h"

using namespace std;

enum ViaType {
    Source,
    Target,
    Added
};

class Via{
    public:
        // Via(unsigned int rowId, unsigned int colId, unsigned int netId, ViaType type, Shape* shape): _rowId(rowId), _colId(colId), _netId(netId), _viaType(type), _shape(shape) {}
        Via(unsigned int netId, ViaType type, Shape* shape): _netId(netId), _viaType(type), _shape(shape) {}
        ~Via() {}
        
        // unsigned int rowId() const { return _rowId; }
        // unsigned int colId() const { return _colId; }
        unsigned int netId() const {return _netId; }
        ViaType viaType() const { return _viaType; }
        Shape* shape() {return _shape;}
        void print() {
            cerr << "Via {netId=" << _netId << ", viaType=" << _viaType << endl;
            cerr << ", shape=";
            _shape->print();
            cerr << "}" << endl;
        }
    private:
        // unsigned int _rowId;
        // unsigned int _colId;
        unsigned int _netId;
        ViaType _viaType;
        Shape* _shape;
};

class ViaCluster{
    public:
        ViaCluster() {}
        ~ViaCluster() {}

        Via* vVia(int viaId) const { return _vVia[viaId]; }
        size_t numVias() const { return _vVia.size(); }
        unsigned int netId() const { return _vVia[0]->netId(); }
        ViaType viaType() const { return _vVia[0]->viaType(); }
        // unsigned int nodeId() const { return _nodeId; }
        // double centerRowId() {
        //     double cRowId = 0;
        //     for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
        //         cRowId += _vVia[viaId]->rowId();
        //     }
        //     cRowId /= _vVia.size();
        //     return cRowId;
        // }

        // double centerColId() {
        //     double cColId = 0;
        //     for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
        //         cColId += _vVia[viaId]->colId();
        //     }
        //     cColId /= _vVia.size();
        //     return cColId;
        // }

        double centerX() {
            double x = 0;
            for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
                x += _vVia[viaId]->shape()->ctrX();
            }
            x /= _vVia.size();
            return x;
        }

        double centerY() {
            double y = 0;
            for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
                y += _vVia[viaId]->shape()->ctrY();
            }
            y /= _vVia.size();
            return y;
        }

        void addVia(Via* v) { _vVia.push_back(v); }
        // void setNodeId(unsigned int nodeId) { _nodeId = nodeId; }
        void print() {
            cerr << "ViaCluster {vVia=" << endl;
            for (size_t viaId = 0; viaId < _vVia.size(); ++ viaId) {
                _vVia[viaId]->print();
            }
            cerr << "}" << endl;
        }
    private:
        vector<Via*> _vVia;
        // unsigned int _nodeId;
};

#endif