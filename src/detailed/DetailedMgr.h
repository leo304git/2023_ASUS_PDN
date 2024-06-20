#ifndef DETAILED_MGR_H
#define DETAILED_MGR_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "DetailedDB.h"
#include "AStarRouter.h"
#include "OctAStarRouter.h"
#include <utility>
using namespace std;

class DetailedMgr {
    public:
        DetailedMgr(DB& db, SVGPlot& plot, double gridWidth) : _db(db), _plot(plot), _gridWidth(gridWidth) {
            _numNegoIters = 1;
            _widthRatio = 0.9;
            _obsCongest = _db.numNets() * 10.0;
            _distWeight = 0.2;
            _cLineDistWeight = 0.1;
            _numXs = _db.boardWidth() / _gridWidth;
            _numYs = _db.boardHeight() / _gridWidth;
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                vector< vector<Grid*> > vLayGrid;
                for (size_t xId = 0; xId < _numXs; ++ xId) {
                    vector<Grid*> vXGrid;
                    for (size_t yId = 0; yId < _numYs; ++ yId) {
                        Grid* grid = new Grid(xId, yId, _db.numNets());
                        vXGrid.push_back(grid);
                    }
                    vLayGrid.push_back(vXGrid);
                }
                _vGrid.push_back(vLayGrid);
            }
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< Grid* > > vNetGrid;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    vector<Grid*> vLayGrid;
                    vNetGrid.push_back(vLayGrid);
                }
                _vNetGrid.push_back(vNetGrid);
            }
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< pair<int, int> > > temp;
                for (size_t portId = 0; portId <_db.vNet(netId)->numTPorts()+1; ++ portId) {
                    vector< pair<int, int> > tempPort;
                    temp.push_back(tempPort);
                }
                _vNetPortGrid.push_back(temp);
            }
            // _vTPortCurr.reserve(_db.numNets());
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                // _vTPortCurr[netId].reserve(_db.vNet(netId)->numTPorts());
                vector<double> temp(_db.vNet(netId)->numTPorts(), 0.0);
                _vTPortCurr.push_back(temp);
            }
            // _vTPortVolt.reserve(_db.numNets());
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                // _vTPortVolt[netId].reserve(_db.vNet(netId)->numTPorts());
                vector<double> temp(_db.vNet(netId)->numTPorts(), 0.0);
                _vTPortVolt.push_back(temp);
            }
        }
        ~DetailedMgr() {}

        vector< vector< vector< pair<int, int> > > > vNetPortGrid() { return _vNetPortGrid; }

        void initGridMap();
        void initPortGridMap();
        void initSegObsGridMap();
        void printResult();
        void plotGridMap();
        void plotGridMapVoltage();
        void plotGridMapCurrent();
        void setPortGridCong(size_t layId, size_t netId, size_t portId, int cong);
        void resetPortGridCong(size_t layId, size_t netId, size_t portId, int cong);
        void removeObsRailGrids();
        void naiveAStar();
        void negoAStar(bool sameNetCong);
        void addPortVia();
        void plotVia();
        void addViaGrid();

        void print() {
            printf("\n===print=====\n");
            printf("_numXs: %d\n", _numXs);
            printf("_numYs: %d\n", _numYs);
            printf("vGrid.size(): %d\n", _vGrid.size());
            for(int i=0; i<_vGrid.size(); i++) {
                printf("vGrid[%d].size(): %d\n", i, _vGrid[i].size());
                for(int j=0; j<_vGrid[i].size(); j++) {
                    printf("vGrid[%d][%d].size(): %d\n", i, j, _vGrid[i][j].size());
                    for(int k=0; k<_vGrid[i][j].size(); k++) {
                        printf("vGrid[%d][%d][%d]: ", i, j, k);
                        _vGrid[i][j][k]->print();
                    }
                }
            }
            printf("\n\n");

            printf("vNetGrid.size(): %d\n", _vNetGrid.size());
            for(int i=0; i<_vNetGrid.size(); i++) {
                printf("vNetGrid[%d].size(): %d\n", i, _vNetGrid[i].size());
                for(int j=0; j<_vNetGrid[i].size(); j++) {
                    printf("vNetGrid[%d][%d].size(): %d\n", i, j, _vNetGrid[i][j].size());
                    for(int k=0; k<_vNetGrid[i][j].size(); k++) {
                        printf("vNetGrid[%d][%d][%d]: ", i, j, k);
                        _vNetGrid[i][j][k]->print();
                    }
                }
            }
        }
        void buildMtx();
        void buildSingleNetMtx(size_t netId);
        double getResistance(Grid*, Grid*);
        void check();
        void SmartGrow(size_t netId, int k);
        void SmartRefine(size_t netId, int k);
        bool SmartRemove(size_t netId, int k);
        bool NetEdgeDetect(size_t netId, size_t layId, Grid* grid);
        void SmartDistribute();
        void PostProcessing(bool smartRemove);
        void RemoveIsolatedGrid();
        void writeColorMap_v2(const char*, bool);

    private:
    
        vector< pair<double, double> > kMeansClustering(vector< pair<int,int> > vGrid, int numClusters, int numEpochs);
        void clearNet(size_t layId, size_t netId);
        bool legal(int xId, int yId) { return (xId>=0 && xId<_vGrid[0].size() && yId>=0 && yId<_vGrid[0][0].size()); }
        DB& _db;
        SVGPlot& _plot;
        double _gridWidth;
        vector< vector< vector< Grid* > > > _vGrid;     // index = [layId] [xId] [yId]; from left xId = 0, from bottom yId = 0
        vector< vector< vector< Grid* > > > _vNetGrid;  // index = [netId] [layId] [gridId]
        vector< vector< vector< pair<int, int> > > > _vNetPortGrid;        // index = [netId] [portId] [gridId]
        size_t _numXs;
        size_t _numYs;
        vector< vector< double > > _vTPortVolt;     // index = [netId] [netTportId], record the target port voltage during simulation
        vector< vector< double > > _vTPortCurr;     // index = [netId] [netTportId], record the target port current during simulation

        // parameters for tuning
        size_t _numNegoIters;    // the number of negotiation iterations in each layer
        double _widthRatio;      // the larger, the wider search width
        int _obsCongest;         // the congestion of obstacles and regions outside boundaries
        double _distWeight;      // the weight of distance in A* cost
        double _cLineDistWeight; // the weight of distance to the center line in A* cost
};

#endif