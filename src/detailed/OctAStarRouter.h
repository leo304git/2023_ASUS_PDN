#ifndef OCT_ASTAR_ROUTER_H
#define OCT_ASTAR_ROUTER_H

#include "../base/Include.h"
#include "DetailedDB.h"
#include "AStarRouter.h"
using namespace std;

// enum Direction {
//     Up, Down, Right, Left,
//     UpRight, UpLeft, DownRight, DownLeft
// };

class OctAStarRouter {
    public:
        OctAStarRouter(vector< vector< Grid* > > vGrid, pair<int, int> sPos, pair<int, int> tPos, pair<int, int> sRealPos, pair<int, int> tRealPos, double gridWidth, double lbLength, double lbWidth, double widthRatio, double obsCongest, double distWeight, double cLineDistWeight, bool rAware = false)
        : _vGrid(vGrid), _sPos(sPos), _tPos(tPos), _sRealPos(sRealPos), _tRealPos(tRealPos), _gridWidth(gridWidth), _lbLength(lbLength), _lbWidth(lbWidth), _widthRatio(widthRatio), _obsCongest(obsCongest), _distWeight(distWeight), _cLineDistWeight(cLineDistWeight), _rAware(rAware) {
            for (size_t xId = 0; xId < numXId(); ++ xId) {
                vector<GNode*> temp;
                for (size_t yId = 0; yId < numYId(); ++ yId) {
                    GNode* node = new GNode(xId, yId);
                    temp.push_back(node);
                }
                _vGNode.push_back(temp);
            }
        }
        ~OctAStarRouter() {}

        bool route();
        void backTrace(int tXId, int tYId);
        void backTraceNoPad();
        double marginCongestCost(int xId, int yId, Direction dir);
        double estDistCost(int xId, int yId) {
            // return abs(_tPos.first - xId) + abs(_tPos.second - yId);
            double longer = max(abs(_tPos.first - xId), abs(_tPos.second - yId));
            double shorter = min(abs(_tPos.first - xId), abs(_tPos.second - yId));
            return longer + shorter * (sqrt(2.0)-1.0);
        }
        double lineDistCost(int xId, int yId) {
            double den = abs(xId*(_tPos.second-_sPos.second) + _tPos.first*(_sPos.second-yId) + _sPos.first*(yId-_tPos.second));
            double num = sqrt(pow(_tPos.first-_sPos.first,2)+pow(_tPos.second-_sPos.second,2));
            return den / num;
        }

        size_t numXId() const { return _vGrid.size(); }
        size_t numYId() const { return _vGrid[0].size(); }
        // vector<Grid*> path() { return _path; }
        Grid* vPGrid(size_t pGridId) { return _vPGrid[pGridId]; }
        size_t numPGrids() const { return _vPGrid.size(); }
        size_t exactWidth() const { return _exactWidth; }
        size_t exactLength() const { return _exactLength; }
        Grid* vPath(size_t pathId) { return _path[pathId]; }
        size_t numPaths() const { return _path.size(); }
        // Temporarily set pathLength as public
        double pathLength(int threshold, int method);
        Direction giveDirection (GNode* curNode);

    private:
        bool legal(int xId, int yId) { return (xId>=0 && xId<numXId() && yId>=0 && yId<numYId()); }
        
        // input
        vector< vector< Grid* > > _vGrid;   // index = [xId] [yId]
        pair<int, int> _sPos;     // (source xId, source yId) of the trace (of segment)
        pair<int, int> _tPos;     // (target xId, target yId) of the trace (of segment)
        pair<int, int> _sRealPos;     // (source xId, source yId) of the segment (not trace)
        pair<int, int> _tRealPos;     // (target xId, target yId) of the segment (not trace)
        double _gridWidth;
        double _lbLength;   // the shortest wirelength form the last iteration
        double _lbWidth;    // the width lower bound calculated from the shortest wirelength
        // double _lbWidthLeft;
        // double _lbWidthRight;
        double _widthRatio;     // the probability of straight routing (without detour on a step)
        double _distWeight;     // the weight of distance cost w.r.t. congestion cost
        double _cLineDistWeight;
        // output
        vector< Grid* > _path;  // the spine of the path
        vector< Grid* > _vPGrid;    // the grids in the path (considering width)
        size_t _exactWidth;
        size_t _exactLength;
        // process
        vector< vector< GNode* > > _vGNode;     // index = [xId] [yId]
        double _obsCongest;         // congestion cost (including history) of obstacles and grids out of boudaries
        bool _rAware;
};

#endif