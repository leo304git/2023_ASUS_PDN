#ifndef TOPOLOGY_MGR
#define TOPOLOGY_MGR

#include "../base/Include.h"
#include "../base/SVGPlot.h"
#include "../base/DB.h"
#include "RGraph.h"

class TopologyMgr {
    public:

        TopologyMgr(DB& db, RGraph& rGraph, SVGPlot& plot): _db(db), _rGraph(rGraph), _plot(plot) {
            cerr << "numNets = " << _db.numNets() << endl;
            // _rGraph.initRGraph(db);
        }
        
        ~TopologyMgr() {}

        void buildOASG(bool case5);

        // bool isSegmentIntersectingWithObstacles(OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle);
        // bool onSegment(OASGNode* p, OASGNode* q, OASGNode* r);
        // int orientation(OASGNode* p, OASGNode* q, OASGNode* r);
        // bool doIntersect(OASGNode* p1, OASGNode* q1, OASGNode* p2, OASGNode* q2);
        // void connectWithObstacle(int netId, int layerId,OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle);
        // bool checkWithVias(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<vector<OASGNode*>>> viaOASGNodes);
        //用來存每一層有哪一個Obstacle要繞Rounding Edges
        // vector<bool> addObsRoundEdges;
        //用來存這一層中有哪些Net已經被建立過了。裡面會存兩個座標的(xMin, xMax, yMin, yMax)
        //如果有一樣的就不再加
        // std::vector<std::array<int, 4>> alreadyAddedEdges;
        // bool edgeExist(int netId, int layerId, OASGNode* a, OASGNode* b);


        void plotOASG();
        void plotRGraph();
        void trimOASG();
        void layerDistribution();
        void buildNCOASG();
        void plotNCOASG();
        
    private:
        void connectNodes(size_t netId, size_t layId, OASGNode* sNode, OASGNode* tNode);
        void detour();
        DB& _db;
        SVGPlot& _plot;
        RGraph& _rGraph;
};

#endif