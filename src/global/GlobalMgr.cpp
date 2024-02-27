#include "GlobalMgr.h"
#include "LayerILP.h"
#include "VoltEigen.h"
#include "FlowLP.h"
#include "VoltCP.h"
#include "VoltSLP.h"
#include "AddCapacity.h"
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
// public functions
using namespace std;

void GlobalMgr::plotDB() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t viaId = 0; viaId < _db.numVias(); ++ viaId) {
            Via* via = _db.vVia(viaId);
            via->shape()->plot(via->netId(), layId);
            // _plot.drawCircle(via->shape()->ctrX(), via->shape()->ctrY(), via->shape()->radius(), via->netId());
        }
        for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
            Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
            for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                obs->vShape(shapeId)->plot(SVGPlotColor::gray, layId);
            }
        }
    }
    
}


void GlobalMgr::buildTestOASG() {
    cerr << "buildTestOASG..." << endl;
    // layer0
    cerr << "layer0..." << endl;
    OASGNode* lay0_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay0_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 0, _rGraph.sourceOASGNode(0,0), _rGraph.targetOASGNode(0,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,0,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.sourceOASGNode(1,0), _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode1, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode1, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(1, 0, _rGraph.targetOASGNode(1,0,0), lay0_detNode2, false);
    _rGraph.addOASGEdge(1, 0, lay0_detNode2, _rGraph.targetOASGNode(1,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,0,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.sourceOASGNode(2,0), _rGraph.targetOASGNode(2,1,0), false);
    _rGraph.addOASGEdge(2, 0, _rGraph.targetOASGNode(2,0,0), _rGraph.targetOASGNode(2,1,0), false);

    // layer1
    cerr << "layer1..." << endl;
    OASGNode* lay1_obsNode1 = _rGraph.addOASGNode(2, 32, 56, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode2 = _rGraph.addOASGNode(2, 48, 56, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode3 = _rGraph.addOASGNode(2, 48, 60, OASGNodeType::MIDDLE);
    OASGNode* lay1_obsNode4 = _rGraph.addOASGNode(2, 32, 60, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay1_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 1, _rGraph.sourceOASGNode(0,1), _rGraph.targetOASGNode(0,0,1), false);

    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,0,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.sourceOASGNode(1,1), _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode1, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode1, _rGraph.targetOASGNode(1,1,1), false);
    _rGraph.addOASGEdge(1, 1, _rGraph.targetOASGNode(1,0,1), lay1_detNode2, false);
    _rGraph.addOASGEdge(1, 1, lay1_detNode2, _rGraph.targetOASGNode(1,1,1), false);

    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), _rGraph.targetOASGNode(2,0,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.sourceOASGNode(2,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode1, lay1_obsNode4, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode4, _rGraph.targetOASGNode(2,1,1), false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode1, false);
    _rGraph.addOASGEdge(2, 1, _rGraph.targetOASGNode(2,0,1), lay1_obsNode2, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode2, lay1_obsNode3, false);
    _rGraph.addOASGEdge(2, 1, lay1_obsNode3, _rGraph.targetOASGNode(2,1,1), false);

    // layer2
    cerr << "layer2..." << endl;
    OASGNode* lay2_net0_obsNode1 = _rGraph.addOASGNode(0, 8, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode2 = _rGraph.addOASGNode(0, 32, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode3 = _rGraph.addOASGNode(0, 32, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net0_obsNode4 = _rGraph.addOASGNode(0, 8, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode1 = _rGraph.addOASGNode(1, 8, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode2 = _rGraph.addOASGNode(1, 32, 24, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode3 = _rGraph.addOASGNode(1, 32, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_net1_obsNode4 = _rGraph.addOASGNode(1, 8, 32, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay2_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);

    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode1, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode1, lay2_net0_obsNode4, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode4, _rGraph.targetOASGNode(0,0,2), false);
    _rGraph.addOASGEdge(0, 2, _rGraph.sourceOASGNode(0,2), lay2_net0_obsNode2, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode2, lay2_net0_obsNode3, false);
    _rGraph.addOASGEdge(0, 2, lay2_net0_obsNode3, _rGraph.targetOASGNode(0,0,2), false);
    
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode1, lay2_net1_obsNode4, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode4, _rGraph.targetOASGNode(1,0,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), lay2_net1_obsNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode2, lay2_net1_obsNode3, false);
    _rGraph.addOASGEdge(1, 2, lay2_net1_obsNode3, _rGraph.targetOASGNode(1,0,2), false);

    _rGraph.addOASGEdge(1, 2, _rGraph.sourceOASGNode(1,2), _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode1, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode1, _rGraph.targetOASGNode(1,1,2), false);
    _rGraph.addOASGEdge(1, 2, _rGraph.targetOASGNode(1,0,2), lay2_detNode2, false);
    _rGraph.addOASGEdge(1, 2, lay2_detNode2, _rGraph.targetOASGNode(1,1,2), false);

    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,0,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.sourceOASGNode(2,2), _rGraph.targetOASGNode(2,1,2), false);
    _rGraph.addOASGEdge(2, 2, _rGraph.targetOASGNode(2,0,2), _rGraph.targetOASGNode(2,1,2), false);

    // layer3
    cerr << "layer3..." << endl;
    OASGNode* lay3_detNode1 = _rGraph.addOASGNode(1, 40, 40, OASGNodeType::MIDDLE);
    OASGNode* lay3_detNode2 = _rGraph.addOASGNode(1, 40, 52, OASGNodeType::MIDDLE);
    _rGraph.addOASGEdge(0, 3, _rGraph.sourceOASGNode(0,3), _rGraph.targetOASGNode(0,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,0,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.sourceOASGNode(1,3), _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode1, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode1, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(1, 3, _rGraph.targetOASGNode(1,0,3), lay3_detNode2, false);
    _rGraph.addOASGEdge(1, 3, lay3_detNode2, _rGraph.targetOASGNode(1,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,0,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.sourceOASGNode(2,3), _rGraph.targetOASGNode(2,1,3), false);
    _rGraph.addOASGEdge(2, 3, _rGraph.targetOASGNode(2,0,3), _rGraph.targetOASGNode(2,1,3), false);
}

// Given three collinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool GlobalMgr::onSegment(OASGNode* p, OASGNode* q, OASGNode* r){
    if (q->x() <= max(p->x(), r->x()) && q->x() >= min(p->x(), r->x()) && 
        q->y() <= max(p->y(), r->y()) && q->y() >= min(p->y(), r->y())) 
       return true; 
  
    return false; 
}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int GlobalMgr::orientation(OASGNode* p, OASGNode* q, OASGNode* r){
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/ 
    // for details of below formula. 
    int val = (q->y() - p->y()) * (r->x() - q->x()) - 
              (q->x() - p->x()) * (r->y() - q->y()); 
  
    if (val == 0) return 0;  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
}
// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool GlobalMgr::doIntersect(OASGNode* p1, OASGNode* q1, OASGNode* p2, OASGNode* q2){
    // Find the four orientations needed for general and 
    // special cases 
    vector<OASGNode*> nodes;
    nodes.resize(4);
    int samePt = 0;
    nodes[0] = p1;
    nodes[1] = q1;
    nodes[2] = p2;
    nodes[3] = q2;
    for(int i = 0;i < 4; ++i){
        for(int j = i+1; j<4;++j){
            if(nodes[i]->x() == nodes[j]->x() && nodes[i]->y() == nodes[j]->y()){
                ++ samePt;
            }
        }
    }
    //We avoid situations that 2 lines start at same pts but not the same lines 
    if(samePt == 1){
        return false;
    }

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

bool GlobalMgr::isSegmentIntersectingWithObstacles(OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle){
    // return false;
    int numObs = obstacle.size();
    for(int i = 0; i< numObs;++i){
        int numVertices = obstacle[0].size();
        for (int j = 0; j< numVertices-1;++j){
            if(doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                addObsRoundEdges[i] = true;
                return true;
            }
        }
        if(doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])){
            addObsRoundEdges[i] = true;
            return true;
        } 
    }
    return false;
}

//Bug!
//現在改掉Obstacle加上Round Edge的Bug
//但是Via的Edges也會用這個function，所以之後假如有Edge撞到其他Obs就會破

void GlobalMgr::connectWithObstacle(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<OASGNode*> > obstacle){
    
    // 紀錄這兩個點跟哪兩個
    
    int numObs = obstacle.size();
    
    bool obs1IsTested = false;
    bool alreadyDealWtihAObs = false;
    
    bool inTouchWithThisObs = false;
    for(int i = 0; i< numObs;++i){
        if(alreadyDealWtihAObs == true){
            break;
        }
        inTouchWithThisObs = false;
        OASGNode * obs1A;
        OASGNode * obs1B;
        OASGNode * obs2A;
        OASGNode * obs2B;

        int numVertices = obstacle[i].size();
        
        for (int j = 0; j< numVertices; j++){
            
            if(j == (numVertices-1)){
                if(!obs1IsTested && doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])){
                    inTouchWithThisObs = true;
                    obs1A = obstacle[i][0];
                    obs1B = obstacle[i][numVertices-1];
                    obs1IsTested = true;
                }
                else if(obs1IsTested && doIntersect(a, b, obstacle[i][0], obstacle[i][numVertices-1])){
                    inTouchWithThisObs = true;
                    obs2A = obstacle[i][0];
                    obs2B = obstacle[i][numVertices-1];
                }
            }
            else{
                if(!obs1IsTested && doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                    inTouchWithThisObs = true;
                    obs1A = obstacle[i][j];
                    obs1B = obstacle[i][j+1];
                    obs1IsTested = true;
                }
                else if(obs1IsTested && doIntersect(a, b, obstacle[i][j], obstacle[i][j+1])){
                    inTouchWithThisObs = true;
                    obs2A = obstacle[i][j];
                    obs2B = obstacle[i][j+1];
                }
            }
        }
        
        if(inTouchWithThisObs ==true){
            alreadyDealWtihAObs = true;

            // for(int nodeId = 0; nodeId < (obstacle[i].size()-1); ++nodeId){
            //     _rGraph.addOASGEdge(netId, layerId, obstacle[i][nodeId], obstacle[i][nodeId+1], false);
            // }
            // _rGraph.addOASGEdge(netId, layerId, obstacle[i][0], obstacle[i][(obstacle[i].size()-1)], false);

            double scanX = a->x();
            double scanY = a->y(); 
            double dis1Aa = sqrt(pow(obs1A->x() - scanX, 2) + pow(obs1A->y() - scanY, 2));
            double dis1Ba = sqrt(pow(obs1B->x() - scanX, 2) + pow(obs1B->y() - scanY, 2));
            double dis2Aa = sqrt(pow(obs2A->x() - scanX, 2) + pow(obs2A->y() - scanY, 2));
            double dis2Ba = sqrt(pow(obs2B->x() - scanX, 2) + pow(obs2B->y() - scanY, 2));
            double minDis = std::min({dis1Aa , dis2Aa, dis1Ba, dis2Ba}); 
            if (minDis == dis1Aa || minDis == dis1Ba){
                if (isSegmentIntersectingWithObstacles( a, obs1A, obstacle) && !edgeExist(netId, layerId, a, obs1A)){
                    // connectWithObstacle(netId, layerId, a, obs1A, obstacle);
                }
                else if(!edgeExist(netId, layerId, a, obs1A)){
                    _rGraph.addOASGEdge(netId, layerId, a, obs1A, false);
                }
                if (isSegmentIntersectingWithObstacles( a, obs1B, obstacle) && !edgeExist(netId, layerId, a, obs1B) ){
                    // connectWithObstacle(netId, layerId, a, obs1B, obstacle);
                }
                else if(!edgeExist(netId, layerId, a, obs1B)){
                    _rGraph.addOASGEdge(netId, layerId, a, obs1B, false);
                }
                if (isSegmentIntersectingWithObstacles( b, obs2A, obstacle) && !edgeExist(netId, layerId, b, obs2A) ){
                    // connectWithObstacle(netId, layerId, b, obs2A, obstacle);
                }
                else if(!edgeExist(netId, layerId, b, obs2A)){
                    _rGraph.addOASGEdge(netId, layerId, b, obs2A, false);
                }
                if (isSegmentIntersectingWithObstacles( b, obs2B, obstacle)&& !edgeExist(netId, layerId, b, obs2B)){
                    // connectWithObstacle(netId, layerId, b, obs2B, obstacle);
                }
                else if(!edgeExist(netId, layerId, b, obs2B)){
                    _rGraph.addOASGEdge(netId, layerId, b, obs2B, false);
                }
            }
            else{
                if (isSegmentIntersectingWithObstacles( b, obs1A, obstacle) && !edgeExist(netId, layerId, b, obs1A)){
                    // connectWithObstacle(netId, layerId, b, obs1A, obstacle);
                }
                else if(!edgeExist(netId, layerId, b, obs1A)){
                    _rGraph.addOASGEdge(netId, layerId, b, obs1A, false);
                }
                if (isSegmentIntersectingWithObstacles( b, obs1B, obstacle) && !edgeExist(netId, layerId, b, obs1B)){
                    // connectWithObstacle(netId, layerId, b, obs1B, obstacle);
                }
                else if(!edgeExist(netId, layerId, b, obs1B)){
                    _rGraph.addOASGEdge(netId, layerId, b, obs1B, false);
                }
                if (isSegmentIntersectingWithObstacles( a, obs2A, obstacle) && !edgeExist(netId, layerId, a, obs2A)){
                    // connectWithObstacle(netId, layerId, a, obs2A, obstacle);
                }
                else if(!edgeExist(netId, layerId, a, obs2A)){
                    _rGraph.addOASGEdge(netId, layerId, a, obs2A, false);
                }
                if (isSegmentIntersectingWithObstacles( a, obs2B, obstacle) && !edgeExist(netId, layerId, a, obs2B)){
                    // connectWithObstacle(netId, layerId, a, obs2B, obstacle);
                }
                else if(!edgeExist(netId, layerId, a, obs2B)){
                    _rGraph.addOASGEdge(netId, layerId, a, obs2B, false);
                }
            }
        }
    }
}

bool GlobalMgr::checkWithVias(int netId, int layerId, OASGNode* a, OASGNode* b, vector<vector<vector<OASGNode*>>> viaOASGNodes){


    bool edgeTouchVia = false;
    for(int i = 0; i < viaOASGNodes.size();++i){
        if(netId == i) continue;
        int numVias = viaOASGNodes[i].size();
        if(isSegmentIntersectingWithObstacles(a,b,viaOASGNodes[i])){

            connectWithObstacle(netId, layerId, a,b,viaOASGNodes[i]);
            edgeTouchVia = true;
        }
    }    
    return edgeTouchVia;
}

bool GlobalMgr::edgeExist(int netId, int layerId, OASGNode* a, OASGNode* b){

    //If true，則不新增new Edges
    int minX = min(a->x(), b->x());
    int maxX = max(a->x(), b->x());
    int minY = min(a->y(), b->y());
    int maxY = max(a->y(), b->y());
    std::array<int, 4> newElement = {minX, maxX, minY, maxY};

    bool elementExists = false;
    for (const auto& element : alreadyAddedEdges) {
        if (element == newElement) {
            elementExists = true;
            break;
        }
    }
    if (elementExists) {
        std::cout << "新元素已存在，不添加。" << std::endl;
    } else {
        // 添加新元素
        alreadyAddedEdges.push_back(newElement);
        std::cout << "新元素已添加。" << std::endl;
    }
    
    return elementExists;

}


//Bug 1: Now obstacle are creating OASG nodes with fixed number(4 for rectangle)
//Therefore, there will be bug when encountering polygon with number of vertices greater than 4.
//Bug 2: Now there's only one obstacle, when we have more than 1, we will create redundant obstacle loop for other obstacles that is not connected.

void GlobalMgr::buildOASG(bool case5) {
    // TODO for Lo:
    // for each layer, for each net, use addOASGNode() and addOASGEdge() to construct a crossing OASG
    // in the later stage, all possible paths from the source to target ports and from target ports to lower-voltage target ports will be searched by DFS
    // so the OASGEdges should point from the source to the target ports or from higher-voltage to lower-voltage target ports all along
    cout << "########################################\n";
    cout << "Build OASG Start \n";
    cout << "########################################\n";

    //Case 5 來不及debug 先加入一個手拉的
    // bool case5 = false;
    
    //Create viaOASGNodes
    //3 dim, 1dim =  netId, 2dim = Source vias and then targets' vias, 3dim = 4points
    vector<vector<vector<OASGNode*> > > viaOASGNodes;
    if (case5 == false) {
    viaOASGNodes.resize(_rGraph.numNets());
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        int viaNodeId = 0;
        vector<vector<OASGNode*>> tempViaOASGNodes;
        tempViaOASGNodes.resize((1+_db.vNet(netId)->numTPorts()), vector<OASGNode*>(4, nullptr));
        // Step1: Create Source Via        
        double minX = _db.vNet(netId)->sourcePort()->boundPolygon()->minX();
        double minY = _db.vNet(netId)->sourcePort()->boundPolygon()->minY();
        double maxX = _db.vNet(netId)->sourcePort()->boundPolygon()->maxX();
        double maxY = _db.vNet(netId)->sourcePort()->boundPolygon()->maxY();
        tempViaOASGNodes[viaNodeId][0] = _rGraph.addOASGNode(netId, minX, minY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][1] = _rGraph.addOASGNode(netId, maxX, minY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][2] = _rGraph.addOASGNode(netId, maxX, maxY, OASGNodeType::MIDDLE);
        tempViaOASGNodes[viaNodeId][3] = _rGraph.addOASGNode(netId, minX, maxY, OASGNodeType::MIDDLE);
        ++viaNodeId ;
        //Secondly, check with the target viaclusters
        for(size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            double minX = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->minX();
            double minY = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->minY();
            double maxX = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->maxX();
            double maxY = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->maxY();
            tempViaOASGNodes[viaNodeId][0] = _rGraph.addOASGNode(netId, minX, minY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][1] = _rGraph.addOASGNode(netId, maxX, minY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][2] = _rGraph.addOASGNode(netId, maxX, maxY, OASGNodeType::MIDDLE);
            tempViaOASGNodes[viaNodeId][3] = _rGraph.addOASGNode(netId, minX, maxY, OASGNodeType::MIDDLE);
            ++viaNodeId;
        }
        viaOASGNodes[netId] = tempViaOASGNodes;
    }
    }

    for (size_t layerId = 0; layerId < _rGraph.numLayers(); ++ layerId){
        //Step 0: 先把每層的Middle 的OASG Node建完(因為每條Net的OASG Node都不一樣，所以直接包在裡面)
        //Obs Node的順序是1左下、2右下、3右上、4左上
        for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId){

            // if(netId == 2 && case5 == true){
            //     _rGraph.addOASGEdge(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), _rGraph.targetOASGNode(netId, 0,layerId), false);
            //     _rGraph.addOASGEdge(netId, layerId, _rGraph.targetOASGNode(netId, 0,layerId), _rGraph.targetOASGNode(netId, 1,layerId), false);
            //     double maxX = _db.vNet(0)->targetPort(0)->boundPolygon()->maxX();
            //     double maxY = _db.vNet(0)->targetPort(0)->boundPolygon()->maxY();
            //     OASGNode* obsNode = _rGraph.addOASGNode(2, maxX, maxY, OASGNodeType::MIDDLE);
            //     _rGraph.addOASGEdge(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), obsNode, false);
            //     _rGraph.addOASGEdge(netId, layerId, obsNode, _rGraph.targetOASGNode(netId, 1,layerId), false);
            //     continue;
            // }

            vector<vector<OASGNode*>> obsNodes;
            //如果這個Obs已經有要加Round Edges，就變成True 
            addObsRoundEdges.resize(_db.numObstacles(layerId));
            for (int i = 0; i < _db.numObstacles(layerId); ++i){
                addObsRoundEdges[i] = false;
            }

            //這裡要判斷OASG Edges有沒有重複加入
            //要確認這裡有沒有resize成功
            alreadyAddedEdges.clear();
            int numOfObsNodes = 0;
            for (int i = 0; i < _db.numObstacles(layerId);++i){
                numOfObsNodes += _db.vObstacle(layerId, i)->vShape(0)->numBPolyVtcs();
            }
            int size_alreadyAddEdges = 1 + _db.vNet(netId)->numTPorts() + numOfObsNodes;
            alreadyAddedEdges.resize(size_alreadyAddEdges * (size_alreadyAddEdges-1));


            bool thisLayerHaveObs = false;
            bool thisNetTouchObsThisLayer = false;

            if(_db.numObstacles(layerId)>0){

                thisLayerHaveObs = true;

                obsNodes.resize(_db.numObstacles(layerId));
                for (int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                    int numPolyVtcs = _db.vObstacle(layerId, obsId)->vShape(0)->numBPolyVtcs();
                    obsNodes[obsId].resize(numPolyVtcs);

                    double tempX, tempY;
                    for(int j = 0; j < numPolyVtcs; ++j){
                        tempX =  _db.vObstacle(layerId, obsId)->vShape(0)->bPolygonX(j);
                        tempY =  _db.vObstacle(layerId, obsId)->vShape(0)->bPolygonY(j);
                        obsNodes[obsId][j] = _rGraph.addOASGNode(netId, tempX, tempY, OASGNodeType::MIDDLE);
                    }
                    
                }
            }

            // int numScanNode = 1 + _db.vNet(netId)->numTPorts() + (4 * _db.numObstacles(layerId));
            int numScanNode = 1 + _db.vNet(netId)->numTPorts(); 
            vector<OASGNode*> traverseNodes;
            traverseNodes.push_back(_rGraph.sourceOASGNode(netId,layerId));
            for (int netTPortId = 0; netTPortId < _db.vNet(netId)->numTPorts(); netTPortId++){
                traverseNodes.push_back(_rGraph.targetOASGNode(netId,netTPortId,layerId));
            }

            //OASG Source
            for (size_t currentScanNodeId = 0; currentScanNodeId < numScanNode; ++ currentScanNodeId){
                // Current Scan Node Id: Source is 0; 1 ~ is target ports
                if (currentScanNodeId == 0){
                    double scanX = _rGraph.sourceOASGNode(netId,layerId)->x();
                    double scanY = _rGraph.sourceOASGNode(netId,layerId)->y();

                    // for (int i = 1;i < numScanNode; ++i){
                        int i = 1;
                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();

                        if (isSegmentIntersectingWithObstacles(_rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], obsNodes)){

                            connectWithObstacle(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], obsNodes);
                            thisNetTouchObsThisLayer = true;
                        }
                        else {
                            if(!checkWithVias(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i],viaOASGNodes)){
                                if(!edgeExist(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i])){
                                    _rGraph.addOASGEdge(netId, layerId, _rGraph.sourceOASGNode(netId,layerId), traverseNodes[i], false);
                                }   
                            }
                        }
                    // }

                }
                
                //開始處理Target
                
                else if(currentScanNodeId > 0 && currentScanNodeId <= _db.vNet(netId)->numTPorts()){
                    // cout << "Start building for targets" << endl ;
                    double scanX = traverseNodes[currentScanNodeId]->x();
                    double scanY = traverseNodes[currentScanNodeId]->y();

                    for (int i = 1;i < numScanNode-1; i++ ){
                        if(i <= currentScanNodeId) continue;

                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();
                        if(curX >= scanX && curY >= scanY){
                            if (isSegmentIntersectingWithObstacles(traverseNodes[i], traverseNodes[i+1], obsNodes)){

                                connectWithObstacle(netId, layerId, traverseNodes[i], traverseNodes[i+1], obsNodes);
                                thisNetTouchObsThisLayer = true;
                            }
                            else {
                                if(!checkWithVias(netId, layerId, traverseNodes[i], traverseNodes[i+1],viaOASGNodes)){
                                    if(!edgeExist(netId, layerId, traverseNodes[i], traverseNodes[i+1])){
                                        _rGraph.addOASGEdge(netId, layerId, traverseNodes[i], traverseNodes[i+1], false);
                                    }
                                }
                            }
                        }
                        if(i == 1){
                            curX = traverseNodes[numScanNode-1]-> x();
                            curY = traverseNodes[numScanNode-1]-> y();
                            if(curX >= scanX && curY >= scanY){
                                if (isSegmentIntersectingWithObstacles(traverseNodes[1], traverseNodes[numScanNode-1], obsNodes)){

                                    connectWithObstacle(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], obsNodes);
                                    thisNetTouchObsThisLayer = true;
                                }
                                else {
                                    if(!checkWithVias(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1],viaOASGNodes)){
                                        if(!edgeExist(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1])){
                                            _rGraph.addOASGEdge(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], false);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    for (int i = 1;i < numScanNode-1; i++ ){
                        if(i == currentScanNodeId) continue;
                        double curX = traverseNodes[i]-> x();
                        double curY = traverseNodes[i]-> y();
                        //後面是要判斷他們不是同一個點

                        if(curX >= scanX && curY <= scanY){
                            if (isSegmentIntersectingWithObstacles(traverseNodes[i], traverseNodes[i+1], obsNodes)){
                                connectWithObstacle(netId, layerId, traverseNodes[i], traverseNodes[i+1], obsNodes);
                                thisNetTouchObsThisLayer = true;
                            }
                            else {
                                if(!checkWithVias(netId, layerId,  traverseNodes[i], traverseNodes[i+1],viaOASGNodes)){
                                    if(!edgeExist(netId, layerId,  traverseNodes[i], traverseNodes[i+1])){
                                        _rGraph.addOASGEdge(netId, layerId,  traverseNodes[i], traverseNodes[i+1], false);
                                    }
                                }
                            }
                        }
                        if(i == 1){
                            curX = traverseNodes[numScanNode-1]-> x();
                            curY = traverseNodes[numScanNode-1]-> y();
                            if(curX >= scanX && curY >= scanY){
                                if (isSegmentIntersectingWithObstacles(traverseNodes[1], traverseNodes[numScanNode-1], obsNodes)){

                                    connectWithObstacle(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], obsNodes);
                                    thisNetTouchObsThisLayer = true;
                                }
                                else {
                                    if(!checkWithVias(netId, layerId,  traverseNodes[1], traverseNodes[numScanNode-1],viaOASGNodes)){
                                        if(!edgeExist(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1])){
                                            _rGraph.addOASGEdge(netId, layerId, traverseNodes[1], traverseNodes[numScanNode-1], false);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                
            }
            //這裡不用判斷Edge有沒有重複，因為是第一次加入
            for(int obsId = 0; obsId < _db.numObstacles(layerId); ++obsId){
                if(addObsRoundEdges[obsId] == true){
                    cout << "In Layer " << layerId << " Net " << netId << " Touched with the Xth obstacle " << obsId << endl;
                    int numPolyVtcs = _db.vObstacle(layerId, obsId)->vShape(0)->numBPolyVtcs();
                    for(int vtxId = 0; vtxId < (numPolyVtcs - 1); ++vtxId){
                        _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][vtxId], obsNodes[obsId][vtxId+1], false);
                    }
                    _rGraph.addOASGEdge(netId, layerId, obsNodes[obsId][0], obsNodes[obsId][numPolyVtcs - 1], false);
                }
            }
        }

    }

    //Check if the OASG edges are flowing in the right direction
    cout <<  _rGraph.numOASGEdges() << endl;
    cout <<  _rGraph.numOASGNodes() << endl;
    // for (size_t layId = 0; layId < _rGraph.numLayers(); ++layId ){
    //     for (size_t netId = 0; netId < _rGraph.numNets(); ++netId){
    //         //Check source first
    //         if( _rGraph.sourceOASGNode(netId, layId)->numInEdges() > 0 ){
    //             for (size_t edgeCount = 0; edgeCount < _rGraph.sourceOASGNode(netId, layId)->numInEdges(); ++edgeCount){
    //                 OASGEdge * edge = _rGraph.vOASGEdge(_rGraph.sourceOASGNode(netId, layId)->inEdgeId(edgeCount));
    //                 _rGraph.swapST(edge);
    //                 cout << "Swapped" <<endl;
    //             }
    //         }
    //         // Now check all target ports
    //         for (size_t tPortId = 0;tPortId < _rGraph.numTPorts(netId); ++tPortId ){
    //             if( _rGraph.targetOASGNode(netId, tPortId,layId)->numOutEdges() > 0 ){
    //                 for (size_t edgeCount = 0; edgeCount < _rGraph.targetOASGNode(netId, tPortId,layId)->numOutEdges(); ++edgeCount){
    //                     OASGEdge * edge = _rGraph.vOASGEdge(_rGraph.targetOASGNode(netId, tPortId, layId)->outEdgeId(edgeCount));
    //                     _rGraph.swapST(edge);
    //                     cout << "Swapped" <<endl;
    //                 }
    //             }
    //         } 
    //     }
    // }
    
    // Check All OASGEdges have the right flow dirction 
    // Determine by its distance with the source
    for (size_t edgeId = 0; edgeId < _rGraph.numOASGEdges(); ++ edgeId  ){
        OASGEdge* edge = _rGraph.vOASGEdge(edgeId);
        int netId = edge->netId();
        double disS = 0;
        double disT = 0;
        double tempX = 0, tempY = 0, sourceX = 0, sourceY = 0;
        sourceX = _rGraph.sourceOASGNode(netId, 0)->x();
        sourceY = _rGraph.sourceOASGNode(netId, 0)->y();
        
        tempX = edge->sNode()->x();
        tempY = edge->sNode()->y();
        disS = sqrt (pow((tempX-sourceX), 2) + pow((tempY-sourceY), 2));
        tempX = edge->tNode()->x();
        tempY = edge->tNode()->y();
        disT = sqrt (pow((tempX-sourceX), 2) + pow((tempY-sourceY), 2));
        if (disS > disT){
            _rGraph.swapST(edge);
        }
    }

    
    cout << "########################################" << endl;
    cout << "Finishing Building OASG" << endl;
    cout << "########################################" << endl;

}

void GlobalMgr::buildOASGXObs() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            OASGNode* sNode = _rGraph.sourceOASGNode(netId, layId);
            for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
                OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, layId);
                _rGraph.addOASGEdge(netId, layId, sNode, tNode, false);
                for (size_t tPortId1 = tPortId+1; tPortId1 < _rGraph.numTPorts(netId); ++ tPortId1) {
                    OASGNode* tNode1 = _rGraph.targetOASGNode(netId, tPortId1, layId);
                    _rGraph.addOASGEdge(netId, layId, tNode, tNode1, false);
                }
            }
        }
        
    }
}

void GlobalMgr::plotOASG() {
    //_plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId,1.0);
            }
        }
    }
}

void GlobalMgr::genCrossConstrs() {

    _rGraph.constructRGraph();

    auto addConstraint = [&] (size_t netId, size_t twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* e = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);

                // the RGEdges from other nets
                for (size_t netId1 = netId+1; netId1 < _rGraph.numNets(); ++ netId1) {
                    Port* sPort1 = _rGraph.sPort(netId1);
                    // for (size_t netTPortId1 = 0; netTPortId1 < _rGraph.numTPorts(netId1); ++ netTPortId1) {
                        size_t netTPortId1 = 0;
                        // the two pin net between the target port and the source port of net1
                        Port* tPort1 = _rGraph.tPort(netId1, netTPortId1);
                        size_t twoPinNetId1 = _rGraph.twoPinNetId(sPort1->portId(), tPort1->portId());
                        for (size_t RGEdgeId1 = 0; RGEdgeId1 < _rGraph.numRGEdges(twoPinNetId1, layId); ++ RGEdgeId1) {
                            RGEdge* e1 = _rGraph.vEdge(twoPinNetId1, layId, RGEdgeId1);
                            if (e->cross(e1)) {
                                _vCrossConstr.push_back(make_pair(e, e1));
                                // _model.addConstr(_vFlow[twoPinNetId][layId][RGEdgeId] + _vFlow[twoPinNetId1][layId][RGEdgeId1] <= 1);
                            }
                        }

                        // the two pin net between the target port and other target ports of net1
                        for (size_t netTPortId11 = netTPortId1+1; netTPortId11 < _rGraph.numTPorts(netId1); ++ netTPortId11) {
                            Port* tPort11 = _rGraph.tPort(netId1, netTPortId11);
                            size_t twoPinNetId11 = _rGraph.twoPinNetId(tPort1->portId(), tPort11->portId());
                            for (size_t RGEdgeId11 = 0; RGEdgeId11 < _rGraph.numRGEdges(twoPinNetId11, layId); ++ RGEdgeId11) {
                                RGEdge* e11 = _rGraph.vEdge(twoPinNetId11, layId, RGEdgeId11);
                                if (e->cross(e11)) {
                                    _vCrossConstr.push_back(make_pair(e, e11));
                                    // _model.addConstr(_vFlow[twoPinNetId][layId][RGEdgeId] + _vFlow[twoPinNetId11][layId][RGEdgeId11] <= 1);
                                }
                            }
                        }

                    // }
                }

            }
        }
    };

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Port* sPort = _rGraph.sPort(netId);
        // for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
            size_t netTPortId = 0;
            // the two pin net between the target port and the source port of the net
            Port* tPort = _rGraph.tPort(netId, netTPortId);
            size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
            addConstraint(netId, twoPinNetId);

            // the two pin net between the target port and other target ports of the net
            for (size_t netTPortId2 = netTPortId+1; netTPortId2 < _rGraph.numTPorts(netId); ++ netTPortId2) {
                Port* tPort2 = _rGraph.tPort(netId, netTPortId2);
                size_t twoPinNetId2 = _rGraph.twoPinNetId(tPort->portId(), tPort2->portId());
                addConstraint(netId, twoPinNetId2);
            }

        // }
    }

    // cerr << "Cross Constraints:" << endl;
    // for (size_t crossId = 0; crossId < _vCrossConstr.size() * 0.5; ++ crossId) {
    //     cerr << "cross " << crossId << endl;
    //     _vCrossConstr[crossId].first->print();
    //     _vCrossConstr[crossId].second->print();
    //     cerr << endl;
    //     for (size_t OASGEdgeId = 0; OASGEdgeId < _vCrossConstr[crossId].first->numEdges(); ++ OASGEdgeId) {
    //         OASGEdge* e1 =_vCrossConstr[crossId].first->vEdge(OASGEdgeId);
    //         if (!e1->viaEdge())
    //         _plot.drawLine(e1->sNode()->x(), e1->sNode()->y(), e1->tNode()->x(), e1->tNode()->y(), crossId, e1->layId(), 1.0);
    //     }
    //     for (size_t OASGEdgeId = 0; OASGEdgeId < _vCrossConstr[crossId].second->numEdges(); ++ OASGEdgeId) {
    //         OASGEdge* e2 =_vCrossConstr[crossId].second->vEdge(OASGEdgeId);
    //         if (!e2->viaEdge())
    //         _plot.drawLine(e2->sNode()->x(), e2->sNode()->y(), e2->tNode()->x(), e2->tNode()->y(), crossId, e2->layId(), 1.0);
    //     }
    // }
}

void GlobalMgr::layerDistribution() {
    // construct the routing graph
    // _rGraph.constructRGraph();
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

void GlobalMgr::plotRGraph() {
    // _plot.startPlot(_db.boardWidth()*_db.numLayers(), _db.boardHeight());
    for (size_t twoPinNetId = 0; twoPinNetId < _rGraph.num2PinNets(); ++ twoPinNetId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
                RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
                if (rgEdge->selected()) {
                    for (size_t OASGEdgeId = 0; OASGEdgeId < rgEdge->numEdges(); ++ OASGEdgeId) {
                        OASGEdge* e = rgEdge->vEdge(OASGEdgeId);
                        _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), e->netId(), layId, 1.0);
                    }
                }
            }
        }
    }
}

void GlobalMgr::buildTestNCOASG() {
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
                                newSNode = NCOASG->addOASGNode(sNode->netId(), sNode->x(), sNode->y(), sNode->nodeType());
                                vNewNodeId[sNode->nodeId()] = newSNode->nodeId();
                            } else {
                                newSNode = NCOASG->vOASGNode( vNewNodeId[sNode->nodeId()] );
                            }
                            // find or create a new tNode for the new edge
                            OASGNode* tNode = e->tNode();
                            OASGNode* newTNode;
                            if (vNewNodeId[tNode->nodeId()] == -1) {
                                newTNode = NCOASG->addOASGNode(tNode->netId(), tNode->x(), tNode->y(), tNode->nodeType());
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

    // set redundant nodes and edges
    for (size_t netId = 0; netId < NCOASG->numNets(); ++ netId) {
        for (int layId = NCOASG->numLayers()-1; layId >= 0; -- layId) {
            // set redundant source nodes and edges
            OASGNode* sNode = NCOASG->sourceOASGNode(netId, layId);
            if (sNode->numOutEdges() == 0) {
                sNode->setRedundant();
                NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
            } else if (sNode->numOutEdges() == 1) {
                if (NCOASG->vOASGEdge(sNode->outEdgeId(0))->redundant()) {
                    sNode->setRedundant();
                    NCOASG->vOASGEdge(sNode->inEdgeId(0))->setRedundant();
                }
            }
            // set redundant target nodes and edges
            for (size_t tPortId = 0; tPortId < NCOASG->numTPorts(netId); ++ tPortId) {
                OASGNode* tNode = NCOASG->targetOASGNode(netId, tPortId, layId);
                if (tNode->numInEdges() == 0) {
                    tNode->setRedundant();
                    NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                } else if (tNode->numInEdges() == 1) {
                    if (NCOASG->vOASGEdge(tNode->inEdgeId(0))->redundant()) {
                        tNode->setRedundant();
                        NCOASG->vOASGEdge(tNode->outEdgeId(0))->setRedundant();
                    }
                }
            }
        }
    }
    _rGraph = *NCOASG;
}

void GlobalMgr::plotNCOASG() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId, 1.0);
            }
        }
    }
}

void GlobalMgr::voltCurrOpt() {
    // store capacity constraints
    // struct CapConstr {
    //     OASGEdge* e1;
    //     bool right1;
    //     double ratio1;
    //     OASGEdge* e2;
    //     bool right2;
    //     double ratio2;
    //     double width;
    // };
    // struct SingleCapConstr {
    //     OASGEdge* e1;
    //     bool right1;
    //     double ratio1;
    //     double width;
    // };
    // vector<CapConstr> vCapConstr;
    // vector<SingleCapConstr> vSglCapConstr;
    // auto addCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
    //     CapConstr capConstr = {e1, right1, ratio1, e2, right2, ratio2, width};
    //     vCapConstr.push_back(capConstr);
    // };
    // auto addSglCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, double width) {
    //     SingleCapConstr sglCapConstr = {e1, right1, ratio1, width};
    //     vSglCapConstr.push_back(sglCapConstr);
    // };
    // // set capacity constraints
    // // TODO for Tsai and Huang:
    // // for each layer, for each neighboring OASGEdges,
    
    
    // //search each layer                                                                           
    // for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
    //     cout << "LAYER :" << layId << endl << endl;
    //     //search each net
    //     for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
    //         //search each edge
    //         for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
    //             OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                
    //             pair<double, double> ratio;
    //             pair<bool, bool> right;
    //             double width;
                
    //             //compare other net edge
    //             for(size_t T_netId = S_netId+1; T_netId < _rGraph.numNets(); ++ T_netId){
    //                 //make sure to compare different net
    //                 for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
    //                     OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
    
    //                 //    BuildCapacityConstraint(e1,e2,solver);
    //                     if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                                      make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                                      make_pair(e2->sNode()->x(), e2->sNode()->y()),
    //                                      make_pair(e2->tNode()->x(), e2->tNode()->y()),
    //                                      ratio, right, width))
    //                         addCapConstr(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                        
    //                 }   
    //             } 

    //             // obstacle constraint
    //             for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                   
    //                 Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
    //                 pair<double, double> S2, T2;
    //                 for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
    //                     for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
    //                         // get the edge coordinates
    //                         S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
    //                         T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));
                        
    //                         if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                                          make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                                          S2, T2, ratio, right, width))
    //                             addSglCapConstr(e1, right.first, ratio.first, width);

    //                     }
    //                 }
    //             }

    //             // board cnstraint
    //             // bottom
    //             if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                              make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                              make_pair(0, 0),
    //                              make_pair(_db.boardWidth(), 0),
    //                              ratio, right, width))
    //                 addSglCapConstr(e1, right.first, ratio.first, width);
    //             // left
    //             if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                              make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                              make_pair(0, 0),
    //                              make_pair(0, _db.boardHeight()),
    //                              ratio, right, width))
    //                 addSglCapConstr(e1, right.first, ratio.first, width);
    //             // top
    //             if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                              make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                              make_pair(0, _db.boardHeight()),
    //                              make_pair(_db.boardWidth(), _db.boardHeight()),
    //                              ratio, right, width))
    //                 addSglCapConstr(e1, right.first, ratio.first, width);
    //             // right
    //             if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
    //                              make_pair(e1->tNode()->x(), e1->tNode()->y()),
    //                              make_pair(_db.boardWidth(), 0),
    //                              make_pair(_db.boardWidth(), _db.boardHeight()),
    //                              ratio, right, width))
    //                 addSglCapConstr(e1, right.first, ratio.first, width);
    //         }
    //     }  
    // }

    // voltageAssignment(true);
 
    voltageDemandAssignment();
    // swapSTbyVolt();
    // voltageDemandAssignment();

    vector<double> vMediumLayerThickness;
    vector<double> vMetalLayerThickness;
    vector<double> vConductivity;
    for (size_t mediumLayId = 0; mediumLayId < _db.numMediumLayers(); ++ mediumLayId) {
        vMediumLayerThickness.push_back(_db.vMediumLayer(mediumLayId)->thickness());
    }
    for (size_t metalLayId = 0; metalLayId < _db.numLayers(); ++ metalLayId) {
        vMetalLayerThickness.push_back(_db.vMetalLayer(metalLayId)->thickness());
        vConductivity.push_back(_db.vMetalLayer(metalLayId)->conductivity());
    }
    double normRatio = _rGraph.sPort(0)->current();
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        if (normRatio > _rGraph.sPort(netId)->current()) {
            normRatio = _rGraph.sPort(netId)->current();
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            if (normRatio > _rGraph.tPort(netId, tPortId)->current()) {
                normRatio = _rGraph.tPort(netId, tPortId)->current();
            }
        }
    }

    FlowLP* currentSolver;
    // VoltCP* voltageSolver;
    VoltSLP* voltageSolver;
    vector<double> vLambda(_vCapConstr.size(), 2.0);
    vector<double> vNetLambda(_vNetCapConstr.size(), 4.0);
    vector<double> vLastOverlap(_vCapConstr.size(), 0.0);
    vector<double> vDiffLastOverlap;
    double PRatio = 10.0;
    double DRatio = 1.0;
    

    // cerr << "Check vEdgeId..." << endl;
    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     assert(_rGraph.vViaOASGEdge(netId, 0, 0)->sNode()->port() == _db.vNet(netId)->sourcePort());
    //     for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
    //         assert(_rGraph.vViaOASGEdge(netId, 0, tPortId+1)->tNode()->port() == _db.vNet(netId)->targetPort(tPortId));
    //     }
    // }
    // assert(false);

    // cout << numIIter << "  " << numVIter << "  " << numIVIter << endl;


    for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
                vNetLambda[netCapId] = 1;
    }
    for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
            vLambda[capId] =  4;
        }

    //Change for into while, add early stop for all three loops
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        
        
        // current optimization
        // currentSolver = new FlowLP(_rGraph, vMediumLayerThickness, vMetalLayerThickness, vConductivity, normRatio);
        currentSolver = new FlowLP(_db, _rGraph);
        currentSolver->setObjective(_db.areaWeight(), _db.viaWeight(), 0.1);
        currentSolver->setConserveConstraints(true);
        // currentSolver->addViaAreaConstraints
        for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
            for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                cerr << "_vUBViaArea[" << netId << "][" << vEdgeId << "] = " << _vUBViaArea[netId][vEdgeId] << endl;
                currentSolver->addViaAreaConstraints(netId, vEdgeId, _vUBViaArea[netId][vEdgeId]);
            }
        }
        for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
            CapConstr cap = _vCapConstr[capId];
            currentSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
        }
        for (size_t sglCapId = 0; sglCapId < _vSglCapConstr.size(); ++ sglCapId) {
            SingleCapConstr sglCap = _vSglCapConstr[sglCapId];
            currentSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
        }
        for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
            CapConstr cap = _vNetCapConstr[netCapId];
            currentSolver->addSameNetCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
        }
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            currentSolver->clearVOverlap();
            for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
                CapConstr cap = _vCapConstr[capId];
                currentSolver->addCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width, true);
            }
            for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
                CapConstr cap = _vNetCapConstr[netCapId];
                currentSolver->addSameNetCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width, true);
            }
            currentSolver->relaxCapacityConstraints(vLambda, vNetLambda);
            currentSolver->solveRelaxed();
            currentSolver->collectRelaxedResult();
            for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
                CapConstr cap = _vCapConstr[capId];
                currentSolver->addCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width, false);
            }
            for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
                CapConstr cap = _vNetCapConstr[netCapId];
                currentSolver->addSameNetCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width, false);
            }
            currentSolver->calculateOverlapCost(vLambda, vNetLambda);
            _vArea.push_back(currentSolver->area());
            _vViaArea.push_back(currentSolver->viaArea());
            _vOverlap.push_back(currentSolver->overlap());
            _vSameNetOverlap.push_back(currentSolver->sameNetOverlap());
            _vBeforeCost.push_back(currentSolver->beforeCost());
            _vAfterCost.push_back(currentSolver->afterCost());
            _vBeforeOverlapCost.push_back(currentSolver->beforeOverlapCost());
            _vAfterOverlapCost.push_back(currentSolver->afterOverlapCost());
            cerr << "iIter = " << iIter << endl;
            currentSolver->printRelaxedResult();
            // lagrange multiplier scheduling
            for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
                // schedule1: exp(exp(.))
                // vLambda[capId] *= vLambda[capId];

                // schedule2: exp(.)
                vLambda[capId] *= 1.1;

                // schedule3: P control
                // vLambda[capId] += PRatio * currentSolver->vOverlap(capId);

                // schedule4: PD control
                // double curOverlap = currentSolver->vOverlap(capId);
                // double diffOverlap = 0;
                // if (iIter > 0) {
                //     diffOverlap = vLastOverlap[capId] - curOverlap;
                // }
                // vLambda[capId] += PRatio * curOverlap + DRatio * diffOverlap;
                // vLastOverlap[capId] = curOverlap;
            }
            for (size_t netCapId = 0; netCapId <_vNetCapConstr.size(); ++ netCapId) {
                //調
                vNetLambda[netCapId] *= 1;
            }
        }

        // voltage optimization
        // vector< vector< double > > vOldVoltage;
        // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        //     vector<double> temp;
        //     for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
        //         temp.push_back(_rGraph.vNPortOASGNode(netId, nPortNodeId)->voltage());
        //         // vOldVoltage[netId][nPortNodeId] = _rGraph.vNPortOASGNode(netId, nPortNodeId)->voltage();
        //     }
        //     vOldVoltage.push_back(temp);
        // }
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            // voltageSolver = new VoltSLP(_db, _rGraph, vOldVoltage);
            voltageSolver = new VoltSLP(_db, _rGraph);
            voltageSolver->setObjective(_db.areaWeight(), _db.viaWeight());
            // voltageSolver->setVoltConstraints(1E-15);
            voltageSolver->setLimitConstraint(0.9);
            for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
                for (size_t vEdgeId = 0; vEdgeId < _rGraph.numViaOASGEdges(netId); ++ vEdgeId) {
                    voltageSolver->addViaAreaConstraints(netId, vEdgeId, _vUBViaArea[netId][vEdgeId]);
                }
            }
            for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
                CapConstr cap = _vCapConstr[capId];
                voltageSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
            }
            for (size_t sglCapId = 0; sglCapId < _vSglCapConstr.size(); ++ sglCapId) {
                SingleCapConstr sglCap = _vSglCapConstr[sglCapId];
                voltageSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
            }
            for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
                CapConstr cap = _vNetCapConstr[netCapId];
                voltageSolver->addSameNetCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
            }
            voltageSolver->relaxCapacityConstraints(vLambda, vNetLambda);
            voltageSolver->solveRelaxed();
            voltageSolver->collectRelaxedResult();
            for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
                CapConstr cap = _vCapConstr[capId];
                voltageSolver->addCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
            }
            for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
                CapConstr cap = _vNetCapConstr[netCapId];
                voltageSolver->addSameNetCapacityOverlap(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
            }
            voltageSolver->calculateOverlapCost(vLambda, vNetLambda);
            _vArea.push_back(voltageSolver->area());
            _vViaArea.push_back(voltageSolver->viaArea());
            _vOverlap.push_back(voltageSolver->overlap());
            _vSameNetOverlap.push_back(voltageSolver->sameNetOverlap());
            _vBeforeCost.push_back(voltageSolver->beforeCost());
            _vAfterCost.push_back(voltageSolver->afterCost());
            _vBeforeOverlapCost.push_back(voltageSolver->beforeOverlapCost());
            _vAfterOverlapCost.push_back(voltageSolver->afterOverlapCost());
            cerr << "vIter = " << vIter << endl;
            voltageSolver->printRelaxedResult();
            // voltageSolver->collectRelaxedTempVoltage();
            // vOldVoltage = voltageSolver->vNewVoltage();
        }
        // voltageSolver->collectRelaxedResult();
        // // cerr << "vIter = " << vIter << endl;
        // voltageSolver->printRelaxedResult();

        // voltage optimization
        // voltageSolver = new VoltCP(_db, _rGraph);
        // voltageSolver->setObjective(_db.areaWeight(), _db.viaWeight());
        // voltageSolver->setVoltConstraints(1E-10);
        // for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
        //     CapConstr cap = vCapConstr[capId];
        //     voltageSolver->addCapacityConstraints(cap.e1, cap.right1, cap.ratio1, cap.e2, cap.right2, cap.ratio2, cap.width);
        // }
        // for (size_t sglCapId = 0; sglCapId < vSglCapConstr.size(); ++ sglCapId) {
        //     SingleCapConstr sglCap = vSglCapConstr[sglCapId];
        //     voltageSolver->addCapacityConstraints(sglCap.e1, sglCap.right1, sglCap.ratio1, sglCap.width);
        // }
        // for (size_t vIter = 0; vIter < 1; ++ vIter) {
        //     voltageSolver->relaxCapacityConstraints(vLambda);
        //     voltageSolver->solveRelaxed();
        //     voltageSolver->collectRelaxedResult();
        //     cerr << "vIter = " << vIter << endl;
        //     voltageSolver->printRelaxedResult();
        //     // for (size_t capId = 0; capId < vCapConstr.size(); ++ capId) {
        //     //     vLambda[capId] *= vLambda[capId];
        //     // }
        // }

    }

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    // segment->plot(netId, layId);
                    _db.vNet(netId)->addSegment(segment, layId);
                // }
            }
        }
    }

    // add viaArea of each port
    // vias will be added in detailed routing
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        assert(_rGraph.vViaOASGEdge(netId, 0, 0)->sNode()->port() == _db.vNet(netId)->sourcePort());
        double sViaArea = _rGraph.vViaOASGEdge(netId, 0, 0)->viaArea();
        assert(sViaArea >= 0);
        _db.vNet(netId)->sourcePort()->setViaArea(sViaArea);
        cerr << "net" << netId << " s: viaArea = " << sViaArea << ", upperbound = " << _vUBViaArea[netId][0] << endl;
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            assert(_rGraph.vViaOASGEdge(netId, 0, tPortId+1)->tNode()->port() == _db.vNet(netId)->targetPort(tPortId));
            double tViaArea = _rGraph.vViaOASGEdge(netId, 0, tPortId+1)->viaArea();
            assert(tViaArea >= 0);
            _db.vNet(netId)->targetPort(tPortId)->setViaArea(tViaArea);
            cerr << "net" << netId << " t" << tPortId << ": viaArea = " << tViaArea << ", upperbound = " << _vUBViaArea[netId][tPortId+1] << endl;
        }
    }

    // print the recorded area and overlapped width
    cerr << "////////////////" << endl;
    cerr << "//    area    //" << endl;
    cerr << "////////////////" << endl;
    size_t i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vArea[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vArea[i] << " -> ";
            i++;
        }
        cerr << endl;
    }
    cerr << "///////////////////" << endl;
    cerr << "//    viaArea    //" << endl;
    cerr << "///////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vViaArea[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vViaArea[i] << " -> ";
            i++;
        }
        cerr << endl;
    }
    cerr << "////////////////////////////" << endl;
    cerr << "//    overlapped width    //" << endl;
    cerr << "////////////////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
    }
    cerr << "/////////////////////////////////////" << endl;
    cerr << "//    same net overlapped width    //" << endl;
    cerr << "/////////////////////////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << _vSameNetOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << _vSameNetOverlap[i] << " -> ";
            i++;
        }
        cerr << endl;
    }
    cerr << "//////////////////////" << endl;
    cerr << "//    Total Cost    //" << endl;
    cerr << "//////////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << "(" << _vBeforeCost[i] << " -> " << _vAfterCost[i] << ") => ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << "(" << _vBeforeCost[i] << " -> " << _vAfterCost[i] << ") => ";
            i++;
        }
        cerr << endl;
    }
    cerr << "////////////////////////" << endl;
    cerr << "//    Overlap Cost    //" << endl;
    cerr << "////////////////////////" << endl;
    i = 0;
    for (size_t ivIter = 0; ivIter < numIVIter; ++ ivIter) {
        cerr << "ivIter = " << ivIter << endl;
        cerr << "I opt: ";
        for (size_t iIter = 0; iIter < numIIter; ++iIter) {
            cerr << "(" << _vBeforeOverlapCost[i] << " -> " << _vAfterOverlapCost[i] << ") => ";
            i++;
        }
        cerr << endl;
        cerr << "V opt: ";
        for (size_t vIter = 0; vIter < numVIter; ++ vIter) {
            cerr << "(" << _vBeforeOverlapCost[i] << " -> " << _vAfterOverlapCost[i] << ") => ";
            i++;
        }
        cerr << endl;
    }
    // assert(false);
}

void GlobalMgr::voltageAssignment(bool currentBased) {
    // auto viaEdgeArea = [&] (OASGEdge* e) -> double {
    //     assert(e->viaEdge());
    //     double polygonArea = e->boundPolygon()->area();
    //     double lowLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId());
    //     double upLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId()+1);
    //     double ratio = _db.VIA16D8A24()->metalArea() / pow(max(lowLayPadRadius, upLayPadRadius), 2);
    //     return polygonArea * ratio;
    // };

    for (size_t sCapId = 0; sCapId < _vSglCapConstr.size(); ++ sCapId) {
        OASGEdge* sglEdge = _vSglCapConstr[sCapId].e1;
        double sglWidth = _vSglCapConstr[sCapId].width / _vSglCapConstr[sCapId].ratio1;
        bool sglRight = _vSglCapConstr[sCapId].right1;
        if (sglRight) {
            if (sglWidth < sglEdge->widthRight()) {
                sglEdge->setWidthRight(sglWidth);
            }
        } else {
            if (sglWidth < sglEdge->widthLeft()) {
                sglEdge->setWidthLeft(sglWidth);
            }
        }
    }
    for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
        OASGEdge* e1 = _vCapConstr[capId].e1;
        OASGEdge* e2 = _vCapConstr[capId].e2;
        double width1 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio1;
        double width2 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio2;
        bool right1 = _vCapConstr[capId].right1;
        bool right2 = _vCapConstr[capId].right2;
        if (right1) {
            if (width1 < e1->widthRight()) {
                e1->setWidthRight(width1);
            }
        } else {
            if (width1 < e1->widthLeft()) {
                e1->setWidthLeft(width1);
            }
        }
        if (right2) {
            if (width2 < e2->widthRight()) {
                e2->setWidthRight(width2);
            }
        } else {
            if (width2 < e2->widthLeft()) {
                e2->setWidthLeft(width2);
            }
        }
    }

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId) + _db.vNet(netId)->numTPorts());
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(outEdge);
                    // outEdge->setViaArea(A);
                    // _vUBViaArea[netId][outEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][outEdge->typeEdgeId()];
                    outEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), conductance);
                } else {
                    assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setMatrix(nPortNode->nPortNodeId(), _rGraph.numNPortOASGNodes(netId)+outNode->port()->netTPortId(), conductance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(inEdge);
                    // inEdge->setViaArea(A);
                    // _vUBViaArea[netId][inEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][inEdge->typeEdgeId()];
                    inEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), conductance);
                } else {
                    assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }

        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            if (currentBased) {
                solver.setInputVector(_rGraph.numNPortOASGNodes(netId) + tPortId, - tNode->port()->current());
            } else {
                double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
                solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, tPortConductance);
            }
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < tNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                assert(!outEdge->viaEdge());
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(outEdge);
                    // outEdge->setViaArea(A);
                    // _vUBViaArea[netId][outEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][outEdge->typeEdgeId()];
                    outEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                }
                cerr << "conductance = " << conductance << endl;
                if (outNode->nPort()) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, outNode->nPortNodeId(), conductance);
                } else {
                    assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, _rGraph.numNPortOASGNodes(netId) + outNode->port()->netTPortId(), conductance);
                }
                
            }
            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < tNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(inEdge);
                    // inEdge->setViaArea(A);
                    // _vUBViaArea[netId][inEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][inEdge->typeEdgeId()];
                    inEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " ";
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, inNode->nPortNodeId(), conductance);
                } else if (inNode->port()->netTPortId() >= 0) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, _rGraph.numNPortOASGNodes(netId) + inNode->port()->netTPortId(), conductance);
                } else {
                    assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(_rGraph.numNPortOASGNodes(netId) + tPortId, inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }
        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(solver.V(nPortNodeId));
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
            cerr << "net" << netId << " tPort" << tPortId << ": voltage = " << solver.V(_rGraph.numNPortOASGNodes(netId) + tPortId);
            cerr << ", current = " << solver.V(_rGraph.numNPortOASGNodes(netId) + tPortId) * tPortConductance << endl;
        }
        _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
        }
        // assert(false);
    }

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     _rGraph.vOASGNode(nodeId) -> print();
    // }
    // vector<double> netVoltage;
    // vector< vector<double> > vVoltage(3, netVoltage);
    // vVoltage[0] = { 4.99999975251689,
    //                 4.99999948245706,
    //                 4.99999939003966,
    //                 4.50000024748309,
    //                 4.50000051754296,
    //                 4.50000060996036,
    //                 4.78574240598038,
    //                 4.86315393669003 };
    // vVoltage[1] = { 4.99999967817019,
    //                 4.99999932627796,
    //                 4.99999920660867,
    //                 4.50000028543947,
    //                 4.50000063733168,
    //                 4.50000075700098,
    //                 4.40000003639034,
    //                 4.40000003639039,
    //                 4.40000003639039,
    //                 4.70485438179277,
    //                 4.80649030108812,
    //                 4.45000000000000,
    //                 4.44999999999998,
    //                 4.45000016091490 };
    // vVoltage[2] = { 4.99999948018458,
    //                 4.99999882037443,
    //                 4.99999859900624,
    //                 4.50000026702640,
    //                 4.50000052112573,
    //                 4.50000060637678,
    //                 4.40000025278904,
    //                 4.40000065849982,
    //                 4.40000079461696 };

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
    //         _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(vVoltage[netId][nPortNodeId]);
    //     }
    //     _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
    //     for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
    //         _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
    //     }
    // }
    // OASGNode* t1 = _rGraph.vNPortOASGNode(1, 7);
    // cerr << "t1(" << t1->x() << "," << t1->y() << ")" << endl;
    // OASGNode* t2 = _rGraph.vNPortOASGNode(1, 8);
    // cerr << "t2(" << t2->x() << "," << t2->y() << ")" << endl;

    // for (size_t nodeId = 0; nodeId < _rGraph.numOASGNodes(); ++ nodeId) {
    //     cerr << "Node[" << nodeId << "].voltage = " << _rGraph.vOASGNode(nodeId)->voltage() << endl;
    // }

    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     Port* sPort = _rGraph.sPort(netId);
    //     for (size_t netTPortId = 0; netTPortId < _rGraph.numTPorts(netId); ++ netTPortId) {
    //         GRBLinExpr netCurrent;
    //         Port* tPort = _rGraph.tPort(netId, netTPortId);
    //         size_t twoPinNetId = _rGraph.twoPinNetId(sPort->portId(), tPort->portId());
    //         for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //             for (size_t RGEdgeId = 0; RGEdgeId < _rGraph.numRGEdges(twoPinNetId, layId); ++ RGEdgeId) {
    //                 RGEdge* rgEdge = _rGraph.vEdge(twoPinNetId, layId, RGEdgeId);
    //             }
    //         }
    //     }
    // }
    
}

double GlobalMgr::oldViaEdgeArea(OASGEdge* e) {
    assert(e->viaEdge());
    double polygonArea = e->boundPolygon()->area();
    double lowLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId());
    double upLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId()+1);
    double ratio = _db.VIA16D8A24()->metalArea() / pow(max(lowLayPadRadius, upLayPadRadius), 2);
    return polygonArea * ratio;
}

void GlobalMgr::voltageDemandAssignment() {
    // auto viaEdgeArea = [&] (OASGEdge* e) -> double {
    //     assert(e->viaEdge());
    //     double polygonArea = e->boundPolygon()->area();
    //     double lowLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId());
    //     double upLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId()+1);
    //     double ratio = _db.VIA16D8A24()->metalArea() / pow(max(lowLayPadRadius, upLayPadRadius), 2);
    //     return polygonArea * ratio;
    // };

    for (size_t sCapId = 0; sCapId < _vSglCapConstr.size(); ++ sCapId) {
        OASGEdge* sglEdge = _vSglCapConstr[sCapId].e1;
        double sglWidth = _vSglCapConstr[sCapId].width / _vSglCapConstr[sCapId].ratio1;
        bool sglRight = _vSglCapConstr[sCapId].right1;
        if (sglRight) {
            if (sglWidth < sglEdge->widthRight()) {
                sglEdge->setWidthRight(sglWidth);
            }
        } else {
            if (sglWidth < sglEdge->widthLeft()) {
                sglEdge->setWidthLeft(sglWidth);
            }
        }
    }
    for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
        OASGEdge* e1 = _vCapConstr[capId].e1;
        OASGEdge* e2 = _vCapConstr[capId].e2;
        double width1 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio1;
        double width2 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio2;
        bool right1 = _vCapConstr[capId].right1;
        bool right2 = _vCapConstr[capId].right2;
        if (right1) {
            if (width1 < e1->widthRight()) {
                e1->setWidthRight(width1);
            }
        } else {
            if (width1 < e1->widthLeft()) {
                e1->setWidthLeft(width1);
            }
        }
        if (right2) {
            if (width2 < e2->widthRight()) {
                e2->setWidthRight(width2);
            }
        } else {
            if (width2 < e2->widthLeft()) {
                e2->setWidthLeft(width2);
            }
        }
    }

    /*
    for (size_t capId = 0; capId < _vNetCapConstr.size(); ++ capId) {
        OASGEdge* e1 = _vNetCapConstr[capId].e1;
        OASGEdge* e2 = _vNetCapConstr[capId].e2;
        double width1 = 0.5 * _vNetCapConstr[capId].width / _vNetCapConstr[capId].ratio1;
        double width2 = 0.5 * _vNetCapConstr[capId].width / _vNetCapConstr[capId].ratio2;
        bool right1 = _vNetCapConstr[capId].right1;
        bool right2 = _vNetCapConstr[capId].right2;
        if (right1) {
            if (width1 < e1->widthRight()) {
                e1->setWidthRight(width1);
            }
        } else {
            if (width1 < e1->widthLeft()) {
                e1->setWidthLeft(width1);
            }
        }
        if (right2) {
            if (width2 < e2->widthRight()) {
                e2->setWidthRight(width2);
            }
        } else {
            if (width2 < e2->widthLeft()) {
                e2->setWidthLeft(width2);
            }
        }
    }
    */

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    // segment->plot(netId, layId);
                    _db.vNet(netId)->addSegment(segment, layId);
                // }
            }
        }
    }
    plotCurrentPaths();
    assert(false);

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId));
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(outEdge);
                    // outEdge->setViaArea(A);
                    // _vUBViaArea[netId][outEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][outEdge->typeEdgeId()];
                    outEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), conductance);
                } else {
                    // assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), outNode->port()->voltage(), conductance);
                    // solver.setMatrix(nPortNode->nPortNodeId(), _rGraph.numNPortOASGNodes(netId)+outNode->port()->netTPortId(), conductance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    // A = viaEdgeArea(inEdge);
                    // inEdge->setViaArea(A);
                    // _vUBViaArea[netId][inEdge->typeEdgeId()] = A;
                    A = _vUBViaArea[netId][inEdge->typeEdgeId()];
                    inEdge->setViaArea(A);
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), conductance);
                } else {
                    // assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }

        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(solver.V(nPortNodeId));
        }
        _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            double tCurrent = 0;
            for (size_t outEdgeId = 0; outEdgeId < tNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    // cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = outEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    // cerr << "planeEdge ";
                    l = outEdge->length();
                    // cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    // cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                // cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                tCurrent += abs(tNode->voltage() - outNode->voltage()) * conductance;
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < tNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    // cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = inEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    // cerr << "planeEdge ";
                    l = inEdge->length();
                    // cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    // cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                tCurrent += abs(inNode->voltage() - tNode->voltage()) * conductance;
            }
            // tCurrent = tCurrent * 0.5;
            double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
            cerr << "net" << netId << " tPort" << tPortId << ": voltage = " << tNode->voltage();
            cerr << ", current = " << tCurrent << endl;
        }
        // assert(false);
    }
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            cerr << "net" << netId << " source layer" << layId << " voltage = " << _rGraph.sourceOASGNode(netId, layId)->voltage() << endl;
        }
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                cerr << "net" << netId << " target" << tPortId << " layer" << layId << " voltage = " << _rGraph.targetOASGNode(netId, tPortId, layId)->voltage() << endl;
            }
        }
    }
}

void GlobalMgr::swapSTbyVolt() {

    cout << "#####################" << endl;
    cout << "Start Swap ST by Volt" << endl;
    cout << "#####################" << endl;
    for (size_t edgeId = 0; edgeId < _rGraph.numOASGEdges(); ++ edgeId) {
        OASGEdge* edge = _rGraph.vOASGEdge(edgeId);
        if ((edge->sNode()->voltage() < edge->tNode()->voltage())) {
            _rGraph.swapST(edge);
            cout << "The edge being swapped is " << edge->netId() << " Layer " << edge->layId() << endl;
            for (size_t capId = 0; capId < _vCapConstr.size(); ++capId) {
                if (_vCapConstr[capId].e1 == edge) {
                    _vCapConstr[capId].right1 = ! _vCapConstr[capId].right1;
                }
                if (_vCapConstr[capId].e2 == edge) {
                    _vCapConstr[capId].right2 = ! _vCapConstr[capId].right2;
                }
            }
            for (size_t sglCapId = 0; sglCapId < _vSglCapConstr.size(); ++ sglCapId) {
                if (_vSglCapConstr[sglCapId].e1 == edge) {
                    _vSglCapConstr[sglCapId].right1 = ! _vSglCapConstr[sglCapId].right1;
                }
            }
            for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++netCapId) {
                if (_vNetCapConstr[netCapId].e1 == edge) {
                    _vNetCapConstr[netCapId].right1 = ! _vNetCapConstr[netCapId].right1;
                }
                if (_vNetCapConstr[netCapId].e2 == edge) {
                    _vNetCapConstr[netCapId].right2 = ! _vNetCapConstr[netCapId].right2;
                }
            }
        }

    }
    
    // assert(false);

    cout << "######################" << endl;
    cout << "Finish Swap ST by Volt" << endl;
    cout << "######################" << endl;
}

void GlobalMgr::currentDistribution() {
    vector<double> vMediumLayerThickness;
    vector<double> vMetalLayerThickness;
    vector<double> vConductivity;
    for (size_t mediumLayId = 0; mediumLayId < _db.numMediumLayers(); ++ mediumLayId) {
        vMediumLayerThickness.push_back(_db.vMediumLayer(mediumLayId)->thickness());
    }
    for (size_t metalLayId = 0; metalLayId < _db.numLayers(); ++ metalLayId) {
        vMetalLayerThickness.push_back(_db.vMetalLayer(metalLayId)->thickness());
        vConductivity.push_back(_db.vMetalLayer(metalLayId)->conductivity());
    }
    double normRatio = _rGraph.sPort(0)->current();
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        if (normRatio > _rGraph.sPort(netId)->current()) {
            normRatio = _rGraph.sPort(netId)->current();
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            if (normRatio > _rGraph.tPort(netId, tPortId)->current()) {
                normRatio = _rGraph.tPort(netId, tPortId)->current();
            }
        }
    }

    // FlowLP solver(_rGraph, vMediumLayerThickness, vMetalLayerThickness, vConductivity, normRatio);
    FlowLP solver(_db, _rGraph);
    
    // set objective
    // cerr << "setObjective..." << endl;
    solver.setObjective(_db.areaWeight(), _db.viaWeight(), 0.5);

    // set flow conservation constraints
    // cerr << "setConserveConstraints..." << endl;
    solver.setConserveConstraints(true);
    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,
    
    
    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        cout << "LAYER :" << layId << endl << endl;
        //search each net
        for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
            //search each edge
            for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
                OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                
                pair<double, double> ratio;
                pair<bool, bool> right;
                double width;
                
                //compare other net edge
                for(size_t T_netId = S_netId+1; T_netId < _rGraph.numNets(); ++ T_netId){
                    //make sure to compare different net
                    for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                        OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
    
                    //    BuildCapacityConstraint(e1,e2,solver);
                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                         make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                         make_pair(e2->sNode()->x(), e2->sNode()->y()),
                                         make_pair(e2->tNode()->x(), e2->tNode()->y()),
                                         ratio, right, width))
                            solver.addCapacityConstraints(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                        
                    }   
                } 

                // obstacle constraint
                for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                   
                    Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                    pair<double, double> S2, T2;
                    for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                        for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                            // get the edge coordinates
                            S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
                            T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));
                            double vectorX = (T2.first - S2.first)/sqrt(pow(T2.first - S2.first,2)+pow(T2.second - S2.second,2));
                            double vectorY = (T2.second - S2.second)/sqrt(pow(T2.first - S2.first,2)+pow(T2.second - S2.second,2));
                            double normalX = vectorY;
                            double normalY = -vectorX;
                            //new S2
                            S2 = make_pair((S2.first - pow(10,-5)*(-vectorX +  normalX)), (S2.second - pow(10,-5)*(-vectorY +  normalY)));
                            //new T2 
                            T2 = make_pair((T2.first - pow(10,-5)*(vectorX +  normalX)), (T2.second - pow(10,-5)*(vectorY +  normalY)));
                        
                            if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                             make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                             S2, T2, ratio, right, width))
                                solver.addCapacityConstraints(e1, right.first, ratio.first, width);

                        }
                    }
                }

                // board cnstraint
                // bottom
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(_db.boardWidth(), 0),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // left
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, 0),
                                 make_pair(0, _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // top
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(0, _db.boardHeight()),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
                // right
                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                 make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                 make_pair(_db.boardWidth(), 0),
                                 make_pair(_db.boardWidth(), _db.boardHeight()),
                                 ratio, right, width))
                    solver.addCapacityConstraints(e1, right.first, ratio.first, width);
            }
        }  
    }
    
    // use solver.addCapacityConstraints(OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width)
    // to add their capacity constraints
    
    /*
    // layer0
    
    solver.addCapacityConstraints(_rGraph.vOASGEdge(24), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(30), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(30), true, 1, _rGraph.vOASGEdge(36), false, 1, 150);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(36), true, 1, 80);
    // layer1
    solver.addCapacityConstraints(_rGraph.vOASGEdge(25), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(24), true, 1, _rGraph.vOASGEdge(31), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(31), true, 1, _rGraph.vOASGEdge(43), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(43), true, 1, 80);
    //layer2
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), false, 1, 80);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(27), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(33), false, 1, 0);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(33), true, 1, _rGraph.vOASGEdge(44), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(44), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(48), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(46), true, 1, 80);
    // layer3
    solver.addCapacityConstraints(_rGraph.vOASGEdge(29), false, 1, 80);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(29), true, 1, _rGraph.vOASGEdge(35), false, 1, 100);
    //solver.addCapacityConstraints(_rGraph.vOASGEdge(35), true, 1, _rGraph.vOASGEdge(45), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(45), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), false, 1, 100);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(49), true, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), false, 1, 0);
    solver.addCapacityConstraints(_rGraph.vOASGEdge(47), true, 1, 80);
    */
    // solve the MCFP formulation and collect the result
    // solver.solve();
    // solver.collectResult();
    // solver.printResult();
    vector<double> vLambda(solver.numCapConstrs(), 2.0);
    for (size_t iter = 0; iter < 5; ++iter) {
        solver.relaxCapacityConstraints(vLambda);
        solver.solveRelaxed();
        solver.collectRelaxedResult();
        cerr << "iter = " << iter << endl;
        solver.printRelaxedResult();
        for (size_t capId = 0; capId < solver.numCapConstrs(); ++ capId) {
            vLambda[capId] *= vLambda[capId];
        }
    }
    solver.relaxCapacityConstraints(vLambda);
    solver.solveRelaxed();
    solver.collectRelaxedResult();
    solver.printRelaxedResult();

    // add traces to each net
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
                OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
                // if (e->current() > 0) {
                    Segment* segment = edge2Segment(e);
                    _db.vNet(netId)->addSegment(segment, layId);
                // }
            }
        }
    }
}

void GlobalMgr::plotCurrentPaths() {
    // for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
    //     for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
    //         for (size_t pEdgeId = 0; pEdgeId < _rGraph.numPlaneOASGEdges(netId, layId); ++ pEdgeId) {
    //             OASGEdge* e = _rGraph.vPlaneOASGEdge(netId, layId, pEdgeId);
    //             Trace* trace = edge2Trace(e);
    //             trace->plot(netId, layId);
    //             // _plot.drawLine(e->sNode()->x(), e->sNode()->y(), e->tNode()->x(), e->tNode()->y(), netId, layId);
    //         }
    //     }
    // }
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        Net* net = _db.vNet(netId);
        for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId) {
            for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
                net->vSegment(layId, segId)->plot(netId, layId);
            }
        }
    }
}

void GlobalMgr::checkFeasible(bool currentBased) {
    // auto viaEdgeArea = [&] (OASGEdge* e) -> double {
    //     assert(e->viaEdge());
    //     double polygonArea = e->boundPolygon()->area();
    //     double lowLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId());
    //     double upLayPadRadius = _db.VIA16D8A24()->padRadius(e->layId()+1);
    //     double ratio = _db.VIA16D8A24()->metalArea() / pow(max(lowLayPadRadius, upLayPadRadius), 2);
    //     return polygonArea * ratio;
    // };

    // for (size_t sCapId = 0; sCapId < _vSglCapConstr.size(); ++ sCapId) {
    //     OASGEdge* sglEdge = _vSglCapConstr[sCapId].e1;
    //     double sglWidth = _vSglCapConstr[sCapId].width / _vSglCapConstr[sCapId].ratio1;
    //     bool sglRight = _vSglCapConstr[sCapId].right1;
    //     if (sglRight) {
    //         if (sglWidth < sglEdge->widthRight()) {
    //             sglEdge->setWidthRight(sglWidth);
    //         }
    //     } else {
    //         if (sglWidth < sglEdge->widthLeft()) {
    //             sglEdge->setWidthLeft(sglWidth);
    //         }
    //     }
    // }
    // for (size_t capId = 0; capId < _vCapConstr.size(); ++ capId) {
    //     OASGEdge* e1 = _vCapConstr[capId].e1;
    //     OASGEdge* e2 = _vCapConstr[capId].e2;
    //     double width1 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio1;
    //     double width2 = 0.5 * _vCapConstr[capId].width / _vCapConstr[capId].ratio2;
    //     bool right1 = _vCapConstr[capId].right1;
    //     bool right2 = _vCapConstr[capId].right2;
    //     if (right1) {
    //         if (width1 < e1->widthRight()) {
    //             e1->setWidthRight(width1);
    //         }
    //     } else {
    //         if (width1 < e1->widthLeft()) {
    //             e1->setWidthLeft(width1);
    //         }
    //     }
    //     if (right2) {
    //         if (width2 < e2->widthRight()) {
    //             e2->setWidthRight(width2);
    //         }
    //     } else {
    //         if (width2 < e2->widthLeft()) {
    //             e2->setWidthLeft(width2);
    //         }
    //     }
    // }

    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId) + _db.vNet(netId)->numTPorts());
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = outEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), conductance);
                } else {
                    assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setMatrix(nPortNode->nPortNodeId(), _rGraph.numNPortOASGNodes(netId)+outNode->port()->netTPortId(), conductance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = inEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), conductance);
                } else {
                    assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }

        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            if (currentBased) {
                solver.setInputVector(_rGraph.numNPortOASGNodes(netId) + tPortId, - tNode->port()->current());
            } else {
                double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
                solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, tPortConductance);
            }
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < tNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                assert(!outEdge->viaEdge());
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = outEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                }
                cerr << "conductance = " << conductance << endl;
                if (outNode->nPort()) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, outNode->nPortNodeId(), conductance);
                } else {
                    assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, _rGraph.numNPortOASGNodes(netId) + outNode->port()->netTPortId(), conductance);
                }
                
            }
            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < tNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = inEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " ";
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, inNode->nPortNodeId(), conductance);
                } else if (inNode->port()->netTPortId() >= 0) {
                    solver.setMatrix(_rGraph.numNPortOASGNodes(netId) + tPortId, _rGraph.numNPortOASGNodes(netId) + inNode->port()->netTPortId(), conductance);
                } else {
                    assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(_rGraph.numNPortOASGNodes(netId) + tPortId, inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }
        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            _rGraph.vNPortOASGNode(netId, nPortNodeId) -> setVoltage(solver.V(nPortNodeId));
        }
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
            cerr << "net" << netId << " tPort" << tPortId << ": voltage = " << solver.V(_rGraph.numNPortOASGNodes(netId) + tPortId);
            cerr << ", current = " << solver.V(_rGraph.numNPortOASGNodes(netId) + tPortId) * tPortConductance << endl;
        }
        _rGraph.sourceOASGNode(netId, 0) -> setVoltage(_rGraph.sourceOASGNode(netId, 0)->port()->voltage());
        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            _rGraph.targetOASGNode(netId, tPortId, 0) -> setVoltage(_rGraph.targetOASGNode(netId, tPortId, 0)->port()->voltage());
        }
        // assert(false);
    }
}

void GlobalMgr::checkVoltDemandFeasible() {
    for (size_t netId = 0; netId < _rGraph.numNets(); ++ netId) {
        VoltEigen solver(_rGraph.numNPortOASGNodes(netId));
        for (size_t nPortNodeId = 0; nPortNodeId < _rGraph.numNPortOASGNodes(netId); ++ nPortNodeId) {
            OASGNode* nPortNode = _rGraph.vNPortOASGNode(netId, nPortNodeId);
            // set matrix and input vector for outNodes
            for (size_t outEdgeId = 0; outEdgeId < nPortNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(nPortNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = outEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = outEdge->length();
                    cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                if (outNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), conductance);
                } else {
                    assert(outNode->port() != _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), outNode->port()->voltage(), conductance);
                    // solver.setMatrix(nPortNode->nPortNodeId(), _rGraph.numNPortOASGNodes(netId)+outNode->port()->netTPortId(), conductance);
                    // cerr << "voltage = " << outNode->port()->voltage() << endl;
                }
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < nPortNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(nPortNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = inEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    cerr << "planeEdge ";
                    l = inEdge->length();
                    cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                if (inNode->nPort()) {
                    solver.setMatrix(nPortNode->nPortNodeId(), inNode->nPortNodeId(), conductance);
                } else {
                    assert(inNode->port() == _db.vNet(netId)->sourcePort());
                    solver.setInputVector(nPortNode->nPortNodeId(), inNode->port()->voltage(), conductance);
                    cerr << "voltage = " << inNode->port()->voltage() << endl;
                }
            }
        }

        // cerr << "G = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     for (size_t colId = 0; colId < solver.numNodes(); ++ colId) {
        //         cerr << setprecision(15) << solver.G(rowId, colId);
        //         if (colId < solver.numNodes()-1) {
        //             cerr << ", ";
        //         }
        //     }
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;
        // cerr << "I = " << endl;
        // for (size_t rowId = 0; rowId < solver.numNodes(); ++ rowId) {
        //     cerr << setprecision(15) << solver.I(rowId);
        //     if (rowId < solver.numNodes()-1) {
        //         cerr << ";" << endl;
        //     }
        // }
        // cerr << endl;

        // cerr << "V = ";
        solver.solve();
        // cerr << endl;

        for (size_t tPortId = 0; tPortId < _rGraph.numTPorts(netId); ++ tPortId) {
            OASGNode* tNode = _rGraph.targetOASGNode(netId, tPortId, 0);
            assert(!tNode->nPort() && tNode->port() == _db.vNet(netId)->targetPort(tPortId));
            double tCurrent = 0;
            for (size_t outEdgeId = 0; outEdgeId < tNode->numOutEdges(); ++ outEdgeId) {
                OASGEdge* outEdge = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId));
                OASGNode* outNode = _rGraph.vOASGEdge(tNode->outEdgeId(outEdgeId))->tNode();
                double conductance;
                double l;
                double A;
                if (outEdge->viaEdge()) {
                    // cerr << "viaEdge "; 
                    l = 0.5* _db.vMetalLayer(outEdge->layId())->thickness() 
                        + _db.vMediumLayer(outEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(outEdge->layId()+1)->thickness();
                    // cerr << "l=" << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = outEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A * 1E-6) / (l * 1E-3);
                } else {
                    // cerr << "planeEdge ";
                    l = outEdge->length();
                    // cerr << "length = " << l << " ";
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(outEdge->layId())->thickness();
                    double width = outEdge->widthLeft() + outEdge->widthRight();
                    // cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(outEdge->layId())->thickness()*1E-3 * width*1E-3) / (l * 1E-3);
                }
                // cerr << "conductance = " << conductance << endl;

                // assert(outNode->nPort() || (!outNode->nPort() && outNode->port() == _db.vNet(netId)->targetPort(tPortId)));
                // solver.setMatrix(nPortNode->nPortNodeId(), outNode->nPortNodeId(), resistance);
                tCurrent += abs(tNode->voltage() - outNode->voltage()) * conductance;
            }

            // set matrix and input vector for inNodes
            for (size_t inEdgeId = 0; inEdgeId < tNode->numInEdges(); ++ inEdgeId) {
                OASGEdge* inEdge = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId));
                OASGNode* inNode = _rGraph.vOASGEdge(tNode->inEdgeId(inEdgeId))->sNode();
                double conductance;
                double l;
                double A;
                if (inEdge->viaEdge()) {
                    // cerr << "viaEdge ";
                    l = 0.5* _db.vMetalLayer(inEdge->layId())->thickness() 
                        + _db.vMediumLayer(inEdge->layId()+1)->thickness() 
                        + 0.5*_db.vMetalLayer(inEdge->layId()+1)->thickness();
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vVia(0)->shape()->boxH();
                    A = inEdge->viaArea();
                    conductance = (_db.vMetalLayer(0)->conductivity() * A*1E-6) / (l*1E-3);
                } else {
                    // cerr << "planeEdge ";
                    l = inEdge->length();
                    // cerr << "length = " << l << " " ;
                    double width = inEdge->widthLeft() + inEdge->widthRight();
                    // cerr << "width = " << width << " ";
                    conductance = (_db.vMetalLayer(0)->conductivity() * _db.vMetalLayer(inEdge->layId())->thickness()*1E-3 * width*1E-3) / (l*1E-3);
                    // resistance = _db.vMetalLayer(0)->conductivity() * l / _db.vMetalLayer(inEdge->layId())->thickness();
                }
                cerr << "conductance = " << conductance << endl;

                tCurrent += abs(inNode->voltage() - tNode->voltage()) * conductance;
            }
            double tPortConductance = tNode->port()->current() / tNode->port()->voltage();
            cerr << "net" << netId << " tPort" << tPortId << ": voltage = " << tNode->voltage();
            cerr << ", current = " << tCurrent << endl;
        }
        // assert(false);
    }
}

Trace* GlobalMgr::edge2Trace(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    return trace;
}

Segment* GlobalMgr::edge2Segment(OASGEdge* edge) {
    assert (!edge->viaEdge());
    double offset = 0.5 * ( edge->widthRight() - edge->widthLeft() );
    double xOffset = offset * (edge->tNode()->y() - edge->sNode()->y()) / edge->length(); // offset * sin(theta)
    double yOffset = offset * (edge->sNode()->x() - edge->tNode()->x()) / edge->length(); // offset * -cos(theta)
    Node* sNode = new Node(edge->sNode()->x()+xOffset, edge->sNode()->y()+yOffset, _plot);
    Node* tNode = new Node(edge->tNode()->x()+xOffset, edge->tNode()->y()+yOffset, _plot);
    assert(edge->widthRight() >= 0);
    assert(edge->widthLeft() >= 0);
    Trace* trace = new Trace(sNode, tNode, edge->widthRight() + edge->widthLeft(), _plot);
    pair<double, double> sPos = make_pair(edge->sNode()->x(), edge->sNode()->y());
    pair<double, double> tPos = make_pair(edge->tNode()->x(), edge->tNode()->y());
    Segment* segment = new Segment(trace, sPos, tPos, edge->widthLeft(), edge->widthRight(), edge->sNode()->voltage(), edge->tNode()->voltage(), edge->current());
    return segment;
}

void GlobalMgr::genCapConstrs() {
    auto addCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
        CapConstr capConstr = {e1, right1, ratio1, e2, right2, ratio2, width};
        _vCapConstr.push_back(capConstr);
    };
    auto addSglCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, double width) {
        SingleCapConstr sglCapConstr = {e1, right1, ratio1, width};
        _vSglCapConstr.push_back(sglCapConstr);
    };
    auto addNetCapConstr = [&] (OASGEdge* e1, bool right1, double ratio1, OASGEdge* e2, bool right2, double ratio2, double width) {
        assert(e1->netId() == e2->netId());
        CapConstr netCapConstr = {e1, right1, ratio1, e2, right2, ratio2, width};
        _vNetCapConstr.push_back(netCapConstr);
    };
    // set capacity constraints
    // TODO for Tsai and Huang:
    // for each layer, for each neighboring OASGEdges,
    
    
    //search each layer                                                                           
    for (size_t layId = 0; layId < _rGraph.numLayers(); ++ layId){
        //search each net
        //for(size_t S_netId = 0; S_netId < _rGraph.numNets(); ++ S_netId){
        //use Rgraph RGEdge to add constraint
        for(size_t S_twoPinNetId = 0; S_twoPinNetId < _rGraph.num2PinNets(); ++S_twoPinNetId ){
            //search each edge
            //for (size_t S_EdgeId = 0; S_EdgeId < _rGraph.numPlaneOASGEdges(S_netId, layId); ++ S_EdgeId){
            for(size_t S_RGEdgeId = 0; S_RGEdgeId < _rGraph.numRGEdges(S_twoPinNetId,layId); ++S_RGEdgeId){
                RGEdge* rge1 = _rGraph.vEdge(S_twoPinNetId,layId,S_RGEdgeId);
                for(size_t S_EdgeId = 0; S_EdgeId < rge1->numEdges(); ++S_EdgeId){
                    //OASGEdge* e1 = _rGraph.vPlaneOASGEdge(S_netId, layId, S_EdgeId);
                    OASGEdge* e1 = rge1->vEdge(S_EdgeId);

                    pair<double, double> ratio;
                    pair<bool, bool> right;
                    double width;
                
                    //compare to other net edge
                    //for(size_t T_netId = S_netId; T_netId < _rGraph.numNets(); ++ T_netId){
                    for(size_t T_twoPinNetId = S_twoPinNetId; T_twoPinNetId < _rGraph.num2PinNets(); ++T_twoPinNetId ){
                        //for (size_t T_EdgeId = 0; T_EdgeId < _rGraph.numPlaneOASGEdges(T_netId, layId); ++ T_EdgeId){
                        for(size_t T_RGEdgeId = 0; T_RGEdgeId < _rGraph.numRGEdges(T_twoPinNetId,layId); ++T_RGEdgeId){
                            RGEdge* rge2 = _rGraph.vEdge(T_twoPinNetId,layId,T_RGEdgeId);

                            auto it = std::find_if(_vCrossConstr.begin(), _vCrossConstr.end(), [rge1, rge2](const std::pair<RGEdge*, RGEdge*>& edgePair) {
                                return (edgePair.first == rge1 && edgePair.second == rge2) || (edgePair.first == rge2 && edgePair.second == rge1);
                            });

                            if (it != _vCrossConstr.end()) break;
                            else {
                            
                                for(size_t T_EdgeId = 0; T_EdgeId < rge2->numEdges(); ++T_EdgeId){
                                    //OASGEdge* e2 = _rGraph.vPlaneOASGEdge(T_netId, layId, T_EdgeId);
                                    OASGEdge* e2 = rge2->vEdge(T_EdgeId);

                                    if (e1 != e2) {
                                        // bool isSameNet = (T_netId == S_netId);
                                        // if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                        //              make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                        //              make_pair(e2->sNode()->x(), e2->sNode()->y()),
                                        //              make_pair(e2->tNode()->x(), e2->tNode()->y()),
                                        //              ratio, right, width, isSameNet))
                                        //     addCapConstr(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                                        double vtxX = e2->tNode()->x() - e2->sNode()->x();
                                        double vtxY = e2->tNode()->y() - e2->sNode()->y();
                                        pair<double, double> S2, T2;

                                        S2 = make_pair((e2->sNode()->x() + pow(10,-7)*vtxX), ( e2->sNode()->y() + pow(10,-7)*vtxY));
                                        T2 = make_pair((e2->tNode()->x() - pow(10,-7)*vtxX), ( e2->tNode()->y() - pow(10,-7)*vtxY));

                                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),make_pair(e1->tNode()->x(), e1->tNode()->y()),S2, T2,ratio, right, width)) {
                                            if (e1->netId() == e2->netId()) {
                                                addNetCapConstr(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                                            } 
                                            else {
                                                addCapConstr(e1, right.first, ratio.first, e2, right.second, ratio.second, width);
                                            }
                                        }
                                    }
                                }
                            }
                        }   
                    }

                    //紀錄edge 與 obstacle間constraint最小的值，存起來最後只新增一項add cap constraint
                    double min_width_R = 999999;
                    double min_width_L = 999999;
                    double min_ratio_R = 1;
                    double min_ratio_L = 1;
                    ///////////////////////////////////////////////////////////////////////////////

                    // obstacle constraint
                    for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                    
                        Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                        pair<double, double> S2, T2;
                        for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                            for(size_t vtxId = 0; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                                // get the edge coordinates
                                S2 = make_pair(obs->vShape(shapeId)->bPolygonX(vtxId), obs->vShape(shapeId)->bPolygonY(vtxId));
                                T2 = make_pair(obs->vShape(shapeId)->bPolygonX((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()), obs->vShape(shapeId)->bPolygonY((vtxId+1) % obs->vShape(shapeId)->numBPolyVtcs()));

                                double vectorX = (T2.first - S2.first);
                                double vectorY = (T2.second - S2.second);
                                
                                double normalX = vectorY;
                                double normalY = -vectorX;

                                S2 = make_pair((S2.first + pow(10,-6)*(vectorX) - pow(10,-8)*(normalX)), (S2.second + pow(10,-6)*(vectorY) - pow(10,-8)*(normalY)));
                                //new T2 
                                T2 = make_pair((T2.first + pow(10,-6)*(-vectorX) - pow(10,-8)*(normalX)), (T2.second + pow(10,-6)*(-vectorY) - pow(10,-8)*(normalY)));
                            
                                if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),make_pair(e1->tNode()->x(), e1->tNode()->y()),S2, T2, ratio, right, width)){
                                    //Right
                                    if(right.first){
                                        if(ratio.first != 0){
                                            if((width/ratio.first) < (min_width_R/min_ratio_R)){
                                                min_width_R = width;
                                                min_ratio_R = ratio.first;
                                                //addSglCapConstr(e1, right.first, ratio.first, width);
                                            }
                                        }
                                    }
                                    //left
                                    else{
                                        if(ratio.first != 0){
                                            if((width/ratio.first) < (min_width_L/min_ratio_L)){
                                                min_width_L = width;
                                                min_ratio_L = ratio.first;
                                                //addSglCapConstr(e1, right.first, ratio.first, width);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // port bounding polygon constraints from other nets
                    // Bug: if the port is not connected on the layer, its bounding polygon should be ignored
                    for(size_t T_netId = 0; T_netId < _rGraph.numNets(); ++ T_netId) {
                        //if (T_netId != S_netId) {
                            pair<double, double> S2, T2;

                            Polygon* bPolygon = _db.vNet(T_netId)->sourcePort()->boundPolygon();
                            pair<double,double> sourceNode = make_pair(bPolygon->ctrX(),bPolygon->ctrY());

                            if(!((e1->sNode()->x() == sourceNode.first && e1->sNode()->y() == sourceNode.second)||(e1->tNode()->x() == sourceNode.first && e1->tNode()->y() == sourceNode.second))){
            
                                for (size_t vtxId = 0; vtxId < bPolygon->numVtcs(); ++ vtxId) {
                                    // get the edge coordinates
                                    S2 = make_pair(bPolygon->vtxX(vtxId), bPolygon->vtxY(vtxId));
                                    T2 = make_pair(bPolygon->vtxX((vtxId+1) % bPolygon->numVtcs()), bPolygon->vtxY((vtxId+1) % bPolygon->numVtcs()));

                                    double vectorX = (T2.first - S2.first);
                                    double vectorY = (T2.second - S2.second);
                                    double normalX = vectorY;
                                    double normalY = -vectorX;
                                    //new S2
                                    S2 = make_pair((S2.first + pow(10,-6)*(vectorX) - pow(10,-8)*(normalX)), (S2.second + pow(10,-6)*(vectorY) - pow(10,-8)*(normalY)));
                                    //new T2 
                                    T2 = make_pair((T2.first + pow(10,-6)*(-vectorX) - pow(10,-8)*(normalX)), (T2.second + pow(10,-6)*(-vectorY) - pow(10,-8)*(normalY)));

                                    if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),make_pair(e1->tNode()->x(), e1->tNode()->y()),S2, T2, ratio, right, width)){
                                        //Right
                                        if(right.first){
                                            if(ratio.first != 0){
                                                if((width/ratio.first) < (min_width_R/min_ratio_R)){
                                                    min_width_R = width;
                                                    min_ratio_R = ratio.first;
                                                    //addSglCapConstr(e1, right.first, ratio.first, width);
                                                }
                                            }
                                        }
                                        //left
                                        else{
                                            if(ratio.first != 0){
                                                if((width/ratio.first) < (min_width_L/min_ratio_L)){
                                                    min_width_L = width;
                                                    min_ratio_L = ratio.first;
                                                    //addSglCapConstr(e1, right.first, ratio.first, width);
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            for (size_t tPortId = 0; tPortId < _db.vNet(T_netId)->numTPorts(); ++ tPortId) {
                                bPolygon = _db.vNet(T_netId)->targetPort(tPortId)->boundPolygon();
                                pair<double,double> targetNode = make_pair(bPolygon->ctrX(),bPolygon->ctrY());

                                if(!((e1->sNode()->x() == targetNode.first && e1->sNode()->y() == targetNode.second) || (e1->tNode()->x() == targetNode.first && e1->tNode()->y() == targetNode.second))){
                                    for (size_t vtxId = 0; vtxId < bPolygon->numVtcs(); ++ vtxId) {
                                        // get the edge coordinates
                                        S2 = make_pair(bPolygon->vtxX(vtxId), bPolygon->vtxY(vtxId));
                                        T2 = make_pair(bPolygon->vtxX((vtxId+1) % bPolygon->numVtcs()), bPolygon->vtxY((vtxId+1) % bPolygon->numVtcs()));

                                        double vectorX = (T2.first - S2.first);
                                        double vectorY = (T2.second - S2.second);
                                        double normalX = vectorY;
                                        double normalY = -vectorX;
                                        //new S2
                                        S2 = make_pair((S2.first + pow(10,-6)*(vectorX) - pow(10,-8)*(normalX)), (S2.second + pow(10,-6)*(vectorY) - pow(10,-8)*(normalY)));
                                        //new T2 
                                        T2 = make_pair((T2.first + pow(10,-6)*(-vectorX) - pow(10,-8)*(normalX)), (T2.second + pow(10,-6)*(-vectorY) - pow(10,-8)*(normalY)));

                                        if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),make_pair(e1->tNode()->x(), e1->tNode()->y()),S2, T2, ratio, right, width)){
                                            //Right
                                            if(right.first){
                                                if(ratio.first != 0){
                                                    if((width/ratio.first) < (min_width_R/min_ratio_R)){
                                                        min_width_R = width;
                                                        min_ratio_R = ratio.first;
                                                        //addSglCapConstr(e1, right.first, ratio.first, width);
                                                    }
                                                }
                                            }
                                            //left
                                            else{
                                                if(ratio.first != 0){
                                                    if((width/ratio.first) < (min_width_L/min_ratio_L)){
                                                        min_width_L = width;
                                                        min_ratio_L = ratio.first;
                                                        //addSglCapConstr(e1, right.first, ratio.first, width);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        //}
                    }

                    //Right
                    addSglCapConstr(e1, 1 , min_ratio_R, min_width_R);
                    //Left
                    addSglCapConstr(e1, 0 , min_ratio_L, min_width_L);
                    

                    // board cnstraint
                    // bottom
                    if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                    make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                    make_pair(0, 0),
                                    make_pair(_db.boardWidth(), 0),
                                    ratio, right, width))
                        addSglCapConstr(e1, right.first, ratio.first, width);
                    // left
                    if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                    make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                    make_pair(0, 0),
                                    make_pair(0, _db.boardHeight()),
                                    ratio, right, width))
                        addSglCapConstr(e1, right.first, ratio.first, width);
                    // top
                    if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                    make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                    make_pair(0, _db.boardHeight()),
                                    make_pair(_db.boardWidth(), _db.boardHeight()),
                                    ratio, right, width))
                        addSglCapConstr(e1, right.first, ratio.first, width);
                    // right
                    if(addConstraint(make_pair(e1->sNode()->x(), e1->sNode()->y()),
                                    make_pair(e1->tNode()->x(), e1->tNode()->y()),
                                    make_pair(_db.boardWidth(), 0),
                                    make_pair(_db.boardWidth(), _db.boardHeight()),
                                    ratio, right, width))
                        addSglCapConstr(e1, right.first, ratio.first, width);
                }
            }
        }  
    }
    for (size_t netCapId = 0; netCapId < _vNetCapConstr.size(); ++ netCapId) {
        // OASGEdge* e1 = _vNetCapConstr[netCapId].e1;
        // OASGEdge* e2 = _vNetCapConstr[netCapId].e2;
        // cerr << "same net constraint " << netCapId << ":" << endl;
        // cerr << "   pEdge (net" << e1->netId() << ", layer" << e1->layId() << ", s(" << e1->sNode()->x() << ", " << e1->sNode()->y();
        // cerr << "), t(" << e1->tNode()->x() << ", " << e1->tNode()->y() << ")" << endl;
        // cerr << "   pEdge (net" << e2->netId() << ", layer" << e2->layId() << ", s(" << e2->sNode()->x() << ", " << e2->sNode()->y();
        // cerr << "), t(" << e2->tNode()->x() << ", " << e2->tNode()->y() << ")" << endl;
        // if (_vNetCapConstr[netCapId].right1) {
        //     cerr << "   e1 right, " ;
        // } else {
        //     cerr << "   e1 left, " ;
        // }
        // cerr << "ratio1 = " << _vNetCapConstr[netCapId].ratio1 << endl;
        // if (_vNetCapConstr[netCapId].right2) {
        //     cerr << "   e2 right, " ;
        // } else {
        //     cerr << "   e2 left, " ;
        // }
        // cerr << "ratio2 = " << _vNetCapConstr[netCapId].ratio2 << endl;
        // cerr << "   width = " << _vNetCapConstr[netCapId].width << endl;
        if (_vNetCapConstr[netCapId].width < 1e-4) {
            _vNetCapConstr[netCapId].width = 0;
        }
    }
}