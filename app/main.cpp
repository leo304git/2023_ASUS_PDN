#include <cstdio>
#include <gurobi_c++.h>
#include "base/Include.h"
#include "base/DB.h"
#include "base/Parser.h"
#include "global/GlobalMgr.h"
#include "base/SVGPlot.h"
#include "detailed/DetailedMgr.h"
#include "global/PreMgr.h"
#include "base/OutputWriter.h"
#include  <time.h>

using namespace std;

int main(int argc, char* argv[]){

    ifstream finST, fin, finOb, finPa;
    ofstream fout, ftunRes, foutResult;
    finST.open(argv[1], ifstream::in);
    if (finST.is_open()) {
        cout << "input file (st components) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file (st components)" << endl;
    }
    finPa.open(argv[2], ifstream::in);
    int numVIter, numIIter, numIVIter; 
    if (finPa.is_open()) {
        cout << "input file (Parameters) is opened successfully" << endl;
        std::map<std::string, int> parameters;
        
        std::string line;
        // 逐行读取文件
        while (std::getline(finPa, line)) {
            size_t delimiter_pos = line.find("=");
            if (delimiter_pos != std::string::npos) {
                // 提取键和值
                std::string key = line.substr(0, delimiter_pos);
                std::string value_str = line.substr(delimiter_pos + 1);
                // 去除前导和尾随空格
                key = key.substr(key.find_first_not_of(" "), key.find_last_not_of(" ") + 1);
                value_str = value_str.substr(value_str.find_first_not_of(" "), value_str.find_last_not_of(" ") + 1);
                
                // 将值转换为整数
                int value = std::stoi(value_str);

                // 存储到map中
                parameters[key] = value;
            }
        }
        numIVIter = parameters["numIVIter"];
        numIIter = parameters["numIIter"];
        numVIter = parameters["numVIter"];

    } else {
        cerr << "Error opening input file (Parameters)" << endl;
    }
    fin.open(argv[3], ifstream::in);
    if (fin.is_open()) {
        cout << "input file is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    finOb.open(argv[4], ifstream::in);
    if (finOb.is_open()) {
        cout << "input file (obstacle) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    fout.open(argv[5], ofstream::out);
    if (fout.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }
    ftunRes.open(argv[6], ofstream::out);
    if (ftunRes.is_open()) {
        cout << "Tuning Result file is opened successfully" << endl;
    } else {
        cerr << "Error opening tuning result file" << endl;
    }
    foutResult.open(argv[7], ofstream::out);
    if (foutResult.is_open()) {
        cout << "Output Result file is opened successfully" << endl;
    } else {
        cerr << "Error opening output result file" << endl;
    }
    // ofstream fout1;
    // fout1.open(argv[2], ofstream::out);
    // if (fout1.is_open()) {
    //     cout << "output file is opened successfully" << endl;
    // } else {
    //     cerr << "Error opening output file" << endl;
    // }

    // double gridWidth = 4;
    // double boardWidth = 15*gridWidth;
    // double boardHeight = 19*gridWidth;
    // size_t numLayers = 4;
    // double gridWidth = 8;
    // double boardWidth = 50*gridWidth;
    // double boardHeight = 15*gridWidth;
    // size_t numLayers = 12;
    double gridWidth = 1;

    // For Example 1
    // double boardWidth = 75*gridWidth;
    // double boardHeight = 40*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 40;
    // double offsetY = 40;

    // For Example 2 
    // double boardWidth = 100*gridWidth;
    // double boardHeight = 70*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 95;
    // double offsetY = 45;

    // // For Example 3 
    // double boardWidth = 100*gridWidth;
    // double boardHeight = 65*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 25;
    // double offsetY = 20;

    // // For Example 4 
    // double boardWidth = 80*gridWidth;
    // double boardHeight = 55*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 120;
    // double offsetY = 10;

    // // For Example 5
    // double boardWidth = 90*gridWidth;
    // double boardHeight = 55*gridWidth;
    // size_t numLayers = 5;
    // double offsetX = 110;
    // double offsetY = 10;

    // // For Example 5 (smaller)
    // double boardWidth = 50*gridWidth;
    // double boardHeight = 55*gridWidth;
    // size_t numLayers = 5;
    // double offsetX = 130;
    // double offsetY = 10;


    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 6.0);
    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 10.0);
    SVGPlot plot(fout, 10.0);
    DB db(plot);

    // db.setBoundary(boardWidth, boardHeight);
    db.setFlowWeight(0.5, 0.5);
    // Parser parser(finST, fin, finOb, db, offsetX, offsetY, plot);
    Parser parser(finST, fin, finOb, db, plot);
    parser.parse();

    // // NetworkMgr mgr(db, plot);
    PreMgr preMgr(db, plot);

    preMgr.nodeClustering();

    preMgr.assignPortPolygon();

    preMgr.plotBoundBox();

    

    // // // replace this line with a real parser function
    // // parser.testInitialize(boardWidth, boardHeight, gridWidth);

    // // db.print();

    //time
    auto start = std::chrono::high_resolution_clock::now();
    // time_t start, endTopo, endSizing, endRouting, end;
    // time(&start);
    
    DetailedMgr* detailedMgr = new DetailedMgr(db, plot, foutResult, 2 * db.VIA16D8A24()->padRadius(0));
    detailedMgr->initPortGridMap();
    detailedMgr->check();

    GlobalMgr globalMgr(db, plot);
    
    globalMgr.numIIter = numIIter;
    globalMgr.numVIter = numVIter;
    globalMgr.numIVIter = numIVIter;

    auto endGridMap = std::chrono::high_resolution_clock::now();

    // // // replace this line with a real OASG building function
    // // globalMgr.buildTestOASG();


    // globalMgr.buildOASG(string(argv[3]).find('5') != std::string::npos);
    globalMgr.buildNewOASG();
    // globalMgr.trimOASG();

    // globalMgr.buildOASGXObs();
    // globalMgr.plotOASG();
    // globalMgr.plotDB();
    // assert(false);
    
    // globalMgr.layerDistribution();
    // // //globalMgr.plotRGraph();
    // globalMgr.buildTestNCOASG();
    // globalMgr.plotNCOASG();
    auto endTopo = std::chrono::high_resolution_clock::now();
    // time(&endTopo);
    // // globalMgr.voltageAssignment();
// /*
    globalMgr.genCapConstrs();
    if (db.numLayers() > 1) {
        globalMgr.setUBViaArea(detailedMgr->vNetPortGrid());
    }
    try {
        // globalMgr.voltageDemandAssignment();
        // globalMgr.voltageAssignment();
        // globalMgr.currentDistribution();
        auto start_time = std::chrono::high_resolution_clock::now();
        
        globalMgr.voltCurrOpt();

        // 获取结束时间点
        auto end_time = std::chrono::high_resolution_clock::now();

        // 计算时间差
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // 输出执行时间
        std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
       

        // globalMgr.checkFeasible();
        // globalMgr.checkVoltDemandFeasible();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
    auto endSizing = std::chrono::high_resolution_clock::now();
    // time(&endSizing);
    // globalMgr.plotCurrentPaths();
// */
// /*
    // DetailedMgr detailedMgr(db, plot, 2 * db.VIA16D8A24()->drillRadius());
    delete detailedMgr;
    detailedMgr = new DetailedMgr(db, plot, foutResult, 2 * db.VIA16D8A24()->drillRadius());
    detailedMgr->initGridMap();
    // detailedMgr->initSegObsGridMap();
    //detailedMgr->check();
    //detailedMgr->plotGridMap();
    // // detailedMgr.naiveAStar();
    // detailedMgr->negoAStar(false);
    detailedMgr->orderedAStar(false);
    detailedMgr->removeObsRailGrids();
    detailedMgr->check();
    auto endRouting = std::chrono::high_resolution_clock::now();
    // time(&endRouting);
    // detailedMgr->plotGridMap();
// */
// /*
    detailedMgr->addPortVia();
    detailedMgr->check();
    // // // detailedMgr.plotVia();
    detailedMgr->addViaGrid();
    detailedMgr->check();

    // // // printf("\n==================== print ===================\n");
    // // // detailedMgr.print();

    // printf("\n==================== buildMtx ===================\n");
    // detailedMgr->buildMtx();
    // detailedMgr->printResult();
// */
// /*
    //detailedMgr->SmartDistribute();
    detailedMgr->PostProcessing(true);
    detailedMgr->RemoveIsolatedGrid();
// */
    auto end = std::chrono::high_resolution_clock::now();
    // time(&end);
    // double time_used = double(end - start);
    // int hour = 0, min = 0;
    // if(time_used >= 60){
    //     min = time_used/60;
    //     time_used = time_used - min*60;
    // }
    // if(min >= 60){
    //     hour = min/60;
    //     min = min%60;
    // }

    detailedMgr->plotGridMap();
    //detailedMgr->plotGridMapVoltage();
    //detailedMgr->plotGridMapCurrent();

    // detailedMgr->writeColorMap_v2("../../exp/output/voltageColorMap.txt", 1);
    // detailedMgr->writeColorMap_v2("../../exp/output/currentColorMap.txt", 0);
    // //globalMgr.plotDB();
    // OutputWriter outputWriter;

    // outputWriter.writeTuningResult(ftunRes, numIIter, numVIter, numIVIter, globalMgr._vArea, globalMgr._vOverlap, globalMgr._vSameNetOverlap, globalMgr._vViaArea, globalMgr._vAfterCost);
    // detailedMgr->buildMtx();
    detailedMgr->printResult(true);

    // cout << "|||||||||||||||||||||||" << endl;
    // cout << "|||    Time Used    |||" << endl;
    // cout << "|||||||||||||||||||||||" << endl;
    cerr << "\n ================ Time ================" << endl;
    cout << "Total Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 1e-3 << " seconds" << endl;
    cout << "GridMap Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endGridMap - start).count() << " milliseconds" << endl;
    cout << "Topo Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endTopo - endGridMap).count() << " milliseconds" << endl;
    cout << "Sizing Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endSizing - endTopo).count() << " milliseconds" << endl;
    cout << "Routing Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endRouting - endSizing).count() << " milliseconds" << endl;
    cout << "Shaping Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - endRouting).count() << " milliseconds" << endl;

    foutResult << "\n ================ Time ================" << endl;
    foutResult << "Total Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() * 1e-3 << " seconds" << endl;
    foutResult << "GridMap Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endGridMap - start).count() << " milliseconds" << endl;
    foutResult << "Topo Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endTopo - endGridMap).count() << " milliseconds" << endl;
    foutResult << "Sizing Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endSizing - endTopo).count() << " milliseconds" << endl;
    foutResult << "Routing Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(endRouting - endSizing).count() << " milliseconds" << endl;
    foutResult << "Shaping Time : " << std::chrono::duration_cast<std::chrono::milliseconds>(end - endRouting).count() << " milliseconds" << endl;
    // cout << "Time : " << hour << " hours " << min <<" mins "<< fixed << setprecision(5) << time_used << " sec " << endl; 
    // cout << "Topo Time : " << difftime(endTopo, start) << " sec " << endl;
    // cout << "Sizing Time : " << difftime(endSizing, endTopo) << " sec " << endl;
    // cout << "Routing Time : " << difftime(endRouting, endSizing) << " sec " << endl;


    // // mgr.genRGraph();
    // // // mgr.drawRGraph();
    // // mgr.distrNet();

    // // // draw routing graph
    // // // mgr.drawRGraph(true);
    // // mgr.drawDB();
    // // fout.close();
    return 0;
}