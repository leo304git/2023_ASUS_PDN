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

    // read input file: st components
    finST.open(argv[1], ifstream::in);
    if (finST.is_open()) {
        cout << "input file (st components) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file (st components)" << endl;
    }

    // read input file: parameters
    finPa.open(argv[2], ifstream::in);
    int numVIter, numIIter, numIVIter; 
    if (finPa.is_open()) {
        cout << "input file (Parameters) is opened successfully" << endl;
        std::map<std::string, int> parameters;
        std::string line;
        while (std::getline(finPa, line)) {
            size_t delimiter_pos = line.find("=");
            if (delimiter_pos != std::string::npos) {
                std::string key = line.substr(0, delimiter_pos);
                std::string value_str = line.substr(delimiter_pos + 1);
                key = key.substr(key.find_first_not_of(" "), key.find_last_not_of(" ") + 1);
                value_str = value_str.substr(value_str.find_first_not_of(" "), value_str.find_last_not_of(" ") + 1);
                int value = std::stoi(value_str);
                parameters[key] = value;
            }
        }
        numIVIter = parameters["numIVIter"];
        numIIter = parameters["numIIter"];
        numVIter = parameters["numVIter"];
    } else {
        cerr << "Error opening input file (Parameters)" << endl;
    }

    // read input file: netlist
    fin.open(argv[3], ifstream::in);
    if (fin.is_open()) {
        cout << "input file is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }

    // read input file: obstacles
    finOb.open(argv[4], ifstream::in);
    if (finOb.is_open()) {
        cout << "input file (obstacle) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }

    // open output file: plot
    fout.open(argv[5], ofstream::out);
    if (fout.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }

    // open output file: result of each sizing iteration
    ftunRes.open(argv[6], ofstream::out);
    if (ftunRes.is_open()) {
        cout << "Tuning Result file is opened successfully" << endl;
    } else {
        cerr << "Error opening tuning result file" << endl;
    }

    // open output file: result of the whole design flow
    foutResult.open(argv[7], ofstream::out);
    if (foutResult.is_open()) {
        cout << "Output Result file is opened successfully" << endl;
    } else {
        cerr << "Error opening output result file" << endl;
    }
        
    SVGPlot plot(fout, 10.0);
    DB db(plot);
    db.setFlowWeight(0.5, 0.5);

    Parser parser(finST, fin, finOb, db, plot);
    parser.parse();

    // construct the via region for each port
    PreMgr preMgr(db, plot);
    preMgr.nodeClustering();
    preMgr.assignPortPolygon();
    preMgr.plotBoundBox();

    // Stage1: grid map initialization (for each port's via region)
    auto start = std::chrono::high_resolution_clock::now();
    
    DetailedMgr* detailedMgr = new DetailedMgr(db, plot, foutResult, 2 * db.VIA16D8A24()->padRadius(0));
    detailedMgr->initPortGridMap();
    detailedMgr->check();

    GlobalMgr globalMgr(db, plot);
    globalMgr.numIIter = numIIter;
    globalMgr.numVIter = numVIter;
    globalMgr.numIVIter = numIVIter;

    auto endGridMap = std::chrono::high_resolution_clock::now();

    // Stage2: topology generation
    globalMgr.buildNewOASG();
    // globalMgr.plotOASG();
    // globalMgr.plotDB();
    auto endTopo = std::chrono::high_resolution_clock::now();
    
// /*
    // Stage3: sizing
    globalMgr.genCapConstrs();
    if (db.numLayers() > 1) {
        globalMgr.setUBViaArea(detailedMgr->vNetPortGrid());
    }
    try {
        globalMgr.voltCurrOpt();
        // globalMgr.checkFeasible();
        // globalMgr.checkVoltDemandFeasible();
    } catch (GRBException e) {
        cerr << "Error = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    }
    auto endSizing = std::chrono::high_resolution_clock::now();
    // globalMgr.plotCurrentPaths();
// */
// /*
    // Stage4: routing
    delete detailedMgr;
    detailedMgr = new DetailedMgr(db, plot, foutResult, 2 * db.VIA16D8A24()->drillRadius());
    detailedMgr->initGridMap();
    // detailedMgr.naiveAStar();
    // detailedMgr->negoAStar(false);
    detailedMgr->orderedAStar(false);
    detailedMgr->removeObsRailGrids();
    detailedMgr->check();
    auto endRouting = std::chrono::high_resolution_clock::now();
    // detailedMgr->plotGridMap();
// */
// /*
    // Stage5-1: via insertion
    detailedMgr->addPortVia();
    detailedMgr->check();
    // detailedMgr.plotVia();
    detailedMgr->addViaGrid();
    detailedMgr->check();
    // detailedMgr->buildMtx();
    // detailedMgr->printResult();
// */
// /*
    // Stage5-2: shaping
    detailedMgr->PostProcessing(true);
    detailedMgr->RemoveIsolatedGrid();
    auto end = std::chrono::high_resolution_clock::now();
// */
    // plot and print result
    detailedMgr->plotGridMap();
    //detailedMgr->plotGridMapVoltage();
    //detailedMgr->plotGridMapCurrent();
    // detailedMgr->buildMtx();
    detailedMgr->printResult(true);

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
    
    return 0;
}