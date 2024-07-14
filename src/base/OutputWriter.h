#ifndef OUTPUTWRITER_H
#define OUTPUTWRITER_H

#include "Include.h"
#include <iostream>



class OutputWriter {
    public:
        OutputWriter() {
        }
        ~OutputWriter() {}

    void writeTuningResultNew(std::ofstream& outputFile, vector<int> vIIter, vector<int> vVIter, int numIVIter, vector<double> vArea, vector<double> vOverlap, vector<double> vSameNetOverlap, vector<double> vViaArea, vector<double> vAfterCost) {
        for (const int& element : vIIter) {
            outputFile << element << " ";
        }
        for (int i = vIIter.size(); i < vArea.size(); ++i) {
            outputFile << "0 ";
        }
        outputFile << "\n";
        for (const int& element : vVIter) {
            outputFile << element << " ";
        }
        for (int i = vVIter.size(); i < vArea.size(); ++i) {
            outputFile << "0 ";
        }
        outputFile << "\n";
        outputFile << numIVIter << " ";
        for (int i = 1; i < vArea.size(); ++i) {
            outputFile << "0 ";
        }
        outputFile << "\n";
        for (const double& element : vArea) {
            outputFile << element << " ";
        }
        outputFile << "\n";
        for (const double& element : vOverlap) {
            outputFile << element << " ";
        }
        outputFile << "\n";
        for (const double& element : vSameNetOverlap) {
            outputFile << element << " ";
        }
        outputFile << "\n";
        for (const double& element : vViaArea) {
            outputFile << element << " ";
        }
        outputFile << "\n";
        for (const double& element : vAfterCost) {
            outputFile << element << " ";
        }
    }

    // void writeTuningResult(std::ofstream& outputFile, int numIIter, int numVIter, int numIVIter, vector<double> vArea, vector<double> vOverlap, vector<double> vSameNetOverlap, vector<double> vViaArea, vector<double> vAfterCost) {
    //     //1:v_area, 2:v_Overlap, 3:v_SameNetOverlap, 4:viaArea

    //     vector<int> vXI;
    //     vector<double> vYI;
    //     vector<int> vXV;
    //     vector<double> vYV;
    //     vector<double> data;

    //     // 遍历向量并将每个 double 写入文件
    //     //每個資料的第一行是xI , 再來是yI, xV, yV
    //     int indexIV = 0;
    //     int IVnum = numIIter + numVIter;

    //     for (int i = 0; i < 5; ++i){
    //         data.clear();
    //         //順序自己記得
    //         if(i == 0) {
    //             // outputFile << "vArea\n\n"; 
    //             data = vArea;
    //         }
    //         else if (i == 1) {
    //             // outputFile << "vOverlap\n\n"; 
    //             data = vOverlap;
    //         }
    //         else if (i == 2){
    //             // outputFile << "vSameNetOverlap\n\n";
    //             data = vSameNetOverlap;
    //         }  
    //         else if (i == 3) {
    //             // outputFile << "vViaArea\n\n"; 
    //             data = vViaArea;
    //         }
    //         else if (i == 4) {
    //             // outputFile << "vAfterCost\n\n"; 
    //             data = vAfterCost;
    //         }
    //         for (const double& value : data) {
    //             if( (indexIV % IVnum ) < numIIter || (indexIV % IVnum ) == (IVnum-1) ){
    //                 vXI.push_back(indexIV);
    //                 vYI.push_back(value);
    //             }
    //             if( (indexIV % IVnum ) >= numIIter || (indexIV % IVnum ) == numIIter -1 ){
    //                 vXV.push_back(indexIV);
    //                 vYV.push_back(value);
    //             }
    //             ++ indexIV;
    //         }
    //         for (const double& element : vXI) {
    //             outputFile << element << " ";
    //         }
    //         outputFile << "\n";
    //         for (const double& element : vYI) {
    //             outputFile << element << " ";
    //         }
    //         outputFile << "\n";
    //         for (const double& element : vXV) {
    //             outputFile << element << " ";
    //         }
    //         outputFile << "\n";
    //         for (const double& element : vYV) {
    //             outputFile << element << " ";
    //         }
    //         outputFile << "\n\n"; 

    //         indexIV = 0;
    //         vXI.clear();
    //         vYI.clear();
    //         vXV.clear();
    //         vYV.clear();

    //     }
    // }
        
};

#endif



 //1:v_area, 2:v_Overlap, 3:v_SameNetOverlap, 4:viaArea
        // //羅：匯出3個Vector of double
        // //1:v_area, 2:v_Overlap, 3:v_SameNetOverlap, 4:viaArea
        // string result_dir1 = "/exp/output/tuningRes1.txt";
        // string result_dir2 = "/exp/output/tuningRes2.txt";
        // string result_dir3 = "/exp/output/tuningRes3.txt";
        // string result_dir4 = "/exp/output/tuningRes4.txt";


        // string tuningOutputFile_dir1 = root_dir + result_dir1;
        // string tuningOutputFile_dir2 = root_dir + result_dir2;
        // string tuningOutputFile_dir3 = root_dir + result_dir3;
        // string tuningOutputFile_dir4 = root_dir + result_dir4;

        // std::ofstream tuningOutput_file1(tuningOutputFile_dir1);
        // std::ofstream tuningOutput_file2(tuningOutputFile_dir2);
        // std::ofstream tuningOutput_file3(tuningOutputFile_dir3);
        // std::ofstream tuningOutput_file4(tuningOutputFile_dir4);

        // vector<int> vXI;
        // vector<double> vYI;
        // vector<int> vXV;
        // vector<double> vYV;

        // // 遍历向量并将每个 double 写入文件
        // //每個資料的第一行是xI , 再來是yI, xV, yV
        // int indexIV = 0;
        // int IVnum = numIIter + numVIter;

        // //1
        // for (const double& value : globalMgr._vArea) {
        //     if( (indexIV % IVnum ) < numIIter || (indexIV % IVnum ) == (IVnum-1) ){
        //         vXI.push_back(indexIV);
        //         // cout << "Now we Are Dealing I" <<endl;
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //         vYI.push_back(value);
        //     }
        //     if( (indexIV % IVnum ) >= numIIter || (indexIV % IVnum ) == numIIter -1 ){
        //         // cout << "Now we Are Dealing V" <<endl;
        //         vXV.push_back(indexIV);
        //         vYV.push_back(value);
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //     }
        //     ++ indexIV;
        // }
        // for (const double& element : vXI) {
        //     tuningOutput_file1 << element << " ";
        // }
        // tuningOutput_file1 << "\n";
        // for (const double& element : vYI) {
        //     tuningOutput_file1 << element << " ";
        // }
        // tuningOutput_file1 << "\n";
        // for (const double& element : vXV) {
        //     tuningOutput_file1 << element << " ";
        // }
        // tuningOutput_file1 << "\n";
        // for (const double& element : vYV) {
        //     tuningOutput_file1 << element << " ";
        // }
        // tuningOutput_file1 << "\n"; 

        // cout << "IV num is" << IVnum << endl;
        // cout << " numIIter is " << numIIter << endl;
        // indexIV = 0;
        // vXI.clear();
        // vYI.clear();
        // vXV.clear();
        // vYV.clear();

        // //2
        //  for (const double& value : globalMgr._vOverlap) {
        //     if( (indexIV % IVnum ) < numIIter || (indexIV % IVnum ) == (IVnum-1) ){
        //         vXI.push_back(indexIV);
        //         // cout << "Now we Are Dealing I" <<endl;
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //         vYI.push_back(value);
        //     }
        //     if( (indexIV % IVnum ) >= numIIter || (indexIV % IVnum ) == numIIter -1 ){
        //         // cout << "Now we Are Dealing V" <<endl;
        //         vXV.push_back(indexIV);
        //         vYV.push_back(value);
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //     }
        //     ++ indexIV;
        // }
        // for (const double& element : vXI) {
        //     tuningOutput_file2 << element << " ";
        // }
        // tuningOutput_file2 << "\n";
        // for (const double& element : vYI) {
        //     tuningOutput_file2 << element << " ";
        // }
        // tuningOutput_file2 << "\n";
        // for (const double& element : vXV) {
        //     tuningOutput_file2 << element << " ";
        // }
        // tuningOutput_file2 << "\n";
        // for (const double& element : vYV) {
        //     tuningOutput_file2 << element << " ";
        // }
        // tuningOutput_file2 << "\n"; 

        // // cout << "IV num is" << IVnum << endl;
        // // cout << " numIIter is " << numIIter << endl;
        // indexIV = 0;
        // vXI.clear();
        // vYI.clear();
        // vXV.clear();
        // vYV.clear();

        // //3
        //  for (const double& value : globalMgr._vSameNetOverlap) {
        //     if( (indexIV % IVnum ) < numIIter || (indexIV % IVnum ) == (IVnum-1) ){
        //         vXI.push_back(indexIV);
        //         // cout << "Now we Are Dealing I" <<endl;
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //         vYI.push_back(value);
        //     }
        //     if( (indexIV % IVnum ) >= numIIter || (indexIV % IVnum ) == numIIter -1 ){
        //         // cout << "Now we Are Dealing V" <<endl;
        //         vXV.push_back(indexIV);
        //         vYV.push_back(value);
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //     }
        //     ++ indexIV;
        // }
        // for (const double& element : vXI) {
        //     tuningOutput_file3 << element << " ";
        // }
        // tuningOutput_file3 << "\n";
        // for (const double& element : vYI) {
        //     tuningOutput_file3 << element << " ";
        // }
        // tuningOutput_file3 << "\n";
        // for (const double& element : vXV) {
        //     tuningOutput_file3 << element << " ";
        // }
        // tuningOutput_file3 << "\n";
        // for (const double& element : vYV) {
        //     tuningOutput_file3 << element << " ";
        // }
        // tuningOutput_file3 << "\n"; 

        // // cout << "IV num is" << IVnum << endl;
        // // cout << " numIIter is " << numIIter << endl;
        // indexIV = 0;
        // vXI.clear();
        // vYI.clear();
        // vXV.clear();
        // vYV.clear();
        

        // //4
        //  for (const double& value : globalMgr._vViaArea) {
        //     if( (indexIV % IVnum ) < numIIter || (indexIV % IVnum ) == (IVnum-1) ){
        //         vXI.push_back(indexIV);
        //         // cout << "Now we Are Dealing I" <<endl;
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //         vYI.push_back(value);
        //     }
        //     if( (indexIV % IVnum ) >= numIIter || (indexIV % IVnum ) == numIIter -1 ){
        //         // cout << "Now we Are Dealing V" <<endl;
        //         vXV.push_back(indexIV);
        //         vYV.push_back(value);
        //         // cout << indexIV << endl;
        //         // cout << value << endl;
        //     }
        //     ++ indexIV;
        // }
        // for (const double& element : vXI) {
        //     tuningOutput_file4 << element << " ";
        // }
        // tuningOutput_file4 << "\n";
        // for (const double& element : vYI) {
        //     tuningOutput_file4 << element << " ";
        // }
        // tuningOutput_file4 << "\n";
        // for (const double& element : vXV) {
        //     tuningOutput_file4 << element << " ";
        // }
        // tuningOutput_file4 << "\n";
        // for (const double& element : vYV) {
        //     tuningOutput_file4 << element << " ";
        // }
        // tuningOutput_file4 << "\n"; 

        // // cout << "IV num is" << IVnum << endl;
        // // cout << " numIIter is " << numIIter << endl;
        // indexIV = 0;
        // vXI.clear();
        // vYI.clear();
        // vXV.clear();
        // vYV.clear();


        // // for (const double& value : globalMgr._vViaArea) {
        // //     tuningOutput_file << value << " " ; // 写入每个 double，并在每行后添加换行符
        // // }
        // // tuningOutput_file << "\n";

        // // // 关闭文件流
        // // tuningOutput_file.close();

        // // std::cout << "数据已成功写入到 tuningRes.txt 文件." << std::endl;
        

        // //#################################
        // //羅
        // //