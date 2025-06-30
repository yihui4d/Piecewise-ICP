/**
 * @file main.cpp
 * @brief Test function entry for Piecewise-ICP with pairwise and 4D modes.
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @date 2025-06-06
 * @license Apache-2.0
 * @copyright 2025 Yihui Yang, TU Munich. All Rights Reserved.
 */


#include "Registration.h"
using namespace std;


int main(int argc, char** argv)
{
	//// 4D point cloud Registration
	std::string folderpath = "configuration_files/configuration_4d.txt";
	int epochNum = 20;							// Number of all scan files
	int startEpoch = 0;							// Index of reference epoch in the scan file list
	// pairMode:
	// 0: all scans are registered to Ref;
	// 1: registered to the last scan (fixed interval); 
	// k(>0): registered to the last k scan(fixed interval); 
	// <0: adaptive interval
	int pairMode = -1 ;
	char* configFile = const_cast<char*> (folderpath.c_str());
	bool success1 = PiecewiseICP_4D_call(configFile, startEpoch, epochNum, pairMode, 0.75);
	if (success1)
		std::cout << "\n\n4D point cloud registration completed! ////////////////////////////////////////////  \n\n\n ";
	else
		std::cerr << "\n\n4D point cloud registration fail!!\n\n";


	//// Pairwise Registration
    std::string confile("configuration_files/configuration_pair.txt");
	std::string outfile("results/PairReg/"); // prefix of output files
	const char* confilename = const_cast<char*> (confile.c_str());
	const char* outfilename = const_cast<char*> (outfile.c_str());
	bool success2 = PiecewiseICP_pair_call(confilename, outfilename);
	if(success2)
		std::cout << "\n\nPairwise registration completed! ////////////////////////////////////////////  \n\n\n ";
	else
		std::cerr << "\n\nPairwise registration fail!!\n\n";

	return 0;
}


