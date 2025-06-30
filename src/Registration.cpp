/**
 * @file Registration.cpp
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @date 2025-06
 */

#include "Registration.h"
using namespace std;

// Global indicators for the change of the distance threshold stage
bool g_toStage2 = false;
bool g_toStage3 = false;
// Global flag to control visualization of processed results 
bool g_isVis = false;


bool PiecewiseICP_4D_call(const char* confile, int startEpoch, int epochNum, int pairMode, float overlapThd)
{
	// Load parameters from configuration file
	ConfigPara paraconfig;
	string confilename = confile;
	std::cout << "Loading parameter configuration file: " << confilename << "\n\n";
	if (!readConfigFile(confilename, paraconfig)) {
		std::cerr << "Error: Cannot open configuration file! Aborting.\n\n";
		return false;
	}

	// Folder path of input point clouds and output results
	string inputFolder = paraconfig.FolderFilePath1;
	string outputFolder = paraconfig.FolderFilePath2;
	// Whether to manually set the average point spacing and the initial size of SV
	bool isSetResSVsize = paraconfig.isSetResSVsize;
	// Average point spacing of two point clouds
	float Res1 = paraconfig.PCres1, Res2 = paraconfig.PCres2;
	// Size of generated SV of two point clouds
	float SVsize1 = paraconfig.SVsize1, SVsize2 = paraconfig.SVsize2;
	// Whether to manually set the initial DT
	bool isManualDTinit = paraconfig.isSetDTinit;
	// The initial DT set by the accuracy of coarse registration
	float DTinit = paraconfig.DTinit;
	// Minimum Level of Detection
	float DTmin = paraconfig.DTmin;
	// Whether to visualize processed results
	g_isVis = paraconfig.isVisual;


	// Extract all scan files from the folder
	vector<std::string> scanFileList;
	vector<long> scanTimeList;
	int fileCount = extractAllFilesFromFolder(inputFolder, scanFileList, scanTimeList);
	cout << "--->>> " << fileCount << " scan files are successfully extracted. \n\n";


	// Determine registration pairs (adaptive mode only)
	std::map<int, int> regPairs;
	std::string adaptivePairFile = "RegPairFile.txt";
	if (pairMode < 0) {
		cout << "--->>> Adaptive pair sequence determination... \n";
		calAdaptivePairSequence(scanFileList, startEpoch, DTinit, overlapThd,
			regPairs, adaptivePairFile);
	}
	cout << endl;

	// Prepare output files
	string outfilenameTM(outputFolder);
	outfilenameTM.append("TransMatrices.txt");
	ofstream outTMFile(outfilenameTM.c_str());

	string outfilenameTP(outputFolder);
	outfilenameTP.append("TransParameters.txt");
	ofstream outTPFile(outfilenameTP.c_str());
	
	if (!outTMFile || !outTPFile) {
		std::cerr << "Error: Unable to open output file(s).\n";
		return false;
	}

	outTPFile << "Epoch  Rx[gon]  Ry[gon]  Rz[gon]  tx[m]  ty[m]  tz[m]  "
			  << "Std_Rx[mgon]  Std_Ry[mgon]  Std_Rz[mgon]  "
			  << "Std_tx[mm]  Std_ty[mm]  Std_tz[mm]" << endl;


	////////////////////////////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(scanFileList[startEpoch], *refCloud);
	
	for (int i = startEpoch; i < epochNum-1; ++i) {
		int step = i - startEpoch + 1;		// 1, 2, ... epochNum
		pcl::console::TicToc time;
		time.tic();

		int refIdx = startEpoch;
		if (pairMode > 0) {
			refIdx = (pairMode >= step) ? startEpoch : (i + 1 - pairMode);
		}
		else if (pairMode < 0) {
			refIdx = regPairs[i + 1];
		}
		else {
			refIdx = startEpoch;
		}

		cout << "\n//////////////////////  " << "Process Pair_" << step 
			<< ":  Epoch-" << scanTimeList[refIdx] << " and Epoch-" << scanTimeList[i+1] 
			<< "   //////////////////////////////////////////// \n\n";
		

		// Load point clouds
		std::string outputPrefix = outputFolder + std::to_string(scanTimeList[i + 1]);
		// All scans are directly registered to the reference
		if (pairMode == 0) {
			outputPrefix.append("_Direct2Ref_");
			pcl::copyPointCloud(*refCloud, *oriCloud1);
		}
		// registered to previous scan with fixed interval (interval = pairMode)
		else if (pairMode > 0) {
			outputPrefix.append("_Fixed_");
			pcl::io::loadPCDFile(scanFileList[refIdx], *oriCloud1);
		}
		// registered to previous scan with adaptive interval
		else {
			outputPrefix.append("_Adaptive_");
			pcl::io::loadPCDFile(scanFileList[refIdx], *oriCloud1);
		}
		
		pcl::io::loadPCDFile(scanFileList[i + 1], *oriCloud2);


		// Auto-estimate resolution if not set
		if (!isSetResSVsize) {
			Res1 = calPCresolution(oriCloud1);
			Res2 = calPCresolution(oriCloud2);
		}

		// Perform Piecewise ICP -------------------------------------------------------
		Eigen::Matrix4f transMat;
		std::vector<float> transPara;
		Eigen::MatrixXd VCM;
		bool success = Piecewise_ICP_4D(oriCloud1, oriCloud2,
			isSetResSVsize, Res1, Res2, SVsize1, SVsize2,
			isManualDTinit, DTinit, DTmin,
			outputPrefix, transMat, transPara, VCM);
		if (!success) {
			std::cerr << "Step " << step << " failed. Skipping to next.\n\n";  continue;
		}
		//------------------------------------------------------------------------------


		// Output transformation matrix and VCM to file
		outTMFile << fixed << setprecision(12);
		outTMFile << scanTimeList[i + 1] << "\n";
		// TransMatrix
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				outTMFile << transMat(r, c) << " ";
			}
			outTMFile << "\n";
		}
		// VCM
		for (int r = 0; r < 6; ++r) {
			for (int c = 0; c < 6; ++c) {
				outTMFile << VCM(r, c) << " ";
			}
			outTMFile << "\n";
		}

		// Output transformation parameters to file
		outTPFile << fixed << setprecision(10);
		outTPFile << scanTimeList[i + 1] << " ";
		for (int p = 0; p < 6; ++p) {
			outTPFile << transPara[p] << " ";
		}
		outTPFile << 1000 * sqrt(VCM(0, 0)) * ARC_TO_GON << " " // mgon
				<< 1000 * sqrt(VCM(1, 1)) * ARC_TO_GON << " " // mgon
				<< 1000 * sqrt(VCM(2, 2)) * ARC_TO_GON << " " // mgon
				<< 1000 * sqrt(VCM(3, 3)) << " "	// mm
				<< 1000 * sqrt(VCM(4, 4)) << " "	// mm
				<< 1000 * sqrt(VCM(5, 5)) << "\n";	// mm


		cout << "--->>> Step-" << i - startEpoch + 1 << "  "
			 << "Total computing time: " << int(0.001 * time.toc()) << " s \n\n";
		transPara.clear();
		oriCloud1->clear(); oriCloud2->clear();
	}
	
	outTMFile.close(); outTPFile.close();


	// Calculate transformations to the reference epoch
	std::vector<int> timeStamp;
	std::vector<Eigen::Matrix4f> transMat2Ref;
	std::vector<Eigen::MatrixXd> VCM2Ref;

	string outfilenameTMref(outputFolder);
	outfilenameTMref.append("TransMatrices_toRef.txt");
	string outfilenameTPref(outputFolder);
	outfilenameTPref.append("TransParameters_toRef.txt");
	calTransToReferenceEpoch(outfilenameTM, pairMode, adaptivePairFile, epochNum -startEpoch -1,
		outfilenameTMref, outfilenameTPref,
		timeStamp, transMat2Ref, VCM2Ref);

	// Accuracy analysis (optional if ground-truth transformation is available)
	/**/
	string outfilenameTPerror(outputFolder); 
	outfilenameTPerror.append("TransPara_AbsError.txt");
	calAbsErrorOfTransPara(outfilenameTMref, 
		"data/data_synthetic/defined_transformations.txt",
		epochNum, startEpoch, outfilenameTPerror);
	

	return true;
}



bool PiecewiseICP_pair_call(const char* confile, const char* outfile)
{
	// Load parameters from configuration file
	ConfigPara paraconfig;
	string confilename = confile;
	std::cout << "Loading parameter configuration file: " << confilename << "\n\n";
	if (!readConfigFile(confilename, paraconfig)) {
		std::cerr << "Error: Cannot open configuration file! Aborting.\n\n";
		return false;
	}

	// File path of two point clouds
	string inputFilePC1 = paraconfig.FolderFilePath1;
	string inputFilePC2 = paraconfig.FolderFilePath2;
	// Whether manually set the average point spacing and the initial size of SV
	bool isSetResSVsize = paraconfig.isSetResSVsize;
	// Average point spacing of two point clouds
	float Res1 = paraconfig.PCres1, Res2 = paraconfig.PCres2;
	// Size of generated SV of two point clouds
	float SVsize1 = paraconfig.SVsize1, SVsize2 = paraconfig.SVsize2;
	// Whether to manually set the initial DT
	bool isManualDTinit = paraconfig.isSetDTinit;
	// The initial DT set by the accuracy of coarse registration
	float DTinit = paraconfig.DTinit;
	// Minimum Level of Detection
	float DTmin = paraconfig.DTmin;
	// Whether to visualize processed results
	g_isVis = paraconfig.isVisual;


	// loading original PC
	pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr oriCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(inputFilePC1, *oriCloud1);
	pcl::io::loadPCDFile(inputFilePC2, *oriCloud2);
	if (oriCloud1->size() < 1 || oriCloud2->size() < 1) {
		return false;
	}
	
	// Auto-estimate resolution if not set
	if (!isSetResSVsize) {
		Res1 = calPCresolution(oriCloud1);
		Res2 = calPCresolution(oriCloud2);
	}

	cout << "Original PC-1 point number: " << oriCloud1->size()
		<< "\t Original PC-2 point number: " << oriCloud2->size() << endl;
	cout << "PC-1 avg. point spacing: " << Res1 << "\t PC-2 avg. point spacing: " << Res2 << endl << endl;


	// pre-process original PC (downsampling and SOR)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_prep(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_prep(new pcl::PointCloud<pcl::PointXYZ>);
	PCpreprocessing(oriCloud1, cloud1_prep, true, Res1, 14, 2.7);
	PCpreprocessing(oriCloud2, cloud2_prep, true, Res2, 14, 2.7);


	// Reduction by subtracting the controind of PC1
	pcl::PointCloud<pcl::PointXYZ>::Ptr Redctcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Redctcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f PC1CT;
	pcl::compute3DCentroid(*cloud1_prep, PC1CT);
	float Shift_x = -1 * PC1CT[0];
	float Shift_y = -1 * PC1CT[1];
	float Shift_z = -1 * PC1CT[2];
	// 3D shift (from centroid to 0,0,0)
	Eigen::Matrix4f TransMat_reduction = Eigen::Matrix4f::Identity();
	TransMat_reduction(0, 3) = Shift_x;
	TransMat_reduction(1, 3) = Shift_y;
	TransMat_reduction(2, 3) = Shift_z;
	Eigen::Matrix4f TransMat_reduction_inverse = Eigen::Matrix4f::Identity();
	TransMat_reduction_inverse(0, 3) = -1 * Shift_x;
	TransMat_reduction_inverse(1, 3) = -1 * Shift_y;
	TransMat_reduction_inverse(2, 3) = -1 * Shift_z;
	pcl::transformPointCloud(*cloud1_prep, *Redctcloud1, TransMat_reduction);
	pcl::transformPointCloud(*cloud2_prep, *Redctcloud2, TransMat_reduction);

	cout << "\nPreprocessed PC-1 point number: " << Redctcloud1->size()
		<< "\tPreprocessed PC-2 point number: " << Redctcloud2->size() << endl << endl;

	if (g_isVis)
		visualizeTwoPC(Redctcloud1, "targetCloud", 0, 0, 0, 1, Redctcloud2, "sourceCloud", 1, 0, 0, 1,  1, 1, 1);


	///////////////////////////////////////////////////////////////////////////////////////////
	pcl::console::TicToc time;
	cout << "\n--->>> Compute Core TransMat... ";  time.tic();
	Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
	vector<float> currDT; ///< Update of distance thresholds
	Eigen::MatrixXd VCM;  ///< VCM of TransPara
	// Perform Piecewise-ICP on shifted point clouds
	Piecewise_ICP(
		Redctcloud1, Redctcloud2,
		isSetResSVsize,	Res1, Res2, SVsize1, SVsize2,
		isManualDTinit,	DTinit,	DTmin,
		currDT, transMat, VCM);
	cout << "--->>> Computing time of core TransMat: " << int(0.001*time.toc()) << " s \n\n";
	//////////////////////////////////////////////////////////////////////////////////.////////

	// Calculate the final TransMatrix
	Eigen::Matrix4f transMat_final = TransMat_reduction_inverse * transMat * TransMat_reduction;
	cout << "Final Registration TransMatrix: \n" << transMat_final << endl << endl;

	// Calculate the final transformation parameters
	Eigen::Vector3f ANGLE_TMfinal;
	Eigen::Vector3f TRANS_TMfinal;
	matrix2angle(transMat_final, ANGLE_TMfinal);
	TRANS_TMfinal << transMat_final(0, 3), transMat_final(1, 3), transMat_final(2, 3);
	cout << fixed << setprecision(6);
	cout << "Rotation (degree):\n" << ANGLE_TMfinal * 180.0 / M_PI << endl;
	cout << "Translation (m):\n" << TRANS_TMfinal << endl << endl;

	// Calculate the transformed source cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr transOriCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*oriCloud2, *transOriCloud2, transMat_final);

	if (g_isVis)
		visualizeThreePC(oriCloud1, "target cloud1", 1, 0, 0, 1, oriCloud2, "source cloud", 0, 1, 0, 1,
			transOriCloud2, "registered source cloud", 0, 0, 1, 1, 0, 0, 0);


	// Save the final 4x4 transformation matrix, parameters, VCM into a *.txt file
	string outfilenameTM(outfile);
	outfilenameTM.append("TransMatrix.txt");
	ofstream outTransFile(outfilenameTM.c_str());
	if (!outTransFile) {
		std::cerr << "Cannot open TransMatrix.txt for writing!\n\n";
		return false;
	}
	
	outTransFile << "4x4 Transformation Matrix:\n";
	outTransFile << fixed << setprecision(12);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			outTransFile << transMat_final(i, j) << " ";
		}
		outTransFile << "\n";
	}
	outTransFile << endl;
	
	outTransFile << "Rotation Angles (unit: gon):\n" << fixed << setprecision(10)
		<< "Rx = " << ANGLE_TMfinal[0] * ARC_TO_GON << "\n"
		<< "Ry = " << ANGLE_TMfinal[1] * ARC_TO_GON << "\n"
		<< "Rz = " << ANGLE_TMfinal[2] * ARC_TO_GON << "\n";
	outTransFile << "Translation (unit: m):\n"
		<< "tx = " << TRANS_TMfinal[0] << "\n"
		<< "ty = " << TRANS_TMfinal[1] << "\n"
		<< "tz = " << TRANS_TMfinal[2] << "\n";
	outTransFile << endl;
	
	outTransFile << "6x6 Variance-Covariance Matrix of transformation parameters:\n";
	outTransFile << fixed << setprecision(12);
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			outTransFile << VCM(i, j) << " ";
		}
		outTransFile << "\n";
	}
	outTransFile << endl;
	
	outTransFile << "Standard Deviations of estimated transformation parameters:\n";
	outTransFile << fixed << setprecision(10)
				 << "Std_Rx = " << 1000 * ARC_TO_GON * sqrt(VCM(0, 0)) << " mgon\n"
				 << "Std_Ry = " << 1000 * ARC_TO_GON * sqrt(VCM(1, 1)) << " mgon\n"
				 << "Std_Rz = " << 1000 * ARC_TO_GON * sqrt(VCM(2, 2)) << " mgon\n"
				 << "Std_tx = " << 1000 * sqrt(VCM(3, 3)) << " mm\n"
				 << "Std_ty = " << 1000 * sqrt(VCM(4, 4)) << " mm\n"
				 << "Std_tz = " << 1000 * sqrt(VCM(5, 5)) << " mm\n";
	outTransFile.close();
	cout << "--->>> Transformation results saved.\n";


	// Save the registered source cloud into a *.pcd file
	string outfilenamePC2(outfile);
	outfilenamePC2.append("RegisteredSourceCloud.pcd");
	pcl::io::savePCDFileBinary(outfilenamePC2, *transOriCloud2);
	cout << "--->>> Registered source cloud saved.\n\n";

	return true;
}



bool Piecewise_ICP_4D(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	bool isSetResSVsize, float Res1, float Res2, float SVsize1, float SVsize2,
	bool isManualDTinit, float DTinit, float DTmin, std::string outfileIdx, 
	Eigen::Matrix4f &transMat, std::vector<float> &transPara, Eigen::MatrixXd &VCM)
{
	cout << "Original PC-1 point number: " << cloud1->size() 
		<< "\t Original PC-2 point number: " << cloud2->size() << endl;
	cout << "PC-1 avg. point spacing: " << Res1 << "\t PC-2 avg. point spacing: " << Res2 << endl << endl;

	// pre-process original PC (downsampling and SOR)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_prep(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_prep(new pcl::PointCloud<pcl::PointXYZ>);
	PCpreprocessing(cloud1, cloud1_prep, true, Res1, 14, 5.0);
	PCpreprocessing(cloud2, cloud2_prep, true, Res2, 14, 5.0);


	// Reduction by subtracting the controind of PC1
	pcl::PointCloud<pcl::PointXYZ>::Ptr Redctcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Redctcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f PC1CT;
	pcl::compute3DCentroid(*cloud1_prep, PC1CT);
	float Shift_x = -1 * PC1CT[0];
	float Shift_y = -1 * PC1CT[1];
	float Shift_z = -1 * PC1CT[2];
	Eigen::Matrix4f TransMat_reduction = Eigen::Matrix4f::Identity();
	TransMat_reduction(0, 3) = Shift_x;
	TransMat_reduction(1, 3) = Shift_y;
	TransMat_reduction(2, 3) = Shift_z;
	Eigen::Matrix4f TransMat_reduction_inverse = Eigen::Matrix4f::Identity();
	TransMat_reduction_inverse(0, 3) = -1 * Shift_x;
	TransMat_reduction_inverse(1, 3) = -1 * Shift_y;
	TransMat_reduction_inverse(2, 3) = -1 * Shift_z;
	pcl::transformPointCloud(*cloud1_prep, *Redctcloud1, TransMat_reduction);
	pcl::transformPointCloud(*cloud2_prep, *Redctcloud2, TransMat_reduction);

	cout << "Preprocessed PC-1 point number: " << Redctcloud1->size() 
		<< "\tPreprocessed PC-2 point number: " << Redctcloud2->size() << endl << endl;

	if (g_isVis)
		visualizeTwoPC(Redctcloud1, "targetCloud", 0, 0, 0, 1, Redctcloud2, "sourceCloud", 1, 0, 0, 1, 1, 1, 1);


	///////////////////////////////////////////////////////////////////////////////////////////
	pcl::console::TicToc time;
	cout << "\n--->>> Compute Core TransMat... ";  time.tic();
	Eigen::Matrix4f computedTransMat = Eigen::Matrix4f::Identity();
	vector<float> currDT; ///< Update of distance thresholds
	// Perform Piecewise-ICP on shifted point clouds
	Piecewise_ICP(
		Redctcloud1, Redctcloud2,
		isSetResSVsize, Res1, Res2, SVsize1, SVsize2,
		isManualDTinit, DTinit, DTmin,
		currDT, computedTransMat, VCM);

	cout << "--->>> Computing time of core TransMat: " << int(0.001*time.toc()) << " s \n\n";
	//////////////////////////////////////////////////////////////////////////////////.////////

	// Calculate the final TransMatrix
	Eigen::Matrix4f transMat_final = TransMat_reduction_inverse * computedTransMat * TransMat_reduction;
	cout << "Final Registration TransMatrix: \n" << transMat_final << endl << endl;

	// Calculate the final transformation parameters
	Eigen::Vector3f ANGLE_TMfinal;
	Eigen::Vector3f TRANS_TMfinal;
	matrix2angle(transMat_final, ANGLE_TMfinal);
	TRANS_TMfinal << transMat_final(0, 3), transMat_final(1, 3), transMat_final(2, 3);
	cout << fixed << setprecision(6);
	cout << "Rotation (degree):\n" << ANGLE_TMfinal * 180.0 / M_PI << endl;
	cout << "Translation (m):\n" << TRANS_TMfinal << endl << endl;

	transMat = transMat_final;
	transPara.resize(6);
	transPara[0] = ANGLE_TMfinal[0] * ARC_TO_GON;
	transPara[1] = ANGLE_TMfinal[1] * ARC_TO_GON;
	transPara[2] = ANGLE_TMfinal[2] * ARC_TO_GON;
	transPara[3] = TRANS_TMfinal[0];
	transPara[4] = TRANS_TMfinal[1];
	transPara[5] = TRANS_TMfinal[2];

	// Calculate the transformed source cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr transOriCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud2, *transOriCloud2, transMat_final);

	if (g_isVis)
		visualizeThreePC(cloud1, "targetCloud", 1, 0, 0, 1, cloud2, "sourceCloud", 0, 1, 0, 1,
			transOriCloud2, "registeredSourceCloud", 0, 0, 1, 1, 0, 0, 0);


	// Save the final 4x4 transformation matrix, parameters, VCM into a *.txt file
	string outfilenameTM(outfileIdx);
	outfilenameTM.append("TransMatrix.txt");
	ofstream outTransFile(outfilenameTM.c_str());
	if (!outTransFile) {
		std::cerr << "Cannot open TransMatrix.txt for writing!\n\n";
		return false;
	}

	outTransFile << "4x4 Transformation Matrix:\n";
	outTransFile << fixed << setprecision(12);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			outTransFile << transMat_final(i, j) << " ";
		}
		outTransFile << "\n";
	}
	outTransFile << endl;

	outTransFile << "Rotation Angles (unit: gon):\n" << fixed << setprecision(10)
		<< "Rx = " << ANGLE_TMfinal[0] * ARC_TO_GON << "\n"
		<< "Ry = " << ANGLE_TMfinal[1] * ARC_TO_GON << "\n"
		<< "Rz = " << ANGLE_TMfinal[2] * ARC_TO_GON << "\n";
	outTransFile << "Translation (unit: m):\n"
		<< "tx = " << TRANS_TMfinal[0] << "\n"
		<< "ty = " << TRANS_TMfinal[1] << "\n"
		<< "tz = " << TRANS_TMfinal[2] << "\n";
	outTransFile << endl;


	outTransFile << "6x6 Variance-Covariance Matrix of transformation parameters:\n";
	outTransFile << fixed << setprecision(12);
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			outTransFile << VCM(i, j) << " ";
		}
		outTransFile << "\n";
	}
	outTransFile << endl;

	outTransFile << "Standard Deviations of estimated transformation parameters:\n";
	outTransFile << fixed << setprecision(10)
		<< "Std_Rx = " << 1000 * ARC_TO_GON * sqrt(VCM(0, 0)) << " mgon\n"
		<< "Std_Ry = " << 1000 * ARC_TO_GON * sqrt(VCM(1, 1)) << " mgon\n"
		<< "Std_Rz = " << 1000 * ARC_TO_GON * sqrt(VCM(2, 2)) << " mgon\n"
		<< "Std_tx = " << 1000 * sqrt(VCM(3, 3)) << " mm\n"
		<< "Std_ty = " << 1000 * sqrt(VCM(4, 4)) << " mm\n"
		<< "Std_tz = " << 1000 * sqrt(VCM(5, 5)) << " mm\n";
	outTransFile.close();
	cout << "--->>> Transformation results saved.\n\n"; 


	cloud1_prep->clear();
	cloud2_prep->clear();
	Redctcloud1->clear();
	Redctcloud2->clear();
	return true;
}



bool calAdaptivePairSequence(std::vector<std::string> fileNameList, int startEpoch,
	float DTinit, float ratioThd, std::map<int, int> &RegPairs, std::string adaptivePairFile)
{
	int IdxTarget = startEpoch;
	for (int j = startEpoch +1; j < fileNameList.size(); ++j) {
		cout << "--> Computing for source cloud - " << j << " ... ";
		float OverlapRatio;
		for (int i = IdxTarget; i < j; ++i) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile(fileNameList[i], *cloud1);
			pcl::io::loadPCDFile(fileNameList[j], *cloud2);
			OverlapRatio = calOverlapRatioByC2Cdist(cloud1, cloud2, DTinit);
			IdxTarget = i;
			if (OverlapRatio > ratioThd)
				break;
		}
		// Establish pair index
		RegPairs.insert(std::make_pair(j- startEpoch, IdxTarget - startEpoch));
		cout << "Pair: " << IdxTarget- startEpoch << " - " << j- startEpoch 
			<< ";  Overlap ratio = " << 100* OverlapRatio << "% \n";
	}
	if (RegPairs.size() != fileNameList.size()-1)
		return false;

	// Output Reg-pair (index are relative to startEpoch: first = key = sourcePC (1,2 ... PCnum-1); second = value = targetPC (0,1 ... PCnum-2))
	ofstream outRegPair(adaptivePairFile);
	if (!outRegPair) {
		std::cerr << "Error: Cannot open adaptivePairFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	for (auto iter = RegPairs.begin(); iter != RegPairs.end(); ++iter) {
		outRegPair << iter->first << " " << iter->second << endl;
	}
	outRegPair.close();

	return true;
}



float calOverlapRatioByC2Cdist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
	float DTinit)
{
	// Calculate C2C distance by nearest correspondences
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> coreC2C;
	coreC2C.setInputTarget(cloud1);
	coreC2C.setInputSource(cloud2);
	boost::shared_ptr<pcl::Correspondences> corC2C(new pcl::Correspondences);
	coreC2C.determineCorrespondences(*corC2C, DBL_MAX);
	// Count distances under DTint
	std::vector<float> C2CdistUnderDTint;
	int CorNum = corC2C->size();
	int UnderInitNum = 0;
	for (int i = 0; i < CorNum; ++i) {
		float Dtemp = sqrt(corC2C->at(i).distance);
		if (Dtemp < DTinit) {
			C2CdistUnderDTint.push_back(Dtemp);
			UnderInitNum++;
		}
	}
	return float(UnderInitNum) / float(CorNum);
}



void Piecewise_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	bool isSetResSVsize, float Res1, float Res2, float SVsize1, float SVsize2,
	bool isManualDTinit, float DTinit, float DTmin,
	std::vector<float> &DTseries, Eigen::Matrix4f &transMat, Eigen::MatrixXd &VCM)
{
	g_toStage2 = false;
	g_toStage3 = false;

	// Empirically determine the initial DT if it is not manually set
	if (!isManualDTinit) {
		double Dist75 = calPercentileDistBetween2PC(cloud1, cloud2, 0.75);
		DTinit = Dist75 * 3.0;
	}
	float currDT = DTinit;
	cout << endl << "DT initial value = " << currDT << " m \n\n";

	// Empirically determining the initial size of SV by average point spacing
	float SVRes1 = Res1 * 10;
	float SVRes2 = Res2 * 10;
	// Manually setting the initial size of SV
	if (isSetResSVsize) {
		SVRes1 = SVsize1;  SVRes2 = SVsize2;
	}

	// Define point clouds array to store the generated patches
	pcl::PointCloud<pcl::PointXYZ> *SVcloud1;
	pcl::PointCloud<pcl::PointXYZ> *SVcloud2;
	// Define point clouds to store the centroids of patches
	pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	// Define point clouds to store the boundary points of patches
	pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud2(new pcl::PointCloud<pcl::PointXYZ>);

	// Patch segmentation, selection and refinement
	int numSVcloud1 = PatchGenerationAndRefinement(cloud1, SVRes1, CTcloud1, BPcloud1, SVcloud1, g_isVis);
	int numSVcloud2 = PatchGenerationAndRefinement(cloud2, SVRes2, CTcloud2, BPcloud2, SVcloud2, g_isVis);

	cout << "PC-1 selected patch number: " << numSVcloud1 << "\tPC-2 selected patch number: " << numSVcloud2 << endl;
	cout << "---------------------------------------------------------------------------- \n\n";


	// Calculate STD of BP and CT of each patch
	std::vector<float> BPstd1, BPstd2;
	std::vector<float> CTstd1, CTstd2;
	calBPandCTSTD(SVcloud1, numSVcloud1, BPstd1, CTstd1);
	calBPandCTSTD(SVcloud2, numSVcloud2, BPstd2, CTstd2);


	// computed transMatrix in current iteration
	Eigen::Matrix4f transMatCurrent = Eigen::Matrix4f::Identity();
	// current iteration count
	int countIteration = 0;
	// corner change of bounding box
	float BBchange_1 = 0.0f;
	float BBchange_2 = 0.0f;
	
	DTseries.clear();
	DTseries.push_back(currDT);

	// Piecewise-ICP iteration
	cout << "Start Piecewise-ICP iteration... \n\n";
	while (!g_toStage3) {
		transMatCurrent =     
			PwICP_singleIteration(cloud1, cloud2, Res1, Res2, SVRes1, SVRes2,
				SVcloud1, SVcloud2, CTcloud1, CTcloud2, BPcloud1, BPcloud2,
				CTstd1, BPstd2, DTmin, currDT, BBchange_1, BBchange_2, VCM);

		// update the current transMat and DT
		transMat = transMatCurrent * transMat;
		DTseries.push_back(currDT);
		countIteration++;

		cout << "--->>> Iteration No." << countIteration << " | Current DT = " << currDT * 100 << " cm. "
			<< "\t Stage_2: " << g_toStage2 << " | Stage_3: " << g_toStage3 << endl
			<< "================================================================================ \n" << endl;
	}
	// Delete patch point cloud array
	delete[] SVcloud1; SVcloud1 = NULL;
	delete[] SVcloud2; SVcloud2 = NULL;
	cout << "Computed TransMat after " << countIteration << " iterations is: \n" << transMat;
	cout << "\n*******************************************\n\n";
}



Eigen::Matrix4f PwICP_singleIteration(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	float Res1, float Res2, float SVRes1, float SVRes2,
	pcl::PointCloud<pcl::PointXYZ>* &SVcloud1, pcl::PointCloud<pcl::PointXYZ>* &SVcloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud2,
	std::vector<float> CTstd1, std::vector<float> BPstd2, float DTmin,
	float &currDT, float &BBchange_1, float &BBchange_2, Eigen::MatrixXd &VCM)
{
	// stable points identified in PC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr stablePC2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stableCT2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr stableCT2_withNorm(new pcl::PointCloud<pcl::PointNormal>);
	// unstable points identified in PC2
	pcl::PointCloud<pcl::PointXYZ>::Ptr unstablePC2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr unstableCT2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr unstableCT2_withNorm(new pcl::PointCloud<pcl::PointNormal>);
	// number of patches in PC2
	int SVnumPC2 = CTcloud2->size();
	// Keep current DT no less than minimum LoDetection
	if (currDT <= DTmin)
		currDT = DTmin;

	// check the number of stable points 
	if (4 > SVnumPC2) {
		std::cerr << "Error: No enough stable points left (<4)! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	pcl::console::TicToc time;
	///////////////////////////////////////////////////////////////////////////////////////////

	//// (1) Establish all correspondences
	// centroids in PC2 to the nearest centroids in PC1 
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> coreCT_CT;
	coreCT_CT.setInputTarget(CTcloud1);
	coreCT_CT.setInputSource(CTcloud2);
	boost::shared_ptr<pcl::Correspondences> corCT_CT(new pcl::Correspondences);
	coreCT_CT.determineCorrespondences(*corCT_CT, DBL_MAX);
	// boundary points in PC2 to the nearest centroids in PC1 
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> coreBP_CT;
	coreBP_CT.setInputTarget(CTcloud1);
	coreBP_CT.setInputSource(BPcloud2);
	boost::shared_ptr<pcl::Correspondences> corBP_CT(new pcl::Correspondences);
	coreBP_CT.determineCorrespondences(*corBP_CT, DBL_MAX);


	//// (2) Calculate LoDetection for each patch
	float max2minLoD = 2.0; ///< Empirical ratio of LoDetection_max to LoDetection_min
	float maxLoD = DTmin * max2minLoD;
	float minLoD = DTmin;

	std::vector<float> LoDet;
	for (int i = 0; i < SVnumPC2; ++i) {
		float sigm1 = CTstd1[corCT_CT->at(i).index_match];
		float sigm2 = BPstd2[i];
		float LoD =  1.96 * sqrt(sigm1*sigm1 + sigm2*sigm2);  ///< 1.96 at 95% confidence level  
		if (LoD > maxLoD)
			LoDet.push_back(maxLoD);
		else if (LoD < minLoD)
			LoDet.push_back(minLoD);
		else
			LoDet.push_back(LoD);
	}
	// Get minimum and maximum LoD
	float LoDet_min = *min_element(LoDet.begin(), LoDet.end());
	float LoDet_max = *max_element(LoDet.begin(), LoDet.end());
	//cout << "\n--->>> Get minimum LoD: " << LoDet_min << " m";
	//cout << "\n--->>> Get maximum LoD: " << LoDet_max << " m\n\n";


	//// (3) Calculate correspondence distances
	float resDis = 0;
	float DisDx, DisDy, DisDz;

	// Point-to-plane distances of CT2-to-CT1
	std::vector<float> Pt2Pl_CT_CT;
	std::vector<float> Pt2Pt_CT_CT;
	for (int i = 0; i < SVnumPC2; ++i) {
		float nmlx = 0, nmly = 0, nmlz = 2;
		if (calPatchNormal(SVcloud1[corCT_CT->at(i).index_match], nmlx, nmly, nmlz)) {
			DisDx = CTcloud1->points[corCT_CT->at(i).index_match].x - CTcloud2->points[i].x;
			DisDy = CTcloud1->points[corCT_CT->at(i).index_match].y - CTcloud2->points[i].y;
			DisDz = CTcloud1->points[corCT_CT->at(i).index_match].z - CTcloud2->points[i].z;
			resDis = fabs(DisDx*nmlx + DisDy * nmly + DisDz * nmlz);
		}
		else {
			std::cerr << "Patch normal calculation fails! \n\n";
			resDis = sqrt(corCT_CT->at(i).distance);
		}
		Pt2Pl_CT_CT.push_back(resDis);
		Pt2Pt_CT_CT.push_back(sqrt(corCT_CT->at(i).distance));
	}

	// Point-to-plane distances of BP2-CT1 distances
	std::vector<float> Pt2Pl_BP_CT;
	for (int i = 0; i < BPcloud2->size(); ++i) {
		float nmlx = 0, nmly = 0, nmlz = 2;
		if (calPatchNormal(SVcloud1[corBP_CT->at(i).index_match], nmlx, nmly, nmlz)) {
			DisDx = CTcloud1->points[corBP_CT->at(i).index_match].x - BPcloud2->points[i].x;
			DisDy = CTcloud1->points[corBP_CT->at(i).index_match].y - BPcloud2->points[i].y;
			DisDz = CTcloud1->points[corBP_CT->at(i).index_match].z - BPcloud2->points[i].z;
			resDis = fabs(DisDx*nmlx + DisDy * nmly + DisDz * nmlz);
		}
		else {
			std::cerr << "Patch normal calculation fails! \n\n";
			resDis = sqrt(corBP_CT->at(i).distance);
		}
		Pt2Pl_BP_CT.push_back(resDis);
	}


	//// (4) Classify the stable and unstable patches and identify stable areas in PC2
	// Set the maximum distance constraint using patch size
	float DTctct = currDT + 1 *(SVRes1 + SVRes2);	///< DT + PatchSize 
	// Number of stable SV in PC2
	int stableSVnum = 0;
	// Obtain centroids of all SV with associated normals
	pcl::PointCloud<pcl::PointNormal>::Ptr CTcloud1_withNorm(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr CTcloud2_withNorm(new pcl::PointCloud<pcl::PointNormal>);
	generateCentroidCloudWithPatchNormals(CTcloud1, SVcloud1, CTcloud1_withNorm);
	generateCentroidCloudWithPatchNormals(CTcloud2, SVcloud2, CTcloud2_withNorm);
	int BPidx = 0;
	for (int i = 0; i < SVnumPC2; ++i) {
		// according to BP
		bool BPdisPass = true;
		for (int k = 0; k < 6; ++k) {
			if (currDT <= LoDet[i]) {
				if (LoDet[i] < Pt2Pl_BP_CT[BPidx + k])
					BPdisPass = false;
			}
			else {
				if (currDT < Pt2Pl_BP_CT[BPidx + k])
					BPdisPass = false;
			}
		}
		BPidx += 6;

		// according to CT
		bool CTdisPass = true;
		if (currDT <= LoDet[i]) {
			if (LoDet[i] < Pt2Pl_CT_CT[i])
				CTdisPass = false; 
		}
		else {
			if (currDT < Pt2Pl_CT_CT[i])
				CTdisPass = false;
		}

		// Classify stable/unstable patches 
		if (CTdisPass && BPdisPass && (Pt2Pt_CT_CT[i] < DTctct)) {
			*stablePC2 += SVcloud2[i];
			stableCT2_withNorm->push_back(CTcloud2_withNorm->points[i]);
			stableSVnum++;
		}
		else {
			*unstablePC2 += SVcloud2[i];
			unstableCT2_withNorm->push_back(CTcloud2_withNorm->points[i]);
		}
	}
	// Check number of stable points 
	if (4 > stableCT2_withNorm->size()) {
		std::cerr << "Error: No enough stable points left, no enough overlapping areas!!! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	pcl::copyPointCloud(*stableCT2_withNorm, *stableCT2);
	pcl::copyPointCloud(*unstableCT2_withNorm, *unstableCT2);
	cout << "Ratio of stable points: " 
		<< 100.0 * stablePC2->size() / float(stablePC2->size() + unstablePC2->size()) << " %\n";


	//// (5) Perform point-to-plane ICP between PC2(stable areas) and PC1
	Eigen::Matrix4f transMatICP = Eigen::Matrix4f::Identity();
	// Fine registration by CT2CT-P2P-ICP:
	transMatICP = P2PICPwithPatchNormal(CTcloud1_withNorm, stableCT2_withNorm, 1e-6);


	//// (6) Compute the corner change of bounding box (BB) after tranformation
	double BoundingBox[6];
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(Res2 * 2);
	octree.setInputCloud(cloud2);
	octree.defineBoundingBox();
	octree.addPointsFromInputCloud();
	octree.getBoundingBox(BoundingBox[0], BoundingBox[1], BoundingBox[2], BoundingBox[3], BoundingBox[4], BoundingBox[5]);
	float maxBBchange = calBoundingBoxCornerChange(BoundingBox, transMatICP);
	cout << "Change of bounding box = " << maxBBchange * 100 << " cm \n";


	//// (7) Update DT for next iteration
	if (!g_toStage2 && maxBBchange < minLoD) {
		g_toStage2 = true;
		cout << "  DT changed to Stage 2!" << endl;
	}
	else if (currDT == LoDet_min) {
		g_toStage3 = true;
		cout << "  DT changed to Stage 3 (End)!" << endl;
	}
	else
		cout << "  DT stage has not changed." << endl;

	// Stage-2
	if (!g_toStage2) {		
		double Dist75 = calPercentileDistBetween2PC(cloud1, stablePC2, 0.75);
		if (currDT > Dist75)
			currDT = Dist75;  ///< Guarantee a monotonically decreasing DT
		else {
			g_toStage2 = true;
			cout << "  DT changed to Stage 2!" << endl;
		}
		if (currDT <= LoDet_min)
			currDT = LoDet_min;
		BBchange_2 = BBchange_1;
		BBchange_1 = maxBBchange;
	}
	// Stage-3
	if (g_toStage2 && !g_toStage3) {
		float upperBound = 0.8;
		float lowerBound = 0.5;
		float alpha = abs(BBchange_1 / BBchange_2); ///< (k-1)/(k-2) 
		if (isnan(alpha) || isinf(alpha))
			currDT = currDT * upperBound;
		else if (alpha < lowerBound)
			currDT = currDT * lowerBound;
		else if (alpha > upperBound)
			currDT = currDT * upperBound;
		else
			currDT = currDT * alpha;

		if (currDT <= LoDet_min)
			currDT = LoDet_min;
		BBchange_2 = BBchange_1;
		BBchange_1 = maxBBchange;
	}

	// Visualize stable and unstable areas in PC2
	if (g_isVis && g_toStage3)
		visualizeTwoPC(stablePC2, "ep2stable", 1, 0.5, 0,  1, unstablePC2, "ep2unstable", 0.18, 0.46, 0.71,  1, 1, 1, 1);


	//// (8) Apply current transformation to update PC2, centroids, BPs, patches
	pcl::PointCloud<pcl::PointXYZ> cloud2temp;
	pcl::transformPointCloud(*cloud2, cloud2temp, transMatICP);
	pcl::copyPointCloud(cloud2temp, *cloud2);
	pcl::transformPointCloud(*CTcloud2, cloud2temp, transMatICP);
	pcl::copyPointCloud(cloud2temp, *CTcloud2);
	pcl::transformPointCloud(*BPcloud2, cloud2temp, transMatICP);
	pcl::copyPointCloud(cloud2temp, *BPcloud2);
	for (int i = 0; i < CTcloud2->size(); ++i) {
		cloud2temp.points.resize(SVcloud2[i].points.size());
		pcl::transformPointCloud(SVcloud2[i], cloud2temp, transMatICP);
		pcl::copyPointCloud(cloud2temp, SVcloud2[i]);
	}


	//// (9) Calculate VCM of transformation parameters and the variance of each registered point after the last iteration
	if (g_toStage3) {
		VCM.resize(6, 6);
		VCM = calTransParaVCM(CTcloud1, CTcloud1_withNorm, stableCT2);
	}


	stablePC2->clear();
	stableCT2->clear();
	stableCT2_withNorm->clear();
	unstablePC2->clear();
	unstableCT2->clear();
	unstableCT2_withNorm->clear();

	return transMatICP;
}




void calTransToReferenceEpoch(std::string transMatFile, int pairMode, std::string adaptivePairFile, int epochNum,
	std::string transMat2RefFile, std::string transPara2RefFile,
	std::vector<int> &timeStamp, std::vector<Eigen::Matrix4f> &allTransMat2Ref, std::vector<Eigen::MatrixXd> &allVCM2Ref)
{
	cout << "\n--->>> Calculate the transformation of each epoch to the reference epoch...\n";
	// Read pairwise transformation matrix and VCM file
	ifstream inTMFile;
	inTMFile.open(transMatFile, ios::in);
	if (!inTMFile) {
		std::cerr << "Error: Cannot open transMatFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	std::vector<Eigen::Matrix4f> allTransMat;
	std::vector<Eigen::MatrixXd> allVCM;
	for (int i = 0; i < epochNum; ++i) {
		int Timetemp;
		inTMFile >> Timetemp;
		int TimestampHour = Timetemp;
		Eigen::Matrix4f Mattemp = Eigen::Matrix4f::Identity();
		Eigen::MatrixXd VCMtemp = Eigen::MatrixXd::Zero(6, 6);
		for (int r1 = 0; r1 < 4; ++r1) {
			for (int c1 = 0; c1 < 4; ++c1) {
				inTMFile >> Mattemp(r1, c1);
			}
		}
		for (int r2 = 0; r2 < 6; ++r2) {
			for (int c2 = 0; c2 < 6; ++c2) {
				inTMFile >> VCMtemp(r2, c2);
			}
		}
		timeStamp.push_back(TimestampHour);
		allTransMat.push_back(Mattemp);
		allVCM.push_back(VCMtemp);
	}
	inTMFile.close();

	// Read determined adaptive pair sequence
	std::map<int, int> RegPair;
	if (pairMode < 0) {
		ifstream inPairFile;
		inPairFile.open(adaptivePairFile, ios::in);
		if (!inPairFile) {
			std::cerr << "Error: Cannot open adaptivePairFile! Aborting.\n";
			std::exit(EXIT_FAILURE);
		}
		for (int i = 0; i < epochNum; i++) {
			int IdxSource, IdxTarget;
			inPairFile >> IdxSource >> IdxTarget;
			RegPair.insert(std::make_pair(IdxSource, IdxTarget));
		}
		inPairFile.close();
		for (auto iter = RegPair.begin(); iter != RegPair.end(); ++iter) {
			cout << "--> Optimal pair: " << iter->first << "-->" << iter->second << endl;
		}
	}


	// Output accumulated transMat 
	ofstream outTMFile(transMat2RefFile.c_str());
	if (!outTMFile) {
		std::cerr << "Error: Cannot open transMat2RefFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	// Output accumulated transPara
	ofstream outTPFile(transPara2RefFile.c_str());
	if (!outTPFile) {
		std::cerr << "Error: Cannot open transPara2RefFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	outTPFile << "Epoch  Rx[gon]  Ry[gon]  Rz[gon]  tx[m]  ty[m]  tz[m]  "
			  << "Std_Rx[mgon]  Std_Ry[mgon]  Std_Rz[mgon]  "
			  << "Std_tx[mm]  Std_ty[mm]  Std_tz[mm]" << endl;


	// Calculate accumulated TransMat and VCM
	for (int i = 0; i < epochNum; i++) {
		Eigen::Matrix4f accumTransMat = Eigen::Matrix4f::Identity();
		Eigen::MatrixXd accumVCM = Eigen::MatrixXd::Zero(6, 6);

		if (pairMode < 0) {
			accumTransMat = allTransMat[i];
			accumVCM = allVCM[i];
			Eigen::MatrixXd Adjoint = Eigen::MatrixXd::Zero(6, 6);
			Eigen::MatrixXd SSmat = Eigen::MatrixXd::Zero(3, 3);
			int IdxTargCurrt = i + 1;
			int RegTimes = 1;
			for (int j = 0; j < i + 1; j++) {
				// Get the index of current target epoch
				IdxTargCurrt = RegPair[IdxTargCurrt];
				// Stop accumulation when target epoch goes to the first epoch
				if (IdxTargCurrt == 0)
					break;
				Eigen::Matrix4f MatAccumNew = allTransMat[IdxTargCurrt - 1];

				////--------------------------------
				accumTransMat = MatAccumNew * accumTransMat;
				// method-1: rigorous adding
				Eigen::MatrixXd MatAccumNewDb = MatAccumNew.cast<double>();
				Eigen::MatrixXd MatR = MatAccumNewDb.block(0, 0, 3, 3);
				SSmat(0, 1) = -1 * MatAccumNewDb(2, 3);	SSmat(0, 2) = 1 * MatAccumNewDb(1, 3);
				SSmat(1, 0) = 1 * MatAccumNewDb(2, 3);	SSmat(1, 2) = -1 * MatAccumNewDb(0, 3);
				SSmat(2, 0) = -1 * MatAccumNewDb(1, 3);	SSmat(2, 1) = 1 * MatAccumNewDb(0, 3);
				Eigen::MatrixXd MattxR = SSmat * MatR;
				Adjoint.block(0, 0, 3, 3) = MatR;
				Adjoint.block(3, 3, 3, 3) = MatR;
				Adjoint.block(3, 0, 3, 3) = MattxR;
				accumVCM = allVCM[IdxTargCurrt - 1] + Adjoint * accumVCM * Adjoint.transpose();

				// method-2: simple adding
				//accumVCM = allVCM[IdxTargCurrt-1] + accumVCM;
				////--------------------------------

				RegTimes++;
			}
			cout << "Source cloud- " << i + 1 << " is aligned " << RegTimes << " time(s)\n";			
		}
		// (pairMode >= 0
		else { // directly to reference epoch
			if ((pairMode == 0) || (i < pairMode)) {
				accumTransMat = allTransMat[i];
				accumVCM = allVCM[i];
			}
			else {	// fixed interval
				for (int j = 0; j < epochNum; j++) {
					accumTransMat = allTransMat[i - pairMode * j] * accumTransMat;
					accumVCM = allVCM[i - pairMode * j] + accumVCM;
					if (i - pairMode * j < pairMode)
						break;
				}
			}
		}
		allTransMat2Ref.push_back(accumTransMat);
		allVCM2Ref.push_back(accumVCM);


		// Output TransMat and VCM to file
		outTMFile << fixed << setprecision(12);
		outTMFile << timeStamp[i] << "\n";
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 4; c++) {
				outTMFile << accumTransMat(r, c) << " ";
			}
			outTMFile << endl;
		}
		for (int r = 0; r < 6; r++) {
			for (int c = 0; c < 6; c++) {
				outTMFile << accumVCM(r, c) << " ";
			}
			outTMFile << endl;
		}

		// Output TransPara to file
		outTPFile << fixed << setprecision(10);
		outTPFile << timeStamp[i] << " ";
		Eigen::Vector3f ANGLE_TMfinal;
		Eigen::Vector3f TRANS_TMfinal;
		matrix2angle(accumTransMat, ANGLE_TMfinal);
		TRANS_TMfinal << accumTransMat(0, 3), accumTransMat(1, 3), accumTransMat(2, 3);
		// Rotation Angle (unit: gon)
		float Rx = ANGLE_TMfinal[0] * ARC_TO_GON;
		float Ry = ANGLE_TMfinal[1] * ARC_TO_GON;
		float Rz = ANGLE_TMfinal[2] * ARC_TO_GON;
		// Translation (unit: m)
		float tx = TRANS_TMfinal[0];
		float ty = TRANS_TMfinal[1];
		float tz = TRANS_TMfinal[2];
		outTPFile << Rx << " " << Ry << " " << Rz << " " << tx << " " << ty << " " << tz << " "
			<< 1000 * sqrt(accumVCM(0, 0)) * ARC_TO_GON << " " // mgon
			<< 1000 * sqrt(accumVCM(1, 1)) * ARC_TO_GON << " " // mgon
			<< 1000 * sqrt(accumVCM(2, 2)) * ARC_TO_GON << " " // mgon
			<< 1000 * sqrt(accumVCM(3, 3)) << " "	// mm
			<< 1000 * sqrt(accumVCM(4, 4)) << " "	// mm
			<< 1000 * sqrt(accumVCM(5, 5)) << endl;	// mm
	}
	outTMFile.close();
	outTPFile.close();
}



void calAbsErrorOfTransPara(std::string transMatFile, std::string GTtransMatFile, int allEpochNum,
	int startEpoch, std::string transParaErrorFile)
{
	int EpoNum = allEpochNum - startEpoch - 1;
	// Read calculated transMat
	ifstream infile1;
	infile1.open(transMatFile, ios::in);
	if (!infile1) {
		std::cerr << "Error: Cannot open transMatFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	std::vector<Eigen::Matrix4f> AllTransMat;
	for (int i = 0; i < EpoNum; i++) { 
		int Timetemp;
		infile1 >> Timetemp;
		Eigen::Matrix4f Mattemp = Eigen::Matrix4f::Identity();
		Eigen::MatrixXd VCMtemp = Eigen::MatrixXd::Zero(6, 6);
		for (int r1 = 0; r1 < 4; ++r1) {
			for (int c1 = 0; c1 < 4; ++c1)
				infile1 >> Mattemp(r1, c1);
		}
		for (int r2 = 0; r2 < 6; ++r2) {
			for (int c2 = 0; c2 < 6; ++c2)
				infile1 >> VCMtemp(r2, c2);
		}
		AllTransMat.push_back(Mattemp);
	}
	infile1.close();

	// Read reference transMat (ground truth)
	ifstream infile2;
	infile2.open(GTtransMatFile, ios::in);
	if (!infile2) {
		std::cerr << "Error: Cannot open GTtransMatFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	std::vector<Eigen::Matrix4f> AllTransMat_ref;
	for (int i = 0; i < allEpochNum; i++) {
		int Timetemp;
		infile2 >> Timetemp;
		Eigen::Matrix4f Mattemp = Eigen::Matrix4f::Identity();
		for (int r1 = 0; r1 < 4; ++r1) {
			for (int c1 = 0; c1 < 4; ++c1)
				infile2 >> Mattemp(r1, c1);
		}
		AllTransMat_ref.push_back(Mattemp);
	}
	infile2.close();


	// Calculate absolute error of transPara
	ofstream outfile(transParaErrorFile, ios::out);
	if (!outfile) {
		std::cerr << "Error: Cannot open transParaErrorFile! Aborting.\n";
		std::exit(EXIT_FAILURE);
	}
	outfile << "Err_Rx[mgon]  Err_Ry[mgon]  Err_Rz[mgon]  Err_tx[mm]  Err_ty[mm]  Err_tz[mm]" << endl;


	for (int i = 0; i < EpoNum; i++) {
		Eigen::Matrix4f transMat_final = AllTransMat[i];
		Eigen::Vector3f ANGLE_TMfinal;
		Eigen::Vector3f TRANS_TMfinal;
		matrix2angle(transMat_final, ANGLE_TMfinal);
		TRANS_TMfinal << transMat_final(0, 3), transMat_final(1, 3), transMat_final(2, 3);
		float Rx = ANGLE_TMfinal[0] * ARC_TO_GON; // Rotation (unit: gon)
		float Ry = ANGLE_TMfinal[1] * ARC_TO_GON;
		float Rz = ANGLE_TMfinal[2] * ARC_TO_GON;
		float tx = TRANS_TMfinal[0];  // Translation (unit: m)
		float ty = TRANS_TMfinal[1];
		float tz = TRANS_TMfinal[2];

		Eigen::Matrix4f transMat_ref = AllTransMat_ref[startEpoch+1 +i];
		Eigen::Vector3f ANGLE_TMref;
		Eigen::Vector3f TRANS_TMref;
		matrix2angle(transMat_ref, ANGLE_TMref);
		TRANS_TMref << transMat_ref(0, 3), transMat_ref(1, 3), transMat_ref(2, 3);
		float Rx_ref = ANGLE_TMref[0] * ARC_TO_GON; // Rotation (unit: gon)
		float Ry_ref = ANGLE_TMref[1] * ARC_TO_GON;
		float Rz_ref = ANGLE_TMref[2] * ARC_TO_GON;
		float tx_ref = TRANS_TMref[0];  // Translation (unit: m)
		float ty_ref = TRANS_TMref[1];
		float tz_ref = TRANS_TMref[2];			
	
		float errRx = 1000 * fabs(Rx_ref - Rx); // (unit: mgom)
		float errRy = 1000 * fabs(Ry_ref - Ry);
		float errRz = 1000 * fabs(Rz_ref - Rz);
		float errtx = 1000 * fabs(tx_ref - tx); // (unit: mm)
		float errty = 1000 * fabs(ty_ref - ty);
		float errtz = 1000 * fabs(tz_ref - tz);
		outfile << errRx <<" " << errRy << " " << errRz << " " 
			<< errtx << " " << errty << " " << errtz << " " << endl;
	}
	outfile.close();
}



Eigen::Matrix4f P2PICPwithPatchNormal(
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudTarget, pcl::PointCloud<pcl::PointNormal>::Ptr cloudSource, double EucldEpsilon)
{
	Eigen::Matrix4f TransMatrix = Eigen::Matrix4f::Identity();
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> p2p_icp;
	p2p_icp.setInputTarget(cloudTarget);
	p2p_icp.setInputSource(cloudSource);
	p2p_icp.setTransformationEpsilon(1e-8);
	p2p_icp.setEuclideanFitnessEpsilon(EucldEpsilon);
	p2p_icp.setMaximumIterations(100);
	pcl::PointCloud<pcl::PointNormal>::Ptr p2picp_cloud(new pcl::PointCloud<pcl::PointNormal>);
	p2p_icp.align(*p2picp_cloud);
	TransMatrix = p2p_icp.getFinalTransformation();
	return TransMatrix;
}



Eigen::MatrixXd calTransParaVCM(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget, pcl::PointCloud<pcl::PointNormal>::Ptr cloudTargetwithNormals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSourceStable)
{
	int N1 = cloudTarget->size();
	int N2 = cloudSourceStable->size();	 ///< number of correspondences
	Eigen::MatrixXd var_A(N2, 6);	///< coefficient matrix
	Eigen::VectorXd Ac1(N2);
	Eigen::VectorXd Ac2(N2);
	Eigen::VectorXd Ac3(N2);
	Eigen::VectorXd Ac4(N2);
	Eigen::VectorXd Ac5(N2);
	Eigen::VectorXd Ac6(N2);
	Eigen::VectorXd var_X(6);		///< parameter vector (incl. Rx, Ry, Rz, tx, ty, tz)
	Eigen::VectorXd var_L(N2);		///< observation vector
	Eigen::VectorXd var_V(N2);		///< improvement vector
	Eigen::MatrixXd Q_xx(6, 6);		///< Cofactor matrix
	Eigen::MatrixXd D_xx(6, 6);		///< VCM

	// Find nearest correspondences
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core2;
	core2.setInputTarget(cloudTarget);
	core2.setInputSource(cloudSourceStable);
	boost::shared_ptr<pcl::Correspondences> cor2(new pcl::Correspondences);
	core2.determineCorrespondences(*cor2, DBL_MAX);

	// Construct coefficient matrix A and observation vector L
	for (int i = 0; i < N2; ++i) {
		int j = cor2->at(i).index_match;
		double Qx = cloudSourceStable->points[i].x;
		double Qy = cloudSourceStable->points[i].y;
		double Qz = cloudSourceStable->points[i].z;
		double Px = cloudTarget->points[j].x;
		double Py = cloudTarget->points[j].y;
		double Pz = cloudTarget->points[j].z;
		double Nx = cloudTargetwithNormals->points[j].normal_x;
		double Ny = cloudTargetwithNormals->points[j].normal_y;
		double Nz = cloudTargetwithNormals->points[j].normal_z;
		Ac1(i) = Nz * Qy - Ny * Qz;
		Ac2(i) = Nx * Qz - Nz * Qx;
		Ac3(i) = Ny * Qx - Nx * Qy;
		Ac4(i) = Nx;
		Ac5(i) = Ny;
		Ac6(i) = Nz;
		var_L(i) = Nx * (Px - Qx) + Ny * (Py - Qy) + Nz * (Pz - Qz);
	}
	var_A << Ac1, Ac2, Ac3, Ac4, Ac5, Ac6;

	// Is the matrix invertible?
	Eigen::MatrixXd var_ATA(6, 6);
	var_ATA = var_A.transpose() * var_A;
	if (abs(var_ATA.determinant()) < 1e-9)
		cout << "\n This is a singular matrix! \n" << endl;

	// Least Square GM-model:
	Q_xx = var_ATA.inverse();
	var_X = Q_xx * var_A.transpose() * var_L;
	var_V = var_A * var_X - var_L;

	// calculate VCM
	double vtpv = var_V.transpose() * var_V;
	double STD0 = 1 * sqrt(vtpv / double(N2 - 6));	///< posterior standard deviation [m]
	D_xx = STD0 * STD0 * Q_xx;

	//Eigen::Matrix4f transMat = Eigen::Matrix4f::Identity();
	//transMat(0, 1) = -1 * var_X(2);	transMat(0, 2) = var_X(1);		transMat(0, 3) = var_X(3);
	//transMat(1, 0) = var_X(2);		transMat(1, 2) = -1 * var_X(0);	transMat(1, 3) = var_X(4);
	//transMat(2, 0) = -1 * var_X(1);	transMat(2, 1) = var_X(0);		transMat(2, 3) = var_X(5);

	return D_xx;
}

