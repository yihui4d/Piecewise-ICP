/**
 * @file Registration.h
 * @brief Public API & core functions for Piecewise‑ICP.
 * @details Declaration of external interfaces for 4D point cloud registration (PiecewiseICP_4D_call) 
 *          and pairwise registration (PiecewiseICP_pair_call), as well as several internal key functions.
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @version 1.0
 * @date 2025-06-06
 * @license Apache-2.0
 * @copyright 2025 Yihui Yang, Technical University of Munich. All rights reserved.
 */

#pragma once

#include "CommonFunc.h"
#include "Segmentation.h"


/**
 * @brief Entry point for Piecewise-ICP (4D point cloud registration). Public API.
 *
 * This function performs multi-epoch point cloud (4DPC) registration, where each scan is aligned to the reference epoch based on a temporal sequence.
 * Registration parameters and file paths are provided in a configuration file.
 *
 * @param[in] confile      Full path to the configuration file (TXT format).
 * @param[in] startEpoch   Index of the reference epoch in the scan file list (usually 0 as the first scan).
 * @param[in] epochNum     Total number of scans to process.
 * @param[in] pairMode     Mode of pair sequence determination:
 *                         = 0: All scans are directly registered to the reference scan
 *                         > 0: Each scan is registered to the previous scan with a fix interval
 *                         < 0: Each scan is registered to the previous scan with adaptive intervals
 * @param[in] overlapThd   Overlap ratio threshold for determining adaptive registration pairs (default: 75%)
 *
 * @return true if the registration process completes successfully; false otherwise.
 */
bool PiecewiseICP_4D_call(const char* confile, int startEpoch, int epochNum, int pairMode, float overlapThd = 0.75f);


/**
 * @brief Entry point for Piecewise-ICP (Pairwise registration only). Public API.
 *
 * This function performs pairwise point cloud registration based on parameters provided in a configuration file.
 *
 * @param[in] confile   Full path to the configuration file (TXT format).
 * @param[in] outfile   Prefix used for naming all output result files.
 *
 * @return true if the registration completes successfully; false otherwise.
 */
bool PiecewiseICP_pair_call(const char* confile, const char* outfile);




/**
 * @brief Perform Piecewise-ICP to register point cloud time series
 *
 * @param[in] cloud1         Target point cloud (reference).
 * @param[in] cloud2         Source point cloud to be registered.
 * @param[in] isSetResSVsize Whether to manually set the point spacing and initial supervoxel size.
 * @param[in] Res1           Average point spacing (resolution) of cloud1.
 * @param[in] Res2           Average point spacing (resolution) of cloud2.
 * @param[in] SVsize1        Supervoxel size for cloud1 (used if isSetResSVsize is true).
 * @param[in] SVsize2        Supervoxel size for cloud2 (used if isSetResSVsize is true).
 * @param[in] isManualDTinit Whether to manually set the initial distance threshold.
 * @param[in] DTinit         Initial distance threshold (used if isManualDTinit is true).
 * @param[in] DTmin          Minimum distance threshold (Level of Detection).
 * @param[in] outfileIdx     Prefix for output file names.
 * @param[out] transMat      Computed 4×4 transformation matrix from source to target.
 * @param[out] transPara     Transformation parameters (e.g., x, y, z, roll, pitch, yaw).
 * @param[out] VCM           6×6 variance-covariance matrix of the estimated parameters.
 *
 * @return true if the registration completes successfully; false otherwise.
 */
bool Piecewise_ICP_4D(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	bool isSetResSVsize, float Res1, float Res2, float SVsize1, float SVsize2,
	bool isManualDTinit, float DTinit, float DTmin, std::string outfileIdx, 
	Eigen::Matrix4f &transMat, std::vector<float> &transPara, Eigen::MatrixXd &VCM);


/**
 * @brief Determine the optimal pair sequence for 4D point clouds using adaptive interval.
 *
 * @param[in] fileNameList      List of full path of all scan files.
 * @param[in] startEpoch        Index of the reference epoch in the scan file list (usually 0 as the first scan).
 * @param[in] DTinit            Initial distance threshold.
 * @param[in] ratioThd          Threshold of the overlapping ratio to be taken as a pair.
 * @param[out] RegPair          Determined registration pair indexes for all scans.
 * @param[in] adaptivePairFile  File path for storing determined pair sequence.
 *
 * @return true if the pair determination completes successfully; false otherwise.
*/
bool calAdaptivePairSequence(std::vector<std::string> fileNameList, int startEpoch,
	float DTinit, float ratioThd, std::map<int, int> &RegPairs, std::string adaptivePairFile);


/**
 * @brief Calculate the overlapping ratio between two point clouds by C2C distance.
 *
 * @param[in] cloud1    Point cloud 1.
 * @param[in] cloud2    Point cloud 2.
 * @param[in] DTinit    Initial distance threshold (points with distances under this value are regarded as overlaps).
 *
 * @return Overlapping ratio.
*/
float calOverlapRatioByC2Cdist(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, 
	float DTinit);


/**
 * @brief Calculate final transformations of each scan to the reference scan based on intermediate
 *        pairwise transformation matrices and determined pair sequence.
 *
 * @param[in] transMatFile       Full path to the file containing computed pairwise 4x4 transformation matrices.
 * @param[in] pairMode           Mode of pair sequence determination:
 *                               = 0: All scans are directly registered to the reference scan;
 *                               > 0: Each scan is registered to the previous scan with a fix interval;
 *                               < 0: Each scan is registered to the previous scan with adaptive intervals.
 * @param[in] adaptivePairFile   Full path to the file containing determined adaptive pair sequence (only used if pairMode < 0).
 * @param[in] epochNum           Number of scans to be registered.
 * @param[in] transMat2RefFile   Full path to the output file storing the final 4x4 transformation matrices.
 * @param[in] transPara2RefFile  Full path to the output file storing the final 6 transformation parameters.
 * @param[out] timeStamp         Output vector storing the timestamp (index) of each registered scan.
 * @param[out] allTransMat2Ref   Output vector of final 4x4 transformation matrices to the reference scan.
 * @param[out] allVCM2Ref        Output vector of variance-covariance matrices of the final transformation parameters.
 */
void calTransToReferenceEpoch(std::string transMatFile, int pairMode, std::string adaptivePairFile, int epochNum,
	std::string transMat2RefFile, std::string transPara2RefFile,
	std::vector<int> &timeStamp, std::vector<Eigen::Matrix4f> &allTransMat2Ref, std::vector<Eigen::MatrixXd> &allVCM2Ref);


/**
 * @brief Perform Piecewise-ICP to register two point clouds.
 *
 * @param[in] cloud1         Target point cloud (reference).
 * @param[in] cloud2         Source point cloud to be registered.
 * @param[in] isSetResSVsize Whether to manually set the point spacing and initial supervoxel size.
 * @param[in] Res1           Average point spacing (resolution) of cloud1.
 * @param[in] Res2           Average point spacing (resolution) of cloud2.
 * @param[in] SVsize1        Supervoxel size for cloud1 (used if isSetResSVsize is true).
 * @param[in] SVsize2        Supervoxel size for cloud2 (used if isSetResSVsize is true).
 * @param[in] isManualDTinit Whether to manually set the initial distance threshold.
 * @param[in] DTinit         Initial distance threshold (used if isManualDTinit is true).
 * @param[in] DTmin          Minimum distance threshold (Level of Detection).
 * @param[out] DTseries		 Updated distance thresholds after each iteration.
 * @param[out] transMat      Computed 4×4 transformation matrix from source to target.
 * @param[out] VCM           6×6 variance-covariance matrix of the estimated parameters.
 */
void Piecewise_ICP(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	bool isSetResSVsize, float Res1, float Res2, float SVsize1, float SVsize2,
	bool isManualDTinit, float DTinit, float DTmin,
	std::vector<float> &DTseries, Eigen::Matrix4f &transMat, Eigen::MatrixXd &VCM);


/**
 * @brief Perform pairwise registration for each iteration.
 *
 * @param[in] cloud1         Target point cloud (reference).
 * @param[in/out] cloud2     Source point cloud to be registered.
 * @param[in] Res1           Average point spacing (resolution) of cloud1.
 * @param[in] Res2           Average point spacing (resolution) of cloud2.
 * @param[in] SVRes1         Patch size for cloud1.
 * @param[in] SVRes2         Patch size for cloud2.
 * @param[in] SVCloud1       Point cloud array storing all patches in cloud1.
 * @param[in] SVCloud2       Point cloud array storing all patches in cloud2.
 * @param[in] CTcloud1       Point cloud storing the centroids of all patches in cloud1.
 * @param[in] CTcloud2       Point cloud storing the centroids of all patches in cloud2.
 * @param[in] BPcloud1       Point cloud storing the boundary points of all patches in cloud1.
 * @param[in] BPcloud2       Point cloud storing the boundary points of all patches in cloud2.
 * @param[in] CTstd1         Standard deviations of the centroid of each patch in cloud1.
 * @param[in] BPstd2         Standard deviations of boundary points of each patch in cloud2.
 * @param[in] DTmin          Minimum distance threshold (Level of Detection).
 * @param[out] currDT        The distance threshold in the current iteration
 * @param[out] BBchange_1    Change in bounding box corner .
 * @param[out] BBchange_2    Change in bounding box corner for cloud2.
 * @param[out] VCM           6×6 variance-covariance matrix of the estimated parameters.
 *
 * @return 4×4 transformation matrix aligning cloud2 to cloud1 for the current iteration.
 */
Eigen::Matrix4f PwICP_singleIteration(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	float Res1, float Res2, float SVRes1, float SVRes2,
	pcl::PointCloud<pcl::PointXYZ>* &SVcloud1, pcl::PointCloud<pcl::PointXYZ>* &SVcloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr CTcloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr BPcloud2,
	std::vector<float> CTstd1, std::vector<float> BPstd2, float DTmin,
	float &currDT, float &BBchange_1, float &BBchange_2, Eigen::MatrixXd &VCM);


/**
 * @brief Calculate absolute errors of transformation parameters with respect to ground truth.
 *
 * @param[in] transMatFile         Full path to the file containing estimated finally 4x4 transformation matrices.
 * @param[in] GTtransMatFile       Full path to the file containing ground-truth 4x4 transformation matrices.
 * @param[in] allEpochNum          Total number of epochs (scans) to be evaluated.
 * @param[in] startEpoch           Index of the reference scan (epoch) in the scan file list (usually 0 as the first scan).
 * @param[in] transParaErrorFile   Full path to the output file storing absolute errors (3 rotations and 3 translation components).
 */
void calAbsErrorOfTransPara(std::string transMatFile, std::string GTtransMatFile, int allEpochNum,
	int startEpoch, std::string transParaErrorFile);


/**
 * @brief Perform point-to-plane ICP using patch centroids and normals.
 *
 * @param[in] cloudTarget   Target point cloud with normals.
 * @param[in] cloudSource   Source point cloud with normals.
 * @param[in] EucldEpsilon  Convergence threshold for Euclidean distance between correspondces.
 * 
 * @return Computed 4×4 transformation matrix aligning the source to the target.
 */
Eigen::Matrix4f P2PICPwithPatchNormal(
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudTarget, pcl::PointCloud<pcl::PointNormal>::Ptr cloudSource, double EucldEpsilon);


/**
 * @brief Calculate the variance-covariance matrix (VCM) of six transformation parameters 
 *        estimated by linear point-to-plane ICP using identified stable areas.
 *
 * @param[in] cloudTarget             Target point cloud (coordinates only).
 * @param[in] cloudTargetwithNormals  Target point cloud with associated patch normals.
 * @param[in] cloudSourceStable       Source point cloud of stable areas.
 * 
 * @return 6×6 variance-covariance matrix (VCM) of the estimated transformation parameters.
 */
Eigen::MatrixXd calTransParaVCM(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget, pcl::PointCloud<pcl::PointNormal>::Ptr cloudTargetwithNormals,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSourceStable);
