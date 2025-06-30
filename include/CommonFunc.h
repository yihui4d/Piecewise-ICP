/**
 * @file CommonFunc.h
 * @brief Some commonly used functions in Piecewise-ICP, including file reading, point cloud processing, visualization, etc.
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @version 1.0
 * @date 2025-06-06
 * @license Apache-2.0
 * @copyright 2025 Yihui Yang, TU Munich. All Rights Reserved.
 */

#pragma once

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <pcl/console/time.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

const double ARC_TO_DEG = 57.29577951308238;	///< 1 rad * ARC_TO_DEG = 1 degree
const double DEG_TO_ARC = 0.0174532925199433;	///< 1 degree * DEG_TO_ARC = 1 rad
const double GON_TO_ARC = 0.0157079632679;		///< 1 gon * GON_TO_ARC = 1 rad
const double ARC_TO_GON = 63.6619772368;		///< 1 rad * ARC_TO_GON = 1 gon
const int kNN = 45;								///< Number of k-nearest neighbors for normal estimation (empirical range: 30-70)
const int minPtNum = 20;						///< Minimum number of points per patch (effective supervoxel)


/**
 * @brief Configuration parameters in the configuration file.
 */
struct ConfigPara
{
	std::string FolderFilePath1;	///< full path 1 (target PC or 4DPC folder path)
	std::string FolderFilePath2;	///< full path 2 (source PC or output folder path)
	bool isSetResSVsize;			///< whether to manually set the average point spacing and the initial size of supervoxels
	float PCres1;					///< resolution of PC1 after subsampling
	float PCres2;					///< resolution of PC2 after subsampling 
	float SVsize1;					///< size of generated SV in cloud1 (manually set)
	float SVsize2;					///< size of generated SV in cloud2 (manually set)
	bool isSetDTinit;				///< whether to manually set the initial DT
	float DTinit;					///< initial DT set by the accuracy of coarse registration (manually set)
	float DTmin;					///< minimum Level of Detection
	bool isVisual;					///< whether to visualize processed results
};


/**
 * @brief Load configuration parameters from a specified configuration file.
 * 
 * @param[in] confile     Full path to the configuration file.
 * @param[out] confPara   Output structure containing all imported configuration parameters.
 * @return true if the configuration file is successfully read; false otherwise.
 */
bool readConfigFile(std::string conFile, ConfigPara &confPara);


/**
 * @brief Extract and sort the full paths of all scan files (*.pcd) from a specified folder.
 *
 * @param[in] folderPath        Full path to the folder containing scan files.
 * @param[out] fileNameList     Extracted full paths of all scan files in ascending time order.
 * @param[out] fileTimeList     Timestamps of each file in ascending order.
 * @return                      The total number of extracted files.
 */
int extractAllFilesFromFolder(std::string folderPath, std::vector<std::string>& fileNameList, std::vector<long>& fileTimeList);


/**
 * @brief Extract the full paths of all files in a specified folder.
 *
 * @param[in] folderPath   Path to the folder.
 * @param[out] files       Vector to store the full paths of all extracted files.
 * @return                 Number of files extracted from the folder.
 */
void getFiles(std::string folderpath, std::vector<std::string>& files);


/**
 * @brief Extract a timestamp from a file name based on specified substring and timestamp length.
 *
 * @param[in] fileName     The name of the file from which to extract the timestamp.
 * @param[in] substring    The substring preceding the timestamp in the file name.
 * @param[in] timeLength   The number of characters representing the timestamp.
 * @return                 Extracted timestamp as a long integer.
 */
long extractTimeFromFileName(std::string fileName, const std::string substring, int timeLength);



/**
 * @brief Calculate the average point spacing (resolution) of a point cloud.
 *
 * This function computes the average Euclidean distance from each point to its nearest neighbor
 * across the entire input point cloud. It is often used as an estimation of point cloud resolution.
 *
 * @param[in] cloud Input point cloud.
 * @return Average nearest-neighbor distance as point cloud resolution.
 */
float calPCresolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


/**
 * @brief Calculate the percentile distance between two point clouds.
 *
 * @param[in] cloud1     Reference point cloud.
 * @param[in] cloud2     Query point cloud.
 * @param[in] percentile Specified percentile (range: 0.0–1.0).
 * @return Distance value at the specified percentile (e.g., 50% = median).
 */
double calPercentileDistBetween2PC(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, float percentile);


/**
 * @brief Calculate the normal vector of a given patch (point cloud).
 *
 * @param[in] cloud   Input patch point cloud (assumed to be nearly planar).
 * @param[out] nx     Output x-component of the estimated normal vector.
 * @param[out] ny     Output y-component of the estimated normal vector.
 * @param[out] nz     Output z-component of the estimated normal vector.
 * @return true if normal estimation is successful; false otherwise.
 */
bool calPatchNormal(
	pcl::PointCloud<pcl::PointXYZ> cloud, float &nx, float &ny, float &nz);


/**
 * @brief Calculate the standard deviation of distances from points to the best-fitting patch plane.
 *
 * @param[in] cloud  Input patch point cloud.
 * @return Standard deviation of point-to-plane distances.
 */
float calPatchSTD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


/**
 * @brief Generate a point cloud comprised of centroids with associated patch normals
 *        for subsequent point-to-plane ICP registration.
 *
 * @param[in] cloudCentroids         Input centroids (one point per patch).
 * @param[in] cloudPatch             Input array of patches corresponding to the centroids.
 * @param[out] cloudCentroids_normals Output centroid point cloud with patch normals.
 */
void generateCentroidCloudWithPatchNormals(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentroids, pcl::PointCloud<pcl::PointXYZ>* cloudPatch,
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudCentroids_normals);


/**
 * @brief Extract three rotation angles (in radians) from a 4x4 transformation matrix.
 *
 * @param[in]  transMat   Input 4x4 transformation matrix.
 * @param[out] rotAngle   Output rotation angles (in rad) around X, Y, Z axes.
 */
void matrix2angle(Eigen::Matrix4f transMat, Eigen::Vector3f &rotAngle);


/**
 * @brief Calculate the maximum change of the corners of a 3D bounding box after transformation.
 *
 * @param[in]  boundingBox  A pointer to a 6-element array containing two opposite corners:
 *                          [x_min, y_min, z_min, x_max, y_max, z_max].
 * @param[in]  transMat     A 4x4 transformation matrix.
 * @return Maximum change of corners after transformation.
 */
float calBoundingBoxCornerChange(const double* boundingBox, const Eigen::Matrix4f transMat);


/**
 * @brief Preprocess the input point cloud by optional voxel-based downsampling and statistical outlier removal (SOR).
 *
 * @param[in] cloud_in         Input raw point cloud.
 * @param[out] cloud_out       Output preprocessed point cloud.
 * @param[in] isDownSamp       Flag to indicate whether to perform voxel downsampling.
 * @param[in] voxelSize        Voxel size for downsampling (used only if isDownSamp is true).
 * @param[in] SOR_NeighborNum  Number of nearest neighbors used in the SOR filter.
 * @param[in] SOR_StdMult      Standard deviation multiplier for the SOR threshold.
 */
void PCpreprocessing(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
	bool isDownSamp, float voxelSize, int SOR_NeighborNum, double SOR_StdMult);


/**
 * @brief Perform Statistical Outlier Removal (SOR) filtering on a point cloud.
 *
 * @param[in] cloud_in         Input point cloud to be filtered.
 * @param[out] cloud_out       Output point cloud after removing outliers.
 * @param[in] SOR_NeighborNum  Number of nearest neighbors.
 * @param[in] SOR_StdMult      Standard deviation multiplier.
 */
void SORfilter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
	int SOR_NeighborNum, double SOR_StdMult);



/** @brief Visualize two point clouds with different color and size in one window
 */
void visualizeTwoPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, std::string name1, double PCr1, double PCg1, double PGb1, int size1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name2, double PCr2, double PCg2, double PGb2, int size2,
	double BGr, double BGg, double BGb);


/** @brief Visualize three point clouds with different color and size in one window
 */
void visualizeThreePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, std::string name1, double PCr1, double PCg1, double PGb1, int size1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string name2, double PCr2, double PCg2, double PGb2, int size2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, std::string name3, double PCr3, double PCg3, double PGb3, int size3,
	double BGr, double BGg, double BGb);
