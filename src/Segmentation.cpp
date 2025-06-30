/**
 * @file Segmentation.cpp
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @date 2025-06
 */

#include "Segmentation.h"
using namespace std;


int PatchGenerationAndRefinement(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float svResolution,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentroid,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBoundary,
	pcl::PointCloud<pcl::PointXYZ>* &cloudPatches, bool isVis)
{
	// Importing points from point clouds
	cl::Array<cl::RPoint3D> points;
	points.clear();
	for (int i = 0; i < cloud->size(); ++i) {
		points.emplace_back(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}
	int numPoints = points.size();

	pcl::console::TicToc time;
	cout << "--->>> Compute point normals... ";  time.tic();
	
	
	// Compute the k-nearest neighbors for each point, and estimate the normals
	cl::KDTree<cl::RPoint3D> kdtree;
	kdtree.SwapPoints(&points);
	assert(kNN < numPoints);
	
	cl::Array<cl::RVector3D> normals(numPoints);
	cl::Array<cl::Array<int>> neighbors(numPoints);
	cl::Array<cl::RPoint3D> neighbor_points(kNN);
	for (int i = 0; i < numPoints; ++i) {
		kdtree.FindKNearestNeighbors(kdtree.points()[i], kNN, &neighbors[i]);
		for (int k = 0; k < kNN; ++k) {
			neighbor_points[k] = kdtree.points()[neighbors[i][k]];
		}
		cl::geometry::point_cloud::PCAEstimateNormal(neighbor_points.begin(),
			neighbor_points.end(),
			&normals[i]);
	}
	kdtree.SwapPoints(&points);
	cout << "Computing time: " << int(0.001*time.toc()) << " s " << endl;


	cout << "--->>> Start supervoxel-based segmentation... ";  time.tic();
	VCCSMetric metric(svResolution);
	int numSV;						///< Number of generated SV
	cl::Array<int> lin_supervoxels;	///< the indices of representative points of SV (size = SV number)
	cl::Array<int> lin_labels;		///< the indices of SV for all points (size = PC point number)

	cl::Array<PointWithNormal> oriented_points(numPoints);
	for (int i = 0; i < numPoints; ++i) {
		oriented_points[i].x = points[i].x;
		oriented_points[i].y = points[i].y;
		oriented_points[i].z = points[i].z;
		oriented_points[i].normal = normals[i];
	}
	cl::geometry::point_cloud::SupervoxelSegmentation(
		oriented_points, neighbors,
		svResolution, metric,
		&lin_supervoxels,	///< the indices of representative points
		&lin_labels);		///< the index of supervoxel for each point
	numSV = lin_supervoxels.size();
	cout << "Computing time: " << int(0.001*time.toc()) << " s " << endl;
	cout << "--->>> " << numSV << " supervoxels are generated." << endl;


	cout << "--->>> Extract and refine patches... ";  time.tic();
	// Exporting points
	pcl::PointCloud<pcl::PointXYZ> allPoints;
	allPoints.clear();
	allPoints.resize(numPoints);
	for (int i = 0; i < numPoints; ++i) {
		allPoints.points[i].x = points[i].x;
		allPoints.points[i].y = points[i].y;
		allPoints.points[i].z = points[i].z;
	}
	// Creat point cloud array to restore all SV 
	cloudPatches = new pcl::PointCloud<pcl::PointXYZ>[numSV];
	// Initializing centroids and boundary points variables
	cloudCentroid->clear(); cloudBoundary->clear();
	// Number of valid SV
	int validSV = 0;
	// Number of invalid SV
	int invalidSV = 0;
	// Point number of all valid SV
	int validSVPtNum = 0;
	// Point cloud of an valid SV
	pcl::PointCloud<pcl::PointXYZ>::Ptr SVcloudtemp_effe(new pcl::PointCloud<pcl::PointXYZ>);


	// Extract SV from point cloud according to the labels of points 
	pcl::PointCloud<pcl::PointXYZ> * SVcloud_all = new pcl::PointCloud<pcl::PointXYZ>[numSV];
	for (auto iter = lin_labels.begin(); iter != lin_labels.end(); iter++) {
		int SVIdx = *iter;
		int PtIdx = distance(lin_labels.begin(), iter);
		SVcloud_all[SVIdx].push_back(allPoints.points[PtIdx]);
	}


	// Extract valid supervoxels (SV), and compute their centroids (CT) and boundary points (BP)
	for (int i = 0; i < numSV; ++i) {
		// filter out SV with insufficient number of points
		if (SVcloud_all[i].size() < minPtNum) {
			++invalidSV;
			continue;
		}
		SVcloudtemp_effe->clear();

		// Refine initially genrated patches by removing points with large residuals (distance > 2σ)
		int effeSVpointnum = PatchRefinement(SVcloud_all[i].makeShared(), SVcloudtemp_effe, 2.0);
		
		// Filtering again after refinement
		if (effeSVpointnum < minPtNum) {
			++invalidSV;
			continue;
		}
		
		// Patch selection based on curvature and planarity metrics
		float variation, planarity, linearity;
		calPatchFeature(SVcloudtemp_effe, variation, planarity, linearity);
		if (variation > 0.02f || planarity < 0.25f) {  // Optionally add: || linearity > 0.8
			//cout << "SV - " << i << "  --- is not planar enough!" << endl;
			++invalidSV;
			continue;
		}
		
		// Store the refined patch
		cloudPatches[validSV] = *SVcloudtemp_effe;
		validSVPtNum = validSVPtNum + cloudPatches[validSV].size();

		// Compute the centroid and boundary points for the current patch
		pcl::PointXYZ centroid;
		pcl::PointCloud<pcl::PointXYZ>::Ptr BP_oneSV(new pcl::PointCloud<pcl::PointXYZ>);
		int BPnumtemp = calPatchCTandBP(*SVcloudtemp_effe, centroid, BP_oneSV);
		
		if (BPnumtemp != 6) {
			std::cerr << "Error: Incorrect number of boundary points calculated! Aborting.\n";
			std::exit(EXIT_FAILURE);
		}
		cloudCentroid->push_back(centroid);
		*cloudBoundary += *BP_oneSV;

		++validSV;
	}
	cout << "Computing time: " << int(0.001*time.toc()) << " s " << endl;
	delete[] SVcloud_all;
	SVcloud_all = NULL;


	//----------------------------------------------------------------------------------------------------------
	cout << "--->>> Number of selected patches = " << validSV
		<< "   Ratio of selected patches = " << 100.0 * validSV / numSV << "% \n"
		<< "--->>> Number of points in selected patches = " << validSVPtNum
		<< "   Ratio of selected points = " << 100.0 * validSVPtNum / cloud->size() << "% \n\n";


	// Visualization of generated patches
	if (isVis) {
		pcl::visualization::PCLVisualizer viewer("Patch visualization");
		viewer.setBackgroundColor(1, 1, 1);
		pcl::PointCloud<pcl::PointXYZRGB> coloredPatches;
		coloredPatches.clear();
		for (int i = 0; i < validSV; i++) {
			double cR = rand() / (RAND_MAX + 1.0f);
			double cG = rand() / (RAND_MAX + 1.0f);
			double cB = rand() / (RAND_MAX + 1.0f);
			pcl::PointCloud<pcl::PointXYZRGB> coloredPatchtemp;
			pcl::copyPointCloud(cloudPatches[i], coloredPatchtemp);
			for (int j = 0; j < coloredPatchtemp.size(); j++) {
				coloredPatchtemp.points[j].r = 250 * cR;
				coloredPatchtemp.points[j].g = 250 * cG;
				coloredPatchtemp.points[j].b = 250 * cB;
			}
			coloredPatches += coloredPatchtemp;
		}
		viewer.addPointCloud<pcl::PointXYZRGB>(coloredPatches.makeShared(), "coloredSVcloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "coloredSVcloud");
		viewer.addPointCloud<pcl::PointXYZ>(cloudCentroid, "CentroidCloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "CentroidCloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "CentroidCloud");
		viewer.setSize(1200, 800);
		while (!viewer.wasStopped())
			viewer.spinOnce(100);
	}
	return validSV;
}


int PatchRefinement(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr refinedPatch, double sigmaMul)
{
	int pointNum = cloud->size();
	refinedPatch->clear();

	// Estimate patch centroid and normal using PCA
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	Eigen::RowVector3f centroid = pca.getMean().head<3>();
	Eigen::RowVector3f normal = pca.getEigenVectors().col(2);
	float coefA = normal(0);
	float coefB = normal(1);
	float coefC = normal(2);
	float coefD = -normal.dot(centroid);

	// Calculate standard deviation of point-to-plane distances
	double stdDist = 0.0;
	std::vector<double> Po2PlDist;
	for (int i = 0; i < pointNum; ++i) {
		double dist = pcl::pointToPlaneDistance(
			cloud->points[i], coefA, coefB, coefC, coefD);
		Po2PlDist.push_back(dist);
		stdDist += dist * dist;
	}
	stdDist = std::sqrt(stdDist / double(pointNum));

	// Filter out points with large residuals (> sigmaMul * STD)
	for (int j = 0; j < pointNum; ++j) {
		if (std::fabs(Po2PlDist[j]) < std::fabs(sigmaMul * stdDist))
			refinedPatch->push_back(cloud->points[j]);
	}
	return refinedPatch->size();
}


void calPatchFeature(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &variation, float &planarity, float &linearity)
{
	int pointNum = cloud->size();
	Eigen::MatrixXf cloud_mat(pointNum, 3);
	for (size_t i = 0; i < pointNum; ++i) {
		cloud_mat.row(i) = cloud->points[i].getVector3fMap();
	}
	// calculate centroid
	Eigen::RowVector3f meanVector = cloud_mat.colwise().mean();
	// reduction to centroid
	cloud_mat.rowwise() -= meanVector;
	// calculate covariance matrix
	Eigen::Matrix3f covMat;
	covMat = (cloud_mat.transpose() * cloud_mat) / pointNum;
	// SVD
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f V = svd.matrixV();
	Eigen::Vector3f S = svd.singularValues();
	// E1 > E2 > E3
	float E1 = S(0);
	float E2 = S(1);
	float E3 = S(2);
	variation = E3 / (E1 + E2 + E3);	// surface variation or curvature
	planarity = (E2 - E3) / E1;			// planarity
	linearity = (E1 - E2) / E1;			// linearity
}


int calPatchCTandBP(pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointXYZ &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBP)
{
	cloudBP->clear();
	
	// Compute the centroid of SV
	Eigen::Vector4f CTtemp;
	pcl::compute3DCentroid(cloud, CTtemp);
	centroid.x = CTtemp[0];
	centroid.y = CTtemp[1];
	centroid.z = CTtemp[2];

	// Initialize boundary points (6 max and min XYZ)
	pcl::PointXYZ BPXmax(-DBL_MAX, 0.0f, 0.0f);
	pcl::PointXYZ BPXmin(DBL_MAX, 0.0f, 0.0f);
	pcl::PointXYZ BPYmax(0.0f, -DBL_MAX, 0.0f);
	pcl::PointXYZ BPYmin(0.0f, DBL_MAX, 0.0f);
	pcl::PointXYZ BPZmax(0.0f, 0.0f, -DBL_MAX);
	pcl::PointXYZ BPZmin(0.0f, 0.0f, DBL_MAX);

	// Find extremal points
	for (int ib = 0; ib < cloud.size(); ++ib) {
		if (cloud.points[ib].x > BPXmax.x)
			BPXmax = cloud.points[ib];
		if (cloud.points[ib].x < BPXmin.x)
			BPXmin = cloud.points[ib];
		if (cloud.points[ib].y > BPYmax.y)
			BPYmax = cloud.points[ib];
		if (cloud.points[ib].y < BPYmin.y)
			BPYmin = cloud.points[ib];
		if (cloud.points[ib].z > BPZmax.z)
			BPZmax = cloud.points[ib];
		if (cloud.points[ib].z < BPZmin.z)
			BPZmin = cloud.points[ib];
	}
	// Store boundary points
	cloudBP->push_back(BPXmax);
	cloudBP->push_back(BPXmin);
	cloudBP->push_back(BPYmax);
	cloudBP->push_back(BPYmin);
	cloudBP->push_back(BPZmax);
	cloudBP->push_back(BPZmin);
	
	return cloudBP->size(); ///< should always return 6
}


void calBPandCTSTD(
	pcl::PointCloud<pcl::PointXYZ>* cloudPatches, int patchNum, std::vector<float>& stdBP, std::vector<float>& stdCT)
{
	stdBP.clear();
	stdCT.clear();
	// float EmpCorr = 0.0f;  ///< Empirical correlation coefficient within each patch

	for (int i = 0; i < patchNum; ++i) {
		int pointNum = cloudPatches[i].size();
		float patchStd = calPatchSTD(cloudPatches[i].makeShared());
		stdBP.push_back(patchStd);
		float Neffe = float(pointNum); // without considering correlation
		//float Neffe = pointNum / (1.0f + EmpCorr * (pointNum - 1.0f));  // considering correlation
		stdCT.push_back(patchStd / Neffe);
	}
}

