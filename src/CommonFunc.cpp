/**
 * @file CommonFunc.cpp
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @date 2025-06
 */

#include "CommonFunc.h"
using namespace std;


bool readConfigFile(std::string conFile, ConfigPara &confPara)
{
	ifstream confileIN;
	confileIN.open(conFile);
	if (!confileIN || conFile.empty()) {
		std::cerr << "Cannot open configuration file! Aborting.\n";
		return false;
	}
	string strtemp;

	// 1. FolderFilePath1
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 2, strtemp.length() - 1);
		confPara.FolderFilePath1 = numtemp;
		cout << "1 FolderFilePath1: " << "*" << confPara.FolderFilePath1 << "*" << endl;
	}

	// 2. FolderFilePath2
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 2, strtemp.length() - 1);
		confPara.FolderFilePath2 = numtemp;
		cout << "2 FolderFilePath2: " << "*" << confPara.FolderFilePath2 << "*" << endl;
	}

	// 3. isSetResSVsize
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.isSetResSVsize = std::stoi(numtemp);
		cout << "3 isSetResSVsize: " << "*" << confPara.isSetResSVsize << "*" << endl;
	}

	// 4. PCres1
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.PCres1 = std::stof(numtemp);
		cout << "4 PCres1: " << "*" << confPara.PCres1 << "*" << endl;
	}
	if (confPara.PCres1 <= 0) {
		std::cerr << "PCres1 out of limits! \n" << endl;
		return false;
	}

	// 5. PCres2
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.PCres2 = std::stof(numtemp);
		cout << "5 PCres2: " << "*" << confPara.PCres2 << "*" << endl;
	}
	if (confPara.PCres2 <= 0) {
		std::cerr << "PCres2 out of limits! \n" << endl;
		return false;
	}

	// 6. SVsize1
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.SVsize1 = std::stof(numtemp);
		cout << "6 SVsize1: " << "*" << confPara.SVsize1 << "*" << endl;
	}
	if (confPara.SVsize1 < confPara.PCres1) {
		std::cerr << "SVsize1 out of limits! \n" << endl;
		return false;
	}

	// 7. SVsize2
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.SVsize2 = std::stof(numtemp);
		cout << "7 SVsize2: " << "*" << confPara.SVsize2 << "*" << endl;
	}
	if (confPara.SVsize2 < confPara.PCres2) {
		std::cerr << "SVsize2 out of limits! \n" << endl;
		return false;
	}

	// 8. isSetDTinit
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.isSetDTinit = std::stoi(numtemp);
		cout << "8 isSetDTinit: " << "*" << confPara.isSetDTinit << "*" << endl;
	}

	// 9. DTinit
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.DTinit = std::stof(numtemp);
		cout << "9 DTinit: " << "*" << confPara.DTinit << "*" << endl;
	}
	if (confPara.DTinit <= 0) {
		std::cerr << "DTinit out of limits! \n" << endl;
		return false;
	}

	// 10. DTmin
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.DTmin = std::stof(numtemp);
		cout << "10 DisThrhdmin: " << "*" << confPara.DTmin << "*" << endl;
	}
	if (confPara.DTinit < confPara.DTmin) {
		std::cerr << "DTmin out of limits! \n" << endl;
		return false;
	}

	// 11. isVisual
	getline(confileIN, strtemp);
	if (!strtemp.empty()) {
		string numtemp = strtemp.substr(strtemp.find(":") + 1, strtemp.length() - 1);
		confPara.isVisual = std::stoi(numtemp);
		cout << "11 isVisual: " << "*" << confPara.isVisual << "*" << endl;
	}

	cout << endl << endl;
	confileIN.close();
	return true;
}


/** @brief Comparator function for sorting a list of key-value pairs.
 */
int cmpMap(const pair<string, long>& x, const pair<string, long>& y) {
	return x.second < y.second; // ascending order
}

int QuickSortOnce(double a[], int low, int high)
{
	double pivot = a[low];
	int i = low, j = high;
	while (i < j) {
		while (a[j] >= pivot && i < j) {
			j--;
		}
		a[i] = a[j];
		while (a[i] <= pivot && i < j) {
			i++;
		}
		a[j] = a[i];
	}
	a[i] = pivot;
	return i;
}

void QuickSort(double a[], int low, int high)
{
	if (low >= high)
		return;
	int pivot = QuickSortOnce(a, low, high);
	QuickSort(a, low, pivot - 1);
	QuickSort(a, pivot + 1, high);
}

/** @brief Calculate the element at a given percentile of an array.
 */
double calArrayPercentileElement(double a[], int n, float percentile)
{
	QuickSort(a, 0, n - 1);
	int leftnum = n * percentile;
	return a[leftnum];
}


int extractAllFilesFromFolder(std::string folderPath, std::vector<std::string>& fileNameList, std::vector<long>& fileTimeList)
{
	fileNameList.clear();  fileTimeList.clear();
	vector<string> allfilenames;
	getFiles(folderPath, allfilenames);
	cout << "--->>> " << allfilenames.size() << " files found in folder: " << folderPath << "\n";

	vector<long> allfiletime;
	for (int j = 0; j < allfilenames.size(); j++) {
		allfiletime.push_back(extractTimeFromFileName(allfilenames.at(j), "Epoch_", 3));  ///< Define your own file name prefix.
	}

	// sort all files in ascending order based on time
	std::map<string, long> Fname_time;
	for (int i = 0; i < allfiletime.size(); ++i) {
		Fname_time.insert(std::make_pair(allfilenames.at(i), allfiletime.at(i)));
	}
	vector<pair<string, long>> Fname_time_sortedVct(Fname_time.begin(), Fname_time.end());
	sort(Fname_time_sortedVct.begin(), Fname_time_sortedVct.end(), cmpMap);
	long filetimetemp = 0;
	for (int k = 0; k < Fname_time_sortedVct.size(); ++k) {
		fileNameList.push_back(Fname_time_sortedVct[k].first);
		filetimetemp = Fname_time_sortedVct[k].second;
		fileTimeList.push_back(filetimetemp);
	}
	return fileNameList.size();
}


void getFiles(std::string folderpath, vector<std::string>& files)
{
	intptr_t hFile = 0;  ///< file handle
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(folderpath).append("/*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib &  _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(folderpath).append("/").append(fileinfo.name), files);
			}
			else {
				files.push_back(p.assign(folderpath).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


long extractTimeFromFileName(std::string fileName, const std::string substring, int timeLength)
{
	int startpos = fileName.find(substring) + substring.size();
	string alltime = fileName.substr(startpos, timeLength);
	return stol(alltime);
}


float calPCresolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	float res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	for (size_t i = 0; i < cloud->size(); ++i) {
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2) {
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
		else {
			std::cerr << "Error: Number of neighbor is 0! \n\n";
			return 0.0;
		}
	}
	if (n_points != 0) {
		res /= n_points;
	}
	return res;
}


double calPercentileDistBetween2PC(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, float percentile)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
	core.setInputTarget(cloud1);
	core.setInputSource(cloud2);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
	core.determineCorrespondences(*cor, DBL_MAX);
	int cornum = cor->size();
	double* distArray = new double[cornum];
	for (int i = 0; i < cornum; ++i)
		distArray[i] = sqrt(cor->at(i).distance);
	double percentileDist = calArrayPercentileElement(distArray, cornum, percentile);
	delete[] distArray;
	return percentileDist;
}


bool calPatchNormal(
	pcl::PointCloud<pcl::PointXYZ> cloud, float &nx, float &ny, float &nz)
{
	float nmlx = 0; float nmly = 0; float nmlz = 2;
	Eigen::Vector4f planepara;
	float curv;
	if (cloud.size() > 4 && computePointNormal(cloud, planepara, curv)) 
	{
		nmlx = planepara[0];
		nmly = planepara[1];
		nmlz = planepara[2];
		// checking the length of estimated normal
		float nLen = sqrt(nmlx * nmlx + nmly * nmly + nmlz * nmlz);
		if (fabs(nLen - 1.0) < 1e-5) {
			nx = nmlx;
			ny = nmly;
			nz = nmlz;
			return true;
		}
		else { // Recalculate normal
			int Pn = cloud.size();
			Eigen::MatrixXf cloud_mat(Pn, 3);
			for (int k = 0; k < Pn; ++k) {
				cloud_mat.row(k) = cloud.points[k].getVector3fMap();
			}
			// SVD
			Eigen::RowVector3f meanVector1 = cloud_mat.colwise().mean();
			cloud_mat.rowwise() -= meanVector1;
			Eigen::Matrix3f covMat = (cloud_mat.transpose() * cloud_mat) / Pn;
			Eigen::JacobiSVD<Eigen::Matrix3f> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::Matrix3f V = svd.matrixV();
			nx = V(0, 2);
			ny = V(1, 2);
			nz = V(2, 2);
			float nLen2 = sqrt(nx * nx + ny * ny + nz * nz);
			if (fabs(nLen2 - 1.0) < 1e-5) {
				return true;
			}
			else {
				std::cerr << "Incalculable normals: " << nx << ", " << ny << ", " << nz << ": " << sqrt(nLen2) << " !!! \n\n";
				return false;
			}
		}
	}
	else {
		std::cerr << "Patch normal calculation fails !!! Assigned with (0, 0, 1) \n\n";
		nx = 0; ny = 0; nz = 1;
		return false;
	}
}


float calPatchSTD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int pointNum = cloud->size();
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	Eigen::RowVector3f centroid = pca.getMean().head<3>();
	Eigen::RowVector3f normal = pca.getEigenVectors().col(2);
	float coefA = normal(0);
	float coefB = normal(1);
	float coefC = normal(2);
	float coefD = -normal.dot(centroid);
	double stdDist = 0.0f;
	for (int i = 0; i < pointNum; ++i) {
		double Po2Pldis = pcl::pointToPlaneDistance(
			cloud->points[i], coefA, coefB, coefC, coefD);
		stdDist += Po2Pldis * Po2Pldis;
	}
	return sqrt(stdDist / double(pointNum - 1));
}


void generateCentroidCloudWithPatchNormals(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentroids, pcl::PointCloud<pcl::PointXYZ>* cloudPatch, 
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudCentroids_normals)
{
	// Normals of all centroids
	pcl::PointCloud<pcl::Normal> normals;
	normals.resize(cloudCentroids->size());
	// Estimate normals from each corresponding patch
	for (int i = 0; i < cloudCentroids->size(); ++i) {
		float nx = 0.0f, ny = 0.0f, nz = 1.0f;
		if (cloudPatch[i].size() > 6 && calPatchNormal(cloudPatch[i], nx, ny, nz)) {
			normals[i].normal_x = nx;
			normals[i].normal_y = ny;
			normals[i].normal_z = nz;
		}
		else {
			std::cerr << "[Warning] Failed to estimate normal for patch " << i
				<< ". Assigning default normal (0, 0, 1).\n";
			normals[i].normal_x = 0.0f;
			normals[i].normal_y = 0.0f;
			normals[i].normal_z = 1.0f;
		}
	}
	// Merge centroids with estimated normals into a single PointNormal cloud
	pcl::concatenateFields(*cloudCentroids, normals, *cloudCentroids_normals);
}


void matrix2angle(Eigen::Matrix4f transMat, Eigen::Vector3f &rotAngle)
{
	double ax, ay, az;
	if (transMat(2, 0) == 1 || transMat(2, 0) == -1) {
		az = 0;
		double dlta;
		dlta = atan2(transMat(0, 1), transMat(0, 2));
		if (transMat(2, 0) == -1) {
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else {
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else {
		ay = -asin(transMat(2, 0));
		ax = atan2(transMat(2, 1) / cos(ay), transMat(2, 2) / cos(ay));
		az = atan2(transMat(1, 0) / cos(ay), transMat(0, 0) / cos(ay));
	}
	rotAngle << ax, ay, az;
}


float calBoundingBoxCornerChange(const double* boundingBox, const Eigen::Matrix4f transMat)
{
	Eigen::Vector4f cornerMin(boundingBox[0], boundingBox[1], boundingBox[2], 1.0f);
	Eigen::Vector4f cornerMax(boundingBox[3], boundingBox[4], boundingBox[5], 1.0f);
	Eigen::Vector4f transformedMin = transMat * cornerMin;
	Eigen::Vector4f transformedMax = transMat * cornerMax;
	float diffMin = (transformedMin.head<3>() - cornerMin.head<3>()).norm();
	float diffMax = (transformedMax.head<3>() - cornerMax.head<3>()).norm();
	return std::max(diffMin, diffMax);
}



void PCpreprocessing(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
	bool isDownSamp, float voxelSize, int SOR_NeighborNum, double SOR_StdMult)
{
	// Voxel-grid downsampling
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	if (isDownSamp) {
		pcl::VoxelGrid<pcl::PointXYZ> VoxFilter;
		VoxFilter.setInputCloud(cloud_in);
		VoxFilter.setLeafSize(voxelSize, voxelSize, voxelSize);
		VoxFilter.filter(*cloud_filtered1);
	}
	else
		pcl::copyPointCloud(*cloud_in, *cloud_filtered1);
	// removing outliers by SOR
	SORfilter(cloud_filtered1, cloud_out, SOR_NeighborNum, SOR_StdMult);
}


void SORfilter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
	int SOR_NeighborNum, double SOR_StdMult)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> StatReFilter;
	StatReFilter.setInputCloud(cloud_in);
	StatReFilter.setMeanK(SOR_NeighborNum);			//Number of KNN
	StatReFilter.setStddevMulThresh(SOR_StdMult);	//Multiple of Std.
	StatReFilter.setNegative(false);
	StatReFilter.filter(*cloud_out);
}



void visualizeTwoPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, string name1, double PCr1, double PCg1, double PGb1, int size1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, string name2, double PCr2, double PCg2, double PGb2, int size2,
	double BGr, double BGg, double BGb)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("TwoPC"));
	viewer->setBackgroundColor(BGr, BGg, BGb);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, name1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, PCr1, PCg1, PGb1, name1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, name1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, name2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, PCr2, PCg2, PGb2, name2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, name2);
	viewer->setSize(1200, 800);
	while (!viewer->wasStopped())
		viewer->spinOnce(100);
}


void visualizeThreePC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, string name1, double PCr1, double PCg1, double PGb1, int size1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, string name2, double PCr2, double PCg2, double PGb2, int size2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3, string name3, double PCr3, double PCg3, double PGb3, int size3,
	double BGr, double BGg, double BGb)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ThreePC"));
	viewer->setBackgroundColor(BGr, BGg, BGb);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, name1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, PCr1, PCg1, PGb1, name1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, name1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, name2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, PCr2, PCg2, PGb2, name2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, name2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud3, name3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, PCr3, PCg3, PGb3, name3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size3, name3);
	viewer->setSize(1200, 800);
	while (!viewer->wasStopped())
		viewer->spinOnce(100);
}

