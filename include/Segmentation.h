/**
 * @file Segmentation.h
 * @brief Patch-based segmentation for 3D point clouds.
 * @details This module implements the extraction and characterization of planar patches from 3D point clouds 
 *          for use in Piecewise-ICP. It includes methods to:
 *          - Segment point clouds into local planar patches (supervoxels).
 *          - Compute geometric features such as centroid, boundary points, and normal vectors of patches.
 *          - Evaluate patch-level statistics such as standard deviation and shape descriptors.
 *          The code for supervoxel segmentation is developed based on https://github.com/yblin/Supervoxel-for-3D-point-clouds
 * @author Dr.-Ing. Yihui Yang (yihui.yang@tum.de)
 * @version 1.0
 * @date 2025-06-06
 * @license Apache-2.0
 * @copyright 2025 Yihui Yang, TU Munich. All Rights Reserved.
 */

#pragma once
#include "CommonFunc.h"
#include "codelibrary/geometry/util/distance_3d.h"
#include "codelibrary/geometry/point_cloud/pca_estimate_normals.h"
#include "codelibrary/geometry/point_cloud/supervoxel_segmentation.h"


/// Point with Normal
struct PointWithNormal: cl::RPoint3D {
	PointWithNormal() {}
	cl::RVector3D normal;
};


/**********************************************
 * VCCS Supervoxel Segmentation for 3D Point Cloud.
 * Reference:
 *		Papon J, Abramov A, Schoeler M, et al., 2013. Voxel cloud connectivity
 *		segmentation-supervoxels for point clouds. Computer Vision and Pattern
 *		Recognition (CVPR), pp. 2027-2034.
 *
 * Copyright 2016 Yangbin Lin. All Rights Reserved.
 * Author: yblin@jmu.edu.cn (Yangbin Lin)
 **********************************************/
class VCCSSupervoxel {
public:
	/// A voxel represents a subset of point cloud, in a regularly spaced, three-dimensional grid.
	struct Voxel {
		int id;                     /// ID of this voxel.
		int key_x, key_y, key_z;    /// Index of this voxel in the octree.
		int supervoxel_id;          /// ID of supervoxel that owns this voxel.
		cl::RPoint3D centroid;      /// Centroid point of this voxel.
		cl::RVector3D normal;       /// Normal vector of this voxel.
		cl::Array<int> indices;     /// Indices of points in this voxel.
		cl::Array<int> neighbors;   /// Neigbor voxels.
		double distance;            /// Distance to the supervoxel.
	};

	/// The supervoxel is represented by a pair (point, normal).
	struct Supervoxel {
		Supervoxel() {}
		Supervoxel(const cl::RPoint3D& p, const cl::RVector3D& n)
			: point(p), normal(n) {}
		cl::RPoint3D point;
		cl::RVector3D normal;
	};

	/// Construct the supervoxel by given [first, last) point cloud and the resolutions.
	template <typename Iterator>
	VCCSSupervoxel(Iterator first, Iterator last, double resolution,
		double seed_resolution)
		: resolution_(resolution),
		seed_resolution_(seed_resolution),
		spatial_importance_(0.5f),
		normal_importance_(1.0f)
	{
		assert(resolution_ > 0.0);
		assert(seed_resolution_ >= 2.0 * resolution_);

		size_points_ = std::distance(first, last);
		assert(size_points_ >= 0);

		Voxelize(first, last, resolution_, true, &voxels_);

		// Compute the normals.
		for (Voxel& v : voxels_) {
			cl::Array<cl::RPoint3D> points;
			for (int index : v.indices) {
				points.push_back(first[index]);
			}
			for (int neighbor : v.neighbors) {
				const Voxel& v1 = voxels_[neighbor];
				for (int index : v1.indices) {
					points.push_back(first[index]);
				}
			}

			cl::geometry::point_cloud::PCAEstimateNormal(points.begin(),
				points.end(),
				&v.normal);
		}

		InitialSupervoxelSeeds();
	}

	void set_spatial_importance(double spatial_importance) {
		assert(spatial_importance >= 0.0);

		spatial_importance_ = spatial_importance;
	}

	void set_normal_importance(double normal_importance) {
		assert(normal_importance >= 0.0);

		normal_importance_ = normal_importance;
	}

	/// Segment the given point cloud. It returns an array 'labels' and supervoxels.
	/// labels[i] denotes the id of supervoxel that owns the i-th point.
	void Segment(cl::Array<int>* labels,
		cl::Array<Supervoxel>* supervoxels) {
		assert(labels);
		assert(supervoxels);

		supervoxels->resize(seed_supervoxels_.size());

		for (Voxel& v : voxels_) {
			v.supervoxel_id = -1;
			v.distance = DBL_MAX;
		}

		cl::Array<Voxel*> queue = seed_supervoxels_;
		for (int i = 0; i < seed_supervoxels_.size(); ++i) {
			Voxel* v = seed_supervoxels_[i];
			v->supervoxel_id = i;
			v->distance = 0.0;
			(*supervoxels)[i] = Supervoxel(v->centroid, v->normal);
		}

		int depth = 1.8 * seed_resolution_ / resolution_;
		for (int i = 1; i < depth; ++i) {
			cl::Array<Voxel*> new_queue;
			for (Voxel* cur : queue) {
				const Supervoxel& s = (*supervoxels)[cur->supervoxel_id];

				for (int neighbor : cur->neighbors) {
					Voxel& v1 = voxels_[neighbor];
					if (v1.supervoxel_id == cur->supervoxel_id) continue;

					double dis = VoxelDistance(v1, s);
					if (dis < v1.distance) {
						v1.distance = dis;
						v1.supervoxel_id = cur->supervoxel_id;
						new_queue.push_back(&v1);
					}
				}
			}
			queue = new_queue;

			/// Update supervoxels.
			UpdateSupervoxels(supervoxels);
		}

		labels->resize(size_points_);
		std::fill(labels->begin(), labels->end(), -1);
		for (const Voxel& v : voxels_) {
			for (int index : v.indices) {
				(*labels)[index] = v.supervoxel_id;
			}
		}
	}

	const cl::Array<Voxel>& voxels() const { return voxels_; }

private:
	/// Initial supervoxel seeds.
	void InitialSupervoxelSeeds() {
		cl::Array<cl::RPoint3D> voxel_points;
		voxel_points.reserve(voxels_.size());
		for (const Voxel& v : voxels_) {
			voxel_points.push_back(v.centroid);
		}

		cl::Array<Voxel> seed_points;
		Voxelize(voxel_points.begin(), voxel_points.end(), seed_resolution_,
			false, &seed_points);
		cl::KDTree<cl::RPoint3D> kd_tree(voxel_points.begin(),
			voxel_points.end());

		cl::Array<int> voxel_indices(seed_points.size());
		for (int i = 0; i < seed_points.size(); ++i) {
			kd_tree.FindNearestPoint(seed_points[i].centroid,
				&voxel_indices[i]);
		}

		seed_supervoxels_.reserve(seed_points.size());

		double search_radius = 0.5 * seed_resolution_;
		double sqr_search_radius = search_radius * search_radius;

		/// Compute the minimum number of voxels which fit in a planar slice through search volume.
		cl::Array<int> neighbors;
		for (int voxel_id : voxel_indices) {
			Voxel& v = voxels_[voxel_id];
			kd_tree.FindRadiusNeighbors(v.centroid, sqr_search_radius,
				&neighbors);
			seed_supervoxels_.push_back(&v);
		}
	}

	/// Compute the distance from voxel to supervoxel.
	double VoxelDistance(const Voxel& v, const Supervoxel& s) const {
		double n_dist = 1.0 - std::fabs(v.normal * s.normal);
		double s_dist = cl::geometry::Distance(v.centroid, s.point) /
			seed_resolution_;
		return normal_importance_ * n_dist + spatial_importance_ * s_dist;
	}

	/// Update the supervoxels.
	void UpdateSupervoxels(cl::Array<Supervoxel>* supervoxels) {
		cl::Array<cl::Array<Voxel*> > clusters(supervoxels->size());
		for (Voxel& v : voxels_) {
			if (v.supervoxel_id != -1) {
				clusters[v.supervoxel_id].push_back(&v);
			}
		}

		for (int i = 0; i < clusters.size(); ++i) {
			cl::Array<Voxel*>& cluster = clusters[i];
			if (cluster.empty()) continue;

			Supervoxel& s = (*supervoxels)[i];

			cl::RPoint3D centroid;
			for (Voxel* voxel : cluster) {
				centroid.x += voxel->centroid.x;
				centroid.y += voxel->centroid.y;
				centroid.z += voxel->centroid.z;

				if (s.normal * voxel->normal < 0.0) {
					s.normal += -voxel->normal;
				}
				else {
					s.normal += voxel->normal;
				}
			}
			s.normal *= 1.0 / s.normal.norm();
			centroid.x /= cluster.size();
			centroid.y /= cluster.size();
			centroid.z /= cluster.size();

			double distance = DBL_MAX;
			Voxel* nearest_voxel = NULL;
			for (Voxel* voxel : cluster) {
				double d = cl::geometry::Distance(voxel->centroid, centroid);
				if (d < distance) {
					distance = d;
					nearest_voxel = voxel;
				}
			}

			assert(nearest_voxel);
			s.point = nearest_voxel->centroid;
		}
	}

	/// Convert point cloud into voxels.
	template <typename Iterator>
	void Voxelize(Iterator first, Iterator last, double resolution,
		bool is_compute_neighbors, cl::Array<Voxel>* voxels) {
		voxels->clear();

		cl::RBox3D box(first, last);
		int size1 = box.x_length() / resolution + 1;
		int size2 = box.y_length() / resolution + 1;
		int size3 = box.z_length() / resolution + 1;

		cl::Octree<int> octree(size1, size2, size3);
		typedef typename cl::Octree<int>::LeafNode LeafNode;

		/// Add the voxels into the octree.
		int index = 0;
		for (Iterator p = first; p != last; ++p, ++index) {
			int x = (p->x - box.x_min()) / resolution;
			int y = (p->y - box.y_min()) / resolution;
			int z = (p->z - box.z_min()) / resolution;
			x = cl::Clamp(x, 0, size1 - 1);
			y = cl::Clamp(y, 0, size2 - 1);
			z = cl::Clamp(z, 0, size3 - 1);

			std::pair<LeafNode*, bool> pair = octree.Insert(x, y, z,
				voxels->size());
			if (pair.second) {
				Voxel v;
				v.key_x = x;
				v.key_y = y;
				v.key_z = z;
				v.id = voxels->size();
				voxels->push_back(v);
			}
			Voxel& voxel = (*voxels)[pair.first->data()];
			voxel.indices.push_back(index);
			voxel.centroid.x += p->x;
			voxel.centroid.y += p->y;
			voxel.centroid.z += p->z;
		}

		/// Compute the adjacent voxels for each voxel in the Octree.
		for (Voxel& v : *voxels) {
			v.centroid.x /= v.indices.size();
			v.centroid.y /= v.indices.size();
			v.centroid.z /= v.indices.size();
		}

		if (is_compute_neighbors) {
			/// Compute the neighbors for each voxel.
			for (Voxel& v : *voxels) {
				for (int x1 = -1; x1 <= 1; ++x1) {
					for (int y1 = -1; y1 <= 1; ++y1) {
						for (int z1 = -1; z1 <= 1; ++z1) {
							int x = v.key_x + x1;
							int y = v.key_y + y1;
							int z = v.key_z + z1;

							if (x < 0 || x >= octree.size1()) continue;
							if (y < 0 || y >= octree.size2()) continue;
							if (z < 0 || z >= octree.size3()) continue;

							LeafNode* leaf = octree.Find(x, y, z);
							if (!leaf) continue;

							v.neighbors.push_back(leaf->data());
						}
					}
				}
			}
		}
	}

	/// Number of points in the input point cloud.
	int size_points_;
	/// Resolution used in the octree.
	double resolution_;
	/// Resolution used to seed the supervoxels.
	double seed_resolution_;
	/// Importance of distance from seed center in clustering.
	double spatial_importance_;
	/// Importance of similarity in normals for clustering.
	double normal_importance_;
	/// Voxels.
	cl::Array<Voxel> voxels_;
	/// The seed supervoxels.
	cl::Array<Voxel*> seed_supervoxels_;
};


/**********************************************
 * Metric used in VCCS supervoxel segmentation.
 * Reference:
 *		Rusu, R.B., Cousins, S., 2011. 3d is here: Point cloud library (pcl),
 *		IEEE International Conference on Robotics and Automation, pp. 1–4.
 *
 * Copyright 2016 Yangbin Lin. All Rights Reserved.
 * Author: yblin@jmu.edu.cn (Yangbin Lin)
**********************************************/
class VCCSMetric {
public:
	explicit VCCSMetric(double resolution)
		: resolution_(resolution) {}

	double operator() (const PointWithNormal& p1,
		const PointWithNormal& p2) const {
		return 1.0 - std::fabs(p1.normal * p2.normal) +
			cl::geometry::Distance(p1, p2) / resolution_ * 0.4;
	}

private:
	double resolution_;
};



/**
 * @brief Generate, refine and select planar patches (supervoxels) from a point cloud.
 * 
 * This function performs patch-based segmentation using a given supervoxel resolution,
 * refines each patch by removing outliers, evaluates geometric features to retain planar patches,
 * and computes the centroids and boundary points of valid patches.
 *
 * Reference:
 *		Lin, Y., Wang, C., Zhai, D., et al., 2018. Toward better boundary preserved supervoxel 
 *		segmentation for 3D point clouds. ISPRS J. Photogram. Remote Sens., pp. 39-47.
 * 
 * @param[in] cloud           Input point cloud.
 * @param[in] svResolution    Initial resolution for supervoxel generation.
 * @param[out] cloudCentroid  Output point cloud storing the centroids of selected patches.
 * @param[out] cloudBoundary  Output point cloud storing the boundary points of selected patches.
 * @param[out] cloudPatches   Array of point clouds for storing selected patches.
 * @param[in] isVis           If true, visualization of generated patches will be shown.
 * 
 * @return Number of finally selected patches.
 */
int PatchGenerationAndRefinement(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float svResolution,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentroid,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBoundary,
	pcl::PointCloud<pcl::PointXYZ>* &cloudPatches, bool isVis);


/**
 * @brief Refine a planar patch by removing outliers based on point-to-plane distances.
 * 
 * @param[in] cloud         Input point cloud of a patch.
 * @param[out] refinedPatch Output point cloud after removing outliers.
 * @param[in] sigmaMul      Multiplier for the standard deviation used as the distance threshold.
 * @return Number of points remaining in the refined patch.
 */
int PatchRefinement(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr refinedPatch, double sigmaMul);


/**
 * @brief Calculate geometric features of a patch point cloud based on eigenvalue analysis.
 *
 * @param[in] cloud         Input point cloud patch.
 * @param[out] variation    Output variation.
 * @param[out] planarity    Output planarity.
 * @param[out] linearity    Output linearity.
 */
void calPatchFeature(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &variation, float &planarity, float &linearity);


/**
 * @brief Calculate the centroid and boundary points of a patch (supervoxel).
 *
 * @param[in] cloud             Input point cloud representing one patch.
 * @param[out] centroid         Output centroid of the patch.
 * @param[out] cloudBP         Output six boundary points of the patch.
 * @return                      Number of boundary points of the patch.
 */
int calPatchCTandBP(
	pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointXYZ &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBP);


/**
 * @brief Calculate the standard deviation of centroids (CT) and 
 *        the average residuals of boundary points (BPs) for all patches.
 *
 * @param[in]  cloudPatches  Array of input point cloud patches.
 * @param[in]  patchNum      Total number of selected patches.
 * @param[out] stdBP         Output standard deviations of boundary points for all patches.
 * @param[out] stdCT         Output standard deviations of centroids for all patches.
 */
void calBPandCTSTD(
	pcl::PointCloud<pcl::PointXYZ>* cloudPatches, int patchNum, std::vector<float>& stdBP, std::vector<float>& stdCT);
