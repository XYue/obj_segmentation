#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

namespace mesh_segmentation{

	class MeshSegmentation
	{
	public:
		typedef std::vector<std::pair<double, double> > ProjectedPointsType;
		typedef std::shared_ptr<ProjectedPointsType> ProjectedPointsPtr;
		typedef std::shared_ptr<ProjectedPointsType const> ProjectedPointsConstPtr;

	public:
		//MeshSegmentation();
		MeshSegmentation(std::string in,
			int row, int col, double dilation,
			std::string out_dir);
		~MeshSegmentation();

		int Segment(std::vector<std::string> & mesh_files);
		int Segment(const std::vector<double> & coefficients,
			std::vector<std::string> & mesh_files);

	protected:
		int MainPlanarAndTransformMatrix(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
			std::vector<double> & coefficients,
			Eigen::Matrix4d & t_matrix);

		int TransformMatrix(
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
			const std::vector<double> & coefficients,
			Eigen::Matrix4d & t_matrix);

		int ProjectedFaceCenterAndBoundary(
			pcl::PolygonMesh::ConstPtr mesh,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
			const std::vector<double> & coefficients,
			Eigen::Matrix4d t_matrix,
			ProjectedPointsPtr o_projected_center,
			double & min_x, double & max_x,
			double & min_y, double & max_y);

		int IntoSubMeshes(
			pcl::PolygonMesh::ConstPtr mesh,
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
			ProjectedPointsConstPtr face_centers,
			double min_x, double max_x,
			double min_y, double max_y,
			std::vector<pcl::PolygonMesh> & sub_meshes,
			std::vector<std::string> & mesh_files);

		int PerpendicularFoot(
			const std::vector<double> & coefficients,
			Eigen::Vector3d pt,
			Eigen::Vector3d & out_pt );

		std::string GenerateSubMeshName(int sub_mesh_index);

	protected:
		std::string m_mesh_file;
		std::string m_out_dir;
		int m_n_row, m_n_col;
		double m_dilation;
	};

}