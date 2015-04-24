#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/TextureMesh.h>
#include <rply.h>

namespace pcl_utility
{
	namespace polygon_mesh
	{
		typedef struct  
		{
			pcl::PointXYZ vertex_buf;
			pcl::PointCloud<pcl::PointXYZ>::Ptr points;
		}VertexCallbackData;

		typedef struct
		{
			pcl::Vertices face_buf;
			std::shared_ptr<std::vector<pcl::Vertices>> faces;
		}FaceCallbackData;

		int LoadPolygonMeshFromPLY(const std::string &filename, 
			pcl::PolygonMesh& mesh);

		static int vertex_callback(p_ply_argument argument);
		static int face_callback(p_ply_argument argument);

		int LoadTextureMeshFromOBJ(const std::string & filename,
			pcl::TextureMesh & textured_mesh);
	}	
}