#include "mesh_segmentation/mesh_segmentation.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcl_utility/pcl_utility.hpp"

#define DBL_THRESHOLD (1.e-7)

namespace mesh_segmentation
{

	MeshSegmentation::MeshSegmentation(std::string in,
		int row, int col, double dilation,
		std::string out_dir)
	{
		m_mesh_file = in;
		m_out_dir = out_dir;
		m_n_row = row;
		m_n_col = col;
		m_dilation = dilation;
	}

	MeshSegmentation::~MeshSegmentation()
	{
	}

	int MeshSegmentation::Segment(std::vector<std::string> & mesh_files)
	{
		int rtn = 0;

		do 
		{
			if (m_mesh_file.empty() || m_n_row * m_n_col <= 1)
			{
				std::cout<<"mesh file could not be empty. or blablabla" <<std::endl;
				break;
			}

			pcl::PolygonMesh::Ptr in_mesh(new pcl::PolygonMesh);
			std::vector<pcl::PolygonMesh> sub_meshes;
			sub_meshes.resize(m_n_row * m_n_col);

			Eigen::Matrix4d t_matrix;
			pcl::PointCloud<pcl::PointXYZ>::Ptr in_mesh_point_cloud(
				new pcl::PointCloud<pcl::PointXYZ>);
			

			//1.mesh read from ply file	
			std::cout<<"loading file..."<<std::endl;
			if (pcl_utility::polygon_mesh::LoadPolygonMeshFromPLY(
				m_mesh_file, *in_mesh))
			{
				std::cout<<"LoadPolygonMeshFromPLY failed."<<std::endl;
				break;
			}

			//2.main planar			
			pcl::fromPCLPointCloud2(in_mesh->cloud, *in_mesh_point_cloud);
			in_mesh->cloud.data.swap(std::vector<pcl::uint8_t>()); //note: can't use cloud in mesh anymore!

			std::cout<<"main planar and transformation matrix..."<<std::endl;
			std::vector<double> coefficents;
			if (MainPlanarAndTransformMatrix(in_mesh_point_cloud, 
				coefficents, t_matrix))
			{
				std::cout
					<<"estimate main planar or transformation matrix fail."
					<<std::endl;
				break;
			}

			//3.face center & projected_pts & min_x,y max_x,y
			std::cout<<"calculate face center and boundary..."<<std::endl;
			ProjectedPointsPtr projected_center(new ProjectedPointsType);
			double min_x = std::numeric_limits<double>::max();
			double min_y = std::numeric_limits<double>::max();
			double max_x = -std::numeric_limits<double>::max();
			double max_y = -std::numeric_limits<double>::max();
			if (ProjectedFaceCenterAndBoundary( in_mesh,
				in_mesh_point_cloud,
				coefficents, t_matrix,
				projected_center, 
				min_x, max_x, min_y, max_y))
			{
				std::cout<<"ProjectedFaceCenterAndBoundary failed."<<std::endl;
				break;
			}

			//4.cutting mesh into sub meshes and write to PLY
			//note: will delete sub_mesh data after saving, can't not use sub_meshes any more!
			std::cout<<"assign faces into sub meshes..."<<std::endl;
			if (IntoSubMeshes( in_mesh, in_mesh_point_cloud, projected_center,
				min_x, max_x, min_y, max_y, sub_meshes, mesh_files))
			{
				std::cout<<"IntoSubMeshes failed."<<std::endl;
				break;
			}

			rtn = 0;
		} while (0);
segment:

		return rtn;
	}

	int MeshSegmentation::Segment(const std::vector<double> & coefficients,
		std::vector<std::string> & mesh_files )
	{
		int rtn = 0;

		do 
		{
			if (m_mesh_file.empty() || m_n_row * m_n_col <= 1)
			{
				std::cout<<"mesh file could not be empty. or blablabla" <<std::endl;
				break;
			}

			pcl::PolygonMesh::Ptr in_mesh(new pcl::PolygonMesh);
			std::vector<pcl::PolygonMesh> sub_meshes;
			sub_meshes.resize(m_n_row * m_n_col);

			Eigen::Matrix4d t_matrix;
			pcl::PointCloud<pcl::PointXYZ>::Ptr in_mesh_point_cloud(
				new pcl::PointCloud<pcl::PointXYZ>);


			//1.mesh read from ply file	
			std::cout<<"loading file..."<<std::endl;
			if (pcl_utility::polygon_mesh::LoadPolygonMeshFromPLY(
				m_mesh_file, *in_mesh))
			{
				std::cout<<"LoadPolygonMeshFromPLY failed."<<std::endl;
				break;
			}

			//2.main planar			
			pcl::fromPCLPointCloud2(in_mesh->cloud, *in_mesh_point_cloud);
			in_mesh->cloud.data.swap(std::vector<pcl::uint8_t>()); //note: can't use cloud in mesh anymore!

			std::cout<<"main planar and transformation matrix..."<<std::endl;
			if (TransformMatrix(in_mesh_point_cloud, 
				coefficients, t_matrix))
			{
				std::cout
					<<"estimate main planar or transformation matrix fail."
					<<std::endl;
				break;
			}

			//3.face center & projected_pts & min_x,y max_x,y
			std::cout<<"calculate face center and boundary..."<<std::endl;
			ProjectedPointsPtr projected_center(new ProjectedPointsType);
			double min_x = std::numeric_limits<double>::max();
			double min_y = std::numeric_limits<double>::max();
			double max_x = -std::numeric_limits<double>::max();
			double max_y = -std::numeric_limits<double>::max();
			if (ProjectedFaceCenterAndBoundary( in_mesh,
				in_mesh_point_cloud,
				coefficients, t_matrix,
				projected_center, 
				min_x, max_x, min_y, max_y))
			{
				std::cout<<"ProjectedFaceCenterAndBoundary failed."<<std::endl;
				break;
			}

			//4.cutting mesh into sub meshes and write to PLY
			//note: will delete sub_mesh data after saving, can't not use sub_meshes any more!
			std::cout<<"assign faces into sub meshes..."<<std::endl;
			if (IntoSubMeshes( in_mesh, in_mesh_point_cloud, projected_center,
				min_x, max_x, min_y, max_y, sub_meshes, mesh_files))
			{
				std::cout<<"IntoSubMeshes failed."<<std::endl;
				break;
			}

			rtn = 0;
		} while (0);
segment:

		return rtn;
	}

	int MeshSegmentation::MainPlanarAndTransformMatrix( 
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud, 
		std::vector<double> & coefficients, 
		Eigen::Matrix4d & t_matrix )
	{
		int rtn = -1;

		do 
		{
			if (!point_cloud)
			{
				std::cout<<"input point cloud is empty."<<std::endl;
				break;
			}
						

			//main plane
			pcl::ModelCoefficients::Ptr mp_coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_MSAC);
			seg.setDistanceThreshold (0.01);
			seg.setAxis(Eigen::Vector3f(0,0,1));

			seg.setInputCloud (point_cloud);
			seg.segment (*inliers, *mp_coefficients);

			if (inliers->indices.size () <= 0)
			{
				std::cout<<
					"Could not estimate a planar model for the given dataset."
					<<std::endl;
				break;
			}

			//coefficients = mp_coefficients->values;
			if (mp_coefficients->values.size() != 4 ||
				(fabs(mp_coefficients->values[0]) < DBL_THRESHOLD &&
				fabs(mp_coefficients->values[1]) < DBL_THRESHOLD &&
				fabs(mp_coefficients->values[2]) < DBL_THRESHOLD))
			{
				std::cout<<"wrong planar model."<<std::endl;
				break;
			}

			//test
			coefficients.resize(4);
			Eigen::Vector3d plane_normal(
				mp_coefficients->values[0], 
				mp_coefficients->values[1],
				mp_coefficients->values[2]);
			plane_normal.normalize();
			for (int i_c = 0; i_c < 3; ++i_c)
			{
				coefficients[i_c] = plane_normal(i_c);
				coefficients[3] += coefficients[i_c];
			}

			//transformation matrix
			Eigen::Vector3d ori_pt(0, 0, 0);
			Eigen::Vector3d tran_pt;
			if (PerpendicularFoot(coefficients, ori_pt, tran_pt))
			{
				std::cout << "PerpendicularFoot failed." << std::endl;
			}

			double a = sqrt(tran_pt(0) * tran_pt(0) +
				tran_pt(1) * tran_pt(1));
			double b = tran_pt.norm();

			if (!a || !b)
			{
				std::cout << "PerpendicularFoot failed." << std::endl;
			}

			t_matrix << 
				-tran_pt(1)/a, -tran_pt(0) * tran_pt(2)/(a*b), -tran_pt(0)/b, 0,
				tran_pt(0)/a, -tran_pt(1) * tran_pt(2)/(a*b), -tran_pt(1)/b, 0,
				0, a/b, -tran_pt(2)/b, 0,
				0, 0, b, 1;

			rtn = 0;
		} while (0);

		return rtn;
	}

	int MeshSegmentation::ProjectedFaceCenterAndBoundary( 
		pcl::PolygonMesh::ConstPtr mesh,
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
		const std::vector<double> & coefficients,
		Eigen::Matrix4d t_matrix,
		ProjectedPointsPtr o_projected_center,
		double & min_x, double & max_x,
		double & min_y, double & max_y )
	{
		int rtn = -1;

		do 
		{
			if (!mesh || !point_cloud || !o_projected_center)
			{
				break;
			}

			int num_faces = mesh->polygons.size();

			std::vector<Eigen::Vector3d> face_centers;
			face_centers.resize(num_faces);

			if (!o_projected_center->empty())
			{
				o_projected_center->clear();				
			}
			o_projected_center->resize(num_faces);
			
			for ( int i_f = 0; i_f < num_faces; ++i_f)
			{
				//1.三角形中点
				face_centers[i_f] = Eigen::Vector3d::Zero();
				const pcl::Vertices & vertics = mesh->polygons[i_f];
				int num_v = vertics.vertices.size();
				for (int i_v = 0; i_v < num_v; ++i_v)
				{
					int pt_index = vertics.vertices[i_v];
					const pcl::PointXYZ & pt = point_cloud->points[pt_index];
					face_centers[i_f](0)+=pt.x;
					face_centers[i_f](1)+=pt.y;
					face_centers[i_f](2)+=pt.z;
				}
				face_centers[i_f](0)/=static_cast<double>(num_v);
				face_centers[i_f](1)/=static_cast<double>(num_v);
				face_centers[i_f](2)/=static_cast<double>(num_v);

				//2.中点投影点及边界
				//perpendicular foot
				Eigen::Vector3d tran_pt;
				if (PerpendicularFoot(coefficients, face_centers[i_f], tran_pt))
				{
					std::cout<< "PerpendicularFoot failed...\n" <<std::endl;
					goto projected;
				}

				//project point
				Eigen::Vector4d pt(tran_pt(0), tran_pt(1), tran_pt(2), 1.);
				pt = pt.transpose() * t_matrix;
				pt /= pt(3);

				if (fabs(pt(2)) > DBL_THRESHOLD)
				{
					std::cout<< "project failed...\n" << std::endl;
					std::cout<<pt<<std::endl;
					//goto projected;		
					system("pause");
				}

				(*o_projected_center)[i_f].first = pt(0);
				(*o_projected_center)[i_f].second = pt(1);		

				min_x = std::min(min_x, pt(0));
				max_x = std::max(max_x, pt(0));
				min_y = std::min(min_y, pt(1));
				max_y = std::max(max_y, pt(1));
			}


			rtn = 0;
		} while (0);
		projected:

		return rtn;
	}
	
	int MeshSegmentation::PerpendicularFoot( 
		const std::vector<double> & coefficients,
		Eigen::Vector3d pt,
		Eigen::Vector3d & out_pt )
	{
		if (coefficients.size() != 4 ||
			(fabs(coefficients[0]) < DBL_THRESHOLD &&
			fabs(coefficients[1]) < DBL_THRESHOLD &&
			fabs(coefficients[2]) < DBL_THRESHOLD) )
		{
			return -1;
		}

		double t = - (coefficients[3] + 
			coefficients[0] * pt(0) +
			coefficients[1] * pt(1) +
			coefficients[2] * pt(2)) /
			(coefficients[0] * coefficients[0] +
			coefficients[1] * coefficients[1] +
			coefficients[2] * coefficients[2]);

		out_pt(0) = pt(0) + coefficients[0] * t;
		out_pt(1) = pt(1) + coefficients[1] * t;
		out_pt(2) = pt(2) + coefficients[2] * t;

		return 0;
	}

	std::string MeshSegmentation::GenerateSubMeshName( 
		int sub_mesh_index )
	{
		std::string result;
		std::stringstream temp;
		int row_idx = sub_mesh_index / m_n_col;
		int col_idx = sub_mesh_index - row_idx * m_n_col;

		std::string file_prefix = m_mesh_file;
		int pos = 0;
		if ((pos = file_prefix.find_last_of(".")) != std::string::npos)
		{
			file_prefix = file_prefix.substr(0, pos);
		}
		if ((pos = file_prefix.find_last_of("\\")) != std::string::npos)
		{
			file_prefix = file_prefix.substr(pos+1);
		}
		file_prefix += "_";

		temp << file_prefix << row_idx << "_" << col_idx << ".ply";
		temp >> result;
		result = m_out_dir + "\\" + result;
		return result;
	}

	int MeshSegmentation::IntoSubMeshes( 
		pcl::PolygonMesh::ConstPtr mesh, 
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud, 
		ProjectedPointsConstPtr face_centers, 
		double min_x, double max_x,
		double min_y, double max_y,
		std::vector<pcl::PolygonMesh> & sub_meshes,
		std::vector<std::string> & mesh_files)
	{
		int rtn = 0;
		
		do 
		{
			double row_unit = (max_y - min_y) / m_n_row;
			double col_unit = (max_x - min_x) / m_n_col;
			int num_sub_meshes = m_n_col * m_n_row;
			mesh_files.swap(std::vector<std::string>());
										
			if (!mesh || !point_cloud || !face_centers ||
				sub_meshes.size() != num_sub_meshes ||
				row_unit <= 0. || col_unit <=0. ||
				m_dilation < 0. || m_dilation >1.)
			{
				break;
			}

			double row_dilation = m_dilation * row_unit;
			double col_dilation = m_dilation * col_unit;

			std::cout<<"dilation: "<<m_dilation<<std::endl;
			
			std::vector<std::vector <size_t> > sub_meshes_indexes;
			sub_meshes_indexes.resize(num_sub_meshes);

			bool north = false;
			bool south = false;
			bool west = false;
			bool east = false;
			size_t num_faces = face_centers->size();
			for (size_t i_f = 0; i_f < num_faces; ++i_f)
			{
				int row_idx, col_idx, sub_mesh_idx;
				double face_x = (*face_centers)[i_f].first;
				double face_y = (*face_centers)[i_f].second;

				//1.判断该面属于哪个sub_mesh
				row_idx = (face_y - min_y) / row_unit;
				col_idx = (face_x - min_x) / col_unit;
				row_idx = row_idx < m_n_row ? row_idx : m_n_row - 1;
				col_idx = col_idx < m_n_col ? col_idx : m_n_col - 1;
				sub_mesh_idx = row_idx * m_n_col + col_idx;

				if (row_idx > 0)
				{
					double boundary = min_y + row_idx * row_unit + row_dilation;
					if (face_y < boundary)
					{
						north = true;
					}
				}

				if (row_idx < m_n_row - 1)
				{
					double boundary = min_y + (row_idx+1) * row_unit - row_dilation;
					if (face_y > boundary)
					{
						south = true;
					}
				}

				if (col_idx > 0)
				{
					double boundary = min_x + col_idx * col_unit + row_dilation;
					if (face_x < boundary)
					{
						west = true;
					}
				}

				if (col_idx < m_n_col - 1)
				{
					double boundary = min_x + (col_idx+1) * col_unit - row_dilation;
					if (face_x > boundary)
					{
						east = true;
					}
				}

				//2.写入该sub_mesh
				sub_meshes_indexes[sub_mesh_idx].push_back(i_f);

				if (north)
				{
					sub_meshes_indexes[sub_mesh_idx - m_n_col].push_back(i_f);
					if (west)
						sub_meshes_indexes[sub_mesh_idx-m_n_col-1].push_back(i_f);
					if (east)
						sub_meshes_indexes[sub_mesh_idx-m_n_col+1].push_back(i_f);
				}

				if (west)
					sub_meshes_indexes[sub_mesh_idx-1].push_back(i_f);
				if (east)
					sub_meshes_indexes[sub_mesh_idx+1].push_back(i_f);

				if (south)
				{
					sub_meshes_indexes[sub_mesh_idx + m_n_col].push_back(i_f);
					if (west)
						sub_meshes_indexes[sub_mesh_idx + m_n_col - 1].push_back(i_f);
					if (east)
						sub_meshes_indexes[sub_mesh_idx + m_n_col + 1].push_back(i_f);
				}

				north = south = west = east = false;
			}

			//将信息填入sub_mesh and saving
			std::vector<std::string> temp_files;
			for (int i_sm = 0; i_sm < num_sub_meshes; ++i_sm)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud(
					new pcl::PointCloud<pcl::PointXYZ>);
				std::vector<pcl::Vertices> sub_polygons;
				std::map<uint32_t, uint32_t> all_to_sub_mapping;

				int num_faces = sub_meshes_indexes[i_sm].size();
				if (num_faces <= 0)
				{
					std::cout<<"no data in this sub mesh"<<std::endl;
					sub_meshes_indexes[i_sm].swap(std::vector<size_t>());
					continue;
				}

				for ( int i_f = 0; i_f < num_faces; ++i_f)
				{
					size_t face_idx = sub_meshes_indexes[i_sm][i_f];
					const pcl::Vertices & vertices = mesh->polygons[face_idx];
					pcl::Vertices sub_vertices;
					for (int i_v = 0; i_v < vertices.vertices.size(); ++i_v)
					{
						uint32_t pt_idx = vertices.vertices[i_v];
						std::map<uint32_t, uint32_t>::iterator itr_pt = 
							all_to_sub_mapping.find(pt_idx);
						if (itr_pt != all_to_sub_mapping.end())
						{
							sub_vertices.vertices.push_back(itr_pt->second);
						} else {
							sub_cloud->push_back(point_cloud->points[pt_idx]);
							all_to_sub_mapping[pt_idx] = sub_cloud->size() - 1;
							sub_vertices.vertices.push_back(all_to_sub_mapping[pt_idx]);
						}
					}
					sub_polygons.push_back(sub_vertices);
				}

				sub_meshes[i_sm].polygons.swap(sub_polygons);
				pcl::toPCLPointCloud2(*sub_cloud, sub_meshes[i_sm].cloud);

				std::string filename = GenerateSubMeshName(i_sm);
				std::cout<<"save "<<filename<<"..."<<std::endl;
				std::cout<<num_faces<<" in this mesh"<<std::endl;
				if (pcl::io::savePLYFile(filename, sub_meshes[i_sm], 15))
				{
					std::cout<<"savePLYFile failed."<<std::endl;
					//goto sub_meshes;
				}	

				temp_files.push_back(filename);
				
				//clear data;
				sub_meshes[i_sm].cloud.data.swap(std::vector<pcl::uint8_t>());
				sub_meshes[i_sm].polygons.swap(std::vector<pcl::Vertices>());
				sub_meshes_indexes[i_sm].swap(std::vector<size_t>());
			}

			mesh_files.swap(temp_files);

			rtn = 0;
		} while (0);
		sub_meshes:

		return rtn;
	}

	int MeshSegmentation::TransformMatrix( 
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud,
		const std::vector<double> & coefficients,
		Eigen::Matrix4d & t_matrix )
	{
		int rtn = -1;

		do 
		{
			if (!point_cloud || coefficients.size() != 4)
			{
				std::cout<<"input point cloud is empty or invalid plane coefficients."<<std::endl;
				break;
			}			

			//transformation matrix
			Eigen::Vector3d ori_pt(0, 0, 0);
			Eigen::Vector3d tran_pt;
			if (PerpendicularFoot(coefficients, ori_pt, tran_pt))
			{
				std::cout << "PerpendicularFoot failed." << std::endl;
			}

			double a = sqrt(tran_pt(0) * tran_pt(0) +
				tran_pt(1) * tran_pt(1));
			double b = tran_pt.norm();

			if (!a || !b)
			{
				std::cout << "PerpendicularFoot failed." << std::endl;
			}

			t_matrix << 
				-tran_pt(1)/a, -tran_pt(0) * tran_pt(2)/(a*b), -tran_pt(0)/b, 0,
				tran_pt(0)/a, -tran_pt(1) * tran_pt(2)/(a*b), -tran_pt(1)/b, 0,
				0, a/b, -tran_pt(2)/b, 0,
				0, 0, b, 1;

			rtn = 0;
		} while (0);

		return rtn;
	}

}
