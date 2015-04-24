#include "pcl_utility/pcl_utility.hpp"

#include <pcl/conversions.h>

namespace pcl_utility
{
	namespace polygon_mesh
	{		
		int LoadPolygonMeshFromPLY( const std::string &filename,
			pcl::PolygonMesh& mesh )
		{
			int rtn = -1;

			p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);

			do
			{
				if ( !ply_file ) break;
				if ( !ply_read_header(ply_file) ) break;

				VertexCallbackData cb_vertex;
				cb_vertex.points = pcl::PointCloud<pcl::PointXYZ>::Ptr(
					new pcl::PointCloud<pcl::PointXYZ>);
				int num_vertex = ply_set_read_cb(
					ply_file, "vertex", "x", vertex_callback, &cb_vertex, 0);
				ply_set_read_cb(
					ply_file, "vertex", "y", vertex_callback, &cb_vertex, 1);
				ply_set_read_cb(
					ply_file, "vertex", "z", vertex_callback, &cb_vertex, 2);

				FaceCallbackData cb_faces;
				cb_faces.faces = std::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>);
				int num_face = ply_set_read_cb(
					ply_file, "face", "vertex_indices", face_callback, &cb_faces, 0);
				if (num_face == 0)
					num_face = ply_set_read_cb(
					ply_file, "face", "vertex_index", face_callback, &cb_faces, 0);
				
				//reserve
				cb_vertex.points->points.reserve(num_vertex);
				cb_faces.faces->reserve(num_face);

				if ( !ply_read(ply_file) ) break;

				if (num_vertex != cb_vertex.points->points.size() ||
					num_face != cb_faces.faces->size())
					break;

				pcl::toPCLPointCloud2(*cb_vertex.points, mesh.cloud);
				mesh.polygons = *cb_faces.faces;

				rtn = 0;
			} while(0);

			if ( ply_file )
			{
				ply_close(ply_file);
				ply_file = NULL;
			}

			return rtn;
		}

		int vertex_callback( p_ply_argument argument )
		{
			long offset;
			VertexCallbackData * cb_vertex = NULL;
			ply_get_argument_user_data(argument, (void**)&cb_vertex, &offset);
			if( !cb_vertex ) return 0;

			double val = ply_get_argument_value(argument);

			switch(offset)
			{
			case 0:
				cb_vertex->vertex_buf.x = val;
				break;
			case 1:
				cb_vertex->vertex_buf.y = val;
				break;
			case 2:
				cb_vertex->vertex_buf.z = val;	
				cb_vertex->points->points.push_back(cb_vertex->vertex_buf);
				break;
			default:
				return 0;
			}

			return 1;
		}

		int face_callback( p_ply_argument argument )
		{
			long length, value_index;
			FaceCallbackData * cb_faces = NULL;
			ply_get_argument_user_data(argument, (void**)&cb_faces, NULL);
			ply_get_argument_property(argument, NULL, &length, &value_index);

			if ( value_index >= 0 && value_index < length)
			{
				cb_faces->face_buf.vertices.push_back(static_cast<uint32_t>(
					ply_get_argument_value(argument)));
				if ( length-1 == value_index )
				{
					cb_faces->face_buf.vertices.shrink_to_fit();
					cb_faces->faces->push_back(cb_faces->face_buf);
					cb_faces->face_buf = pcl::Vertices();
				}
			}

			return 1;
		}

		int LoadTextureMeshFromOBJ( const std::string & filename, pcl::TextureMesh & textured_mesh )
		{
			int ret = -1;

			do 
			{
// 				int
// 					pcl::OBJReader::read (const std::string &file_name, pcl::TextureMesh &mesh,
// 					Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
// 					int &file_version, const int offset)
// 				{
// 					pcl::console::TicToc tt;
// 					tt.tic ();
// 
// 					int data_type;
// 					unsigned int data_idx;
// 					if (readHeader (file_name, mesh.cloud, origin, orientation, file_version, data_type, data_idx, offset))
// 					{
// 						PCL_ERROR ("[pcl::OBJReader::read] Problem reading header!\n");
// 						return (-1);
// 					}
// 
// 					std::ifstream fs;
// 					fs.open (file_name.c_str (), std::ios::binary);
// 					if (!fs.is_open () || fs.fail ())
// 					{
// 						PCL_ERROR ("[pcl::OBJReader::readHeader] Could not open file '%s'! Error : %s\n",
// 							file_name.c_str (), strerror(errno));
// 						fs.close ();
// 						return (-1);
// 					}
// 
// 					// Seek at the given offset
// 					fs.seekg (data_idx, std::ios::beg);
// 
// 					// Get normal_x and rgba fields indices
// 					int normal_x_field = -1;
// 					// std::size_t rgba_field = 0;
// 					for (std::size_t i = 0; i < mesh.cloud.fields.size (); ++i)
// 						if (mesh.cloud.fields[i].name == "normal_x")
// 						{
// 							normal_x_field = i;
// 							break;
// 						}
// 
// 						std::size_t v_idx = 0;
// 						std::size_t vn_idx = 0;
// 						std::size_t vt_idx = 0;
// 						std::size_t f_idx = 0;
// 						std::string line;
// 						std::vector<std::string> st;
// 						std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > coordinates;
// 						try
// 						{
// 							while (!fs.eof ())
// 							{
// 								getline (fs, line);
// 								// Ignore empty lines
// 								if (line == "")
// 									continue;
// 
// 								// Tokenize the line
// 								std::stringstream sstream (line);
// 								sstream.imbue (std::locale::classic ());
// 								line = sstream.str ();
// 								boost::trim (line);
// 								boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
// 
// 								// Ignore comments
// 								if (st[0] == "#")
// 									continue;
// 								// Vertex
// 								if (st[0] == "v")
// 								{
// 									try
// 									{
// 										for (int i = 1, f = 0; i < 4; ++i, ++f)
// 										{
// 											float value = boost::lexical_cast<float> (st[i]);
// 											memcpy (&mesh.cloud.data[v_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
// 												&value,
// 												sizeof (float));
// 										}
// 										++v_idx;
// 									}
// 									catch (const boost::bad_lexical_cast &e)
// 									{
// 										PCL_ERROR ("Unable to convert %s to vertex coordinates!", line.c_str ());
// 										return (-1);
// 									}
// 									continue;
// 								}
// 								// Vertex normal
// 								if (st[0] == "vn")
// 								{
// 									try
// 									{
// 										for (int i = 1, f = normal_x_field; i < 4; ++i, ++f)
// 										{
// 											float value = boost::lexical_cast<float> (st[i]);
// 											memcpy (&mesh.cloud.data[vn_idx * mesh.cloud.point_step + mesh.cloud.fields[f].offset],
// 												&value,
// 												sizeof (float));
// 										}
// 										++vn_idx;
// 									}
// 									catch (const boost::bad_lexical_cast &e)
// 									{
// 										PCL_ERROR ("Unable to convert line %s to vertex normal!", line.c_str ());
// 										return (-1);
// 									}
// 									continue;
// 								}
// 								// Texture coordinates
// 								if (st[0] == "vt")
// 								{
// 									try
// 									{
// 										Eigen::Vector3f c (0, 0, 0);
// 										for (std::size_t i = 1; i < st.size (); ++i)
// 											c[i-1] = boost::lexical_cast<float> (st[i]);
// 										if (c[2] == 0)
// 											coordinates.push_back (Eigen::Vector2f (c[0], c[1]));
// 										else
// 											coordinates.push_back (Eigen::Vector2f (c[0]/c[2], c[1]/c[2]));
// 										++vt_idx;
// 									}
// 									catch (const boost::bad_lexical_cast &e)
// 									{
// 										PCL_ERROR ("Unable to convert line %s to texture coordinates!", line.c_str ());
// 										return (-1);
// 									}
// 									continue;
// 								}
// 								// Material
// 								if (st[0] == "usemtl")
// 								{
// 									mesh.tex_polygons.push_back (std::vector<pcl::Vertices> ());
// 									mesh.tex_materials.push_back (pcl::TexMaterial ());
// 									for (std::size_t i = 0; i < companions_.size (); ++i)
// 									{
// 										std::vector<pcl::TexMaterial>::const_iterator mat_it = companions_[i].getMaterial (st[1]);
// 										if (mat_it != companions_[i].materials_.end ())
// 										{
// 											mesh.tex_materials.back () = *mat_it;
// 											break;
// 										}
// 									}
// 									// We didn't find the appropriate material so we create it here with name only.
// 									if (mesh.tex_materials.back ().tex_name == "")
// 										mesh.tex_materials.back ().tex_name = st[1];
// 									mesh.tex_coordinates.push_back (coordinates);
// 									coordinates.clear ();
// 									continue;
// 								}
// 								// Face
// 								if (st[0] == "f")
// 								{
// 									//We only care for vertices indices
// 									pcl::Vertices face_v; face_v.vertices.resize (st.size () - 1);
// 									for (std::size_t i = 1; i < st.size (); ++i)
// 									{
// 										int v;
// 										sscanf (st[i].c_str (), "%d", &v);
// 										v = (v < 0) ? v_idx + v : v - 1;
// 										face_v.vertices[i-1] = v;
// 									}
// 									mesh.tex_polygons.back ().push_back (face_v);
// 									++f_idx;
// 									continue;
// 								}
// 							}
// 						}
// 						catch (const char *exception)
// 						{
// 							PCL_ERROR ("[pcl::OBJReader::read] %s\n", exception);
// 							fs.close ();
// 							return (-1);
// 						}
// 
// 						double total_time = tt.toc ();
// 						PCL_DEBUG ("[pcl::OBJReader::read] Loaded %s as a TextureMesh in %g ms with %g points, %g texture materials, %g polygons.\n",
// 							file_name.c_str (), total_time,
// 							v_idx -1, mesh.tex_materials.size (), f_idx -1);
// 						fs.close ();
// 						return (0);
// 				}

				ret = 0;
			} while (0);
error0:

			return ret;
		}

	}
}