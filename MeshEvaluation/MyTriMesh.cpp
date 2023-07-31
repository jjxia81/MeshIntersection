#include <cmath>
#include <stack>
#include "MyTriMesh.h"


namespace meshseg {

	//LineSeg::plane_intersects();
	std::vector<LineIntersectMap> LineSeg::plane_intersects;

	LineSeg::LineSeg()
	{

	}
	LineSeg::LineSeg(size_t va_id, size_t vb_id, const Point3& va, const Point3& vb)
	{
		if (PointOrder(va, vb))
		{
			this->va_id = va_id;
			this->vb_id = vb_id;
			this->pa = va;
			this->pb = vb;
		}
		else 
		{
			this->va_id = vb_id;
			this->vb_id = va_id;
			this->pa = vb;
			this->pb = va;
		}
		this->s_key = this->generateLineSegKey(va_id, vb_id, va, vb);
	}
	LineSeg::~LineSeg()
	{

	}

	bool LineSeg::intersectPlane(float plane_z, IntersectRes& intersect_res)
	{
		if (this->pa[2] == this->pb[2]) return false;
		if (plane_z >= this->pa[2] && plane_z <= this->pb[2])
		{
			if (std::fabsf(plane_z - this->pa[2]) < 1e-2)
			{
				intersect_res.intersect_at_boarder = false;
				intersect_res.intersectP = this->pa;
				intersect_res.v_id = this->va_id;
				return true;
			}
			if (std::fabsf(plane_z - this->pb[2]) < 1e-2)
			{
				intersect_res.intersect_at_boarder = false;
				intersect_res.intersectP = this->pb;
				intersect_res.v_id = this->vb_id;
				return true;
			} 
			auto ab = this->pb - this->pa;
			auto dir =float( double (plane_z - this->pa[2]) / double(this->pb[2] - this->pa[2])) * ab;
			intersect_res.intersectP = this->pa + dir;
			intersect_res.intersect_at_boarder = true;
			return true;
		}
		return false;
	}

	//bool LineSeg::intersectPlane(float plane_z, Point3& intersectPoint, bool intersect_at_boarder)
	//{
	//	if (this->pa[2] == this->pb[2]) return false;
	//	if (plane_z >= this->pa[2] && plane_z <= this->pb[2])
	//	{
	//		auto ab = this->pb - this->pa;
	//		auto dir = (plane_z - this->pa[2]) / (this->pb[2] - this->pa[2]) * ab;
	//		intersectPoint = this->pa + dir;
	//		return true;
	//	}
	//	return false;
	//}

	std::string LineSeg::generateLineSegKey(size_t va_id, size_t vb_id, const Point3& va, const Point3& vb)
	{
		std::string key;
		if (PointOrder(va, vb))
		{
			key = std::to_string(va_id) + "_" + std::to_string(vb_id);
		}
		else {
			key = std::to_string(vb_id) + "_" + std::to_string(va_id);
		}
		return key;
	}

	bool LineSeg::PointOrder(const Point3& pa, const Point3& pb)
	{
		if (pa[2] < pb[2]) return true;
		if (pa[2] > pb[2]) return false;

		if (pa[0] < pb[0]) return true;
		if (pa[0] > pb[0]) return false;

		if (pa[1] < pb[1]) return true;
		if (pa[1] > pb[1]) return false;
		return true;
	}


	void LineSeg::addFaceId(size_t face_id)
	{
		this->face_ids.insert(face_id);
	}

	void LineSeg::setMaxPlaneZ(size_t plane_num)
	{
		this->plane_num = plane_num; 
	}

	void LineSeg::calLinePlaneIntersection()
	{
		float min_plane_z = this->pa[2];
		float max_plane_z = this->pb[2];
		int start_z = (int)std::ceil(min_plane_z);
		start_z = start_z > 0 ? start_z : 0;
		int end_z = (int)max_plane_z < plane_num - 1 ? (int)max_plane_z : plane_num - 1;

		//std::cout << " start z : " << start_z << "  end z : " << end_z << std::endl;
		for (int i = start_z; i <= end_z; ++i)
		{
			IntersectRes new_inter_res;
			new_inter_res.line_key = this->s_key;
			if (intersectPlane(float(i), new_inter_res))
			{
				/*if (std::fabsf(new_inter_res.intersectP[2] - float(i)) > 1e-3)
					continue;*/
				plane_intersects[i][this->s_key] = new_inter_res;
			}
		}
		//std::cout << " end cal " << std::endl;
	}

	MyTriangle::MyTriangle()
	{

	}
	MyTriangle::~MyTriangle()
	{

	}

	void MyTriangle::assignLines(size_t a_id, size_t b_id, size_t c_id,
		const Point3& pa, const Point3& pb, const Point3& pc)
	{
		this->key_ab = LineSeg::generateLineSegKey(a_id, b_id, pa, pb);
		this->key_bc = LineSeg::generateLineSegKey(b_id, c_id, pb, pc);
		this->key_ac = LineSeg::generateLineSegKey(a_id, c_id, pa, pc);

		this->lines_keys_.push_back(this->key_ab);
		this->lines_keys_.push_back(this->key_bc);
		this->lines_keys_.push_back(this->key_ac);

		this->tri_face[0] = a_id;
		this->tri_face[1] = b_id;
		this->tri_face[2] = c_id;
	}

	bool MyTriangle::generateMeshIntersectLine(LineIntersectMap& intersect_line_map)
	{
		intersect_line_keys_.clear();
		inter_res_at_v_id_.clear();
		this->intersect_res_.clear();
		std::set<size_t> inter_v_ids;
	
		std::vector<Point3> intersect_points;
		for (auto l_key : this->lines_keys_)
		{
			if (intersect_line_map.find(l_key) != intersect_line_map.end())
			{
				auto inter_res = intersect_line_map[l_key];
				if (inter_res.intersect_at_boarder)
				{
					this->intersect_res_.push_back(inter_res);
				}
				else {
					
					if (inter_v_ids.find(inter_res.v_id) == inter_v_ids.end())
					{
						inter_v_ids.insert(inter_res.v_id);
						this->intersect_res_.push_back(inter_res);
					}
				}
			}
		}
		/*if (this->intersect_res_.size() > 0)
		{
			std::cout << "intersect_res_ num : " << intersect_res_.size() << std::endl;
		}*/
		if (intersect_res_.size() < 2) return false;
		return true;
		
		
	}

	MyTriangle::MyTriangle(size_t a_id, size_t b_id, size_t c_id,
		const Point3& pa, const Point3& pb, const Point3& pc)
	{
		this->assignLines(a_id, b_id, c_id, pa, pb, pc);
	}

	void MyTriangle::calNeighborFaceIds(std::unordered_map<std::string, LineSeg>& line_seg_map)
	{
		for (auto l_key : this->lines_keys_)
		{
			for (auto f_id : line_seg_map[l_key].face_ids)
			{
				this->neighbor_face_ids_.insert(f_id);
			}
		}
	}

	void MyTriangle::intersectPlane()
	{

	}

	MyTriMesh::MyTriMesh()
	{

	}
	MyTriMesh::~MyTriMesh()
	{

	}
	void MyTriMesh::LoadMesh(const std::string& mesh_path)
	{
		auto in_mesh = trimesh::TriMesh::read(mesh_path);
		this->vertices = in_mesh->vertices;
		this->faces = in_mesh->faces;
	}

	void MyTriMesh::generateLineSegs()
	{
		const auto& faces = this->faces;
		const auto& vertices = this->vertices;
		this->faceID_lineSeg_vec_.resize(faces.size());

		for (size_t f_id = 0; f_id < faces.size(); ++f_id)
		{
			auto f = faces[f_id];

			MyTriangle new_tri(f[0], f[1], f[2], vertices[f[0]], vertices[f[1]], vertices[f[2]]);
			this->mesh_triangles_.push_back(new_tri);

			for (size_t id = 0; id < 3; ++id)
			{
				size_t v_id = f[id];
				vid_to_face_ids_map_[v_id].insert(f_id);
			}
			

			for (size_t i = 0; i < 3; ++i)
			{
				size_t va_id = f[i];
				size_t vb_id = f[(i + 1) % 3];
				auto& pa = vertices[va_id];
				auto& pb = vertices[vb_id];

				LineSeg l_s(va_id, vb_id, pa, pb);
				l_s.addFaceId(f_id);
				if (this->line_seg_map_.find(l_s.s_key) == this->line_seg_map_.end())
				{
					this->line_seg_map_[l_s.s_key] = l_s;
				}
				else {
					this->line_seg_map_[l_s.s_key].addFaceId(f_id);
				}
				faceID_lineSeg_vec_[f_id].push_back(l_s.s_key);
			}
		}
	}

	void MyTriMesh::intersectPlanes(size_t plane_num)
	{
		
		LineSeg::plane_intersects.resize(plane_num);
		for (auto& line_ele : this->line_seg_map_)
		{
			line_ele.second.setMaxPlaneZ(plane_num);
			line_ele.second.calLinePlaneIntersection();
		}
	}

	void MyTriMesh::calPlaneContour()
	{
		std::string out_inter_res = "D:\\data\\output\\contours\\inter_res.txt";
		std::fstream s_file{ out_inter_res, s_file.out};

		for (size_t p_id = 0; p_id < LineSeg::plane_intersects.size(); ++p_id)
		{
			auto& cur_intersect_res = LineSeg::plane_intersects[p_id];
			s_file << "p_id" << p_id << std::endl;
			for (auto& inter_res : cur_intersect_res)
			{
				s_file << inter_res.first << " " << inter_res.second.intersectP[0] << " "
					<< inter_res.second.intersectP[1] << " " << inter_res.second.intersectP[2] << " "
					<< inter_res.second.intersect_at_boarder << " " << std::endl;
			}
		}


		int contour_num = LineSeg::plane_intersects.size();
		std::vector<std::vector<Point3>> contours;

		for (size_t p_id = 0; p_id < contour_num; ++p_id)
		{
			std::vector<Point3> contour_points;
			auto& intersects = LineSeg::plane_intersects[p_id];

			std::set<size_t> intersectFaceIds;
			for (auto inter : intersects)
			{
				for (auto f_id : line_seg_map_[inter.first].face_ids)
				{
					intersectFaceIds.insert(f_id);
				}
			}
			std::stack<size_t> search_face_ids;
			std::set<std::string> inter_line_keys;
			std::set<size_t> inter_at_v_ids;
			std::set<size_t> visited_face_ids;
			std::set<size_t> added_search_face_ids;
			
			for (auto cur_f_id : intersectFaceIds)
			{
				search_face_ids.push(cur_f_id);
				while (!search_face_ids.empty())
				{
					auto s_cur_f_id = search_face_ids.top();
					search_face_ids.pop();
					if (visited_face_ids.find(s_cur_f_id) != visited_face_ids.end())
					{
						continue;
					}
					visited_face_ids.insert(s_cur_f_id);
					added_search_face_ids.insert(s_cur_f_id);
					auto& cur_triangle = mesh_triangles_[s_cur_f_id];
					
					if (cur_triangle.generateMeshIntersectLine(intersects))
					{
						
						for (auto inter_res : cur_triangle.intersect_res_)
						{
							if (inter_line_keys.find(inter_res.line_key) != inter_line_keys.end())
							{
								continue;
							}
							inter_line_keys.insert(inter_res.line_key);
							
							if (inter_res.intersect_at_boarder)
							{
								contour_points.push_back(inter_res.intersectP);
							}
							else {
								if (inter_at_v_ids.find(inter_res.v_id) != inter_at_v_ids.end())
								{
									continue;
								}
								inter_at_v_ids.insert(inter_res.v_id);
								contour_points.push_back(inter_res.intersectP);
							}
						}
						//search_face_ids = std::stack<size_t>();
						
						for (size_t id = 0; id < 3; ++id)
						{
							size_t v_id = cur_triangle.tri_face[id];

							for (auto new_f_id : this->vid_to_face_ids_map_[v_id])
							{
								if (intersectFaceIds.find(new_f_id) == intersectFaceIds.end()) continue;

								if (visited_face_ids.find(new_f_id) != visited_face_ids.end())
								{
									continue;
								}
								
								search_face_ids.push(new_f_id);
							}

						}

						for (const auto& t_l_key : cur_triangle.lines_keys_)
						{
							for (auto n_f_id : line_seg_map_[t_l_key].face_ids)
							{
								if (intersectFaceIds.find(n_f_id) == intersectFaceIds.end()) continue;

								if (visited_face_ids.find(n_f_id) != visited_face_ids.end())
								{
									continue;
								}
								search_face_ids.push(n_f_id);
							}
						}

					}
					
				}
			
				
			}
			//std::vector<bool> face_visited(faces.size(), false);
			
			contours.push_back(contour_points);  
			/*if(p_id >= 11)
				break;*/
		}
		this->mesh_plane_contours_ = contours;
	}

	//void MyTriMesh::calPlaneContour()
	//{
	//	int contour_num = LineSeg::plane_intersects.size();
	//	std::vector<std::vector<Point3>> contours;
	//	for (size_t p_id = 0; p_id < contour_num; ++p_id)
	//	{
	//		std::unordered_map<std::string, bool> line_visited;
	//		std::vector<Point3> contour_points;
	//		auto& intersects = LineSeg::plane_intersects[p_id];
	//		//std::vector<bool> face_visited(faces.size(), false);
	//		std::string all_keys;
	//		for (auto inter : intersects)
	//		{
	//			auto str_key = inter.first;
	//			all_keys += " " + str_key;
	//			if (line_visited.find(str_key) != line_visited.end()) continue;
	//			line_visited[str_key] = true;
	//			contour_points.push_back(inter.second);
	//			std::cout << str_key << std::endl;
	//			auto cur_line = line_seg_map_[str_key];
	//			std::stack<size_t> faceIDs;
	//			for (auto f_id : cur_line.face_ids)
	//			{
	//				faceIDs.push(f_id);
	//			}

	//			while (!faceIDs.empty())
	//			{
	//				auto cur_f_id = faceIDs.top();
	//				faceIDs.pop();
	//				auto line_keys = faceID_lineSeg_vec_[cur_f_id];
	//				for (auto l_key : line_keys)
	//				{
	//					if (intersects.find(l_key) != intersects.end())
	//					{
	//						if (line_visited.find(l_key) == line_visited.end())
	//						{
	//							line_visited[l_key] = true;
	//							const auto& cur_p = (intersects[l_key]);
	//							if (!contour_points.empty())
	//							{
	//								auto last_point = contour_points[contour_points.size() - 1];
	//								if (last_point[0] == cur_p[0] && cur_p[1] == last_point[1])
	//								{
	//									continue;
	//								}
	//							} 
	//							contour_points.push_back((intersects[l_key]));
	//							std::cout << l_key << std::endl;
	//							for (auto f_id : line_seg_map_[l_key].face_ids)
	//							{
	//								if (cur_f_id == f_id) continue;
	//								faceIDs.push(f_id);
	//							}
	//						}
	//					}
	//				}
	//			}
	//		}
	//		std::cout << " all keys : " << all_keys << std::endl;
	//		contours.push_back(contour_points);
	//		break;
	//	}
	//	this->mesh_plane_contours_ = contours;
	//}
	

	void MyTriMesh::writeContourPoints(const std::string& out_path)
	{
		

		trimesh::TriMesh out_mesh;

		/*if (!s.is_open())
		{
			return;
		}*/
	
		/*size_t slice_step = 5;*/
		for (size_t s_id = 0; s_id < this->mesh_plane_contours_.size(); s_id ++)
		{
			std::string contour_mesh_path_obj = "D:\\data\\output\\contours\\Result_afterSmooth_contour_" + std::to_string(s_id) + ".obj";
			std::fstream s{ contour_mesh_path_obj, s.out};
			auto c_points = this->mesh_plane_contours_[s_id];
			for (auto point : c_points)
			{
				std::string new_line = "v " + std::to_string(point[0]) + " " + std::to_string(point[1]) + " " + std::to_string(point[2]);
				s << new_line << std::endl;
				out_mesh.vertices.push_back(point);
			}
			size_t v_id = 1;

			while (v_id < c_points.size())
			{
				std::string new_line = "l " + std::to_string(v_id) + " " + std::to_string(v_id + 1);
				/*size_t next_id = v_id + 1;*/
				s << new_line << std::endl;
				v_id++;
			}
			/*contour_sizes.push_back(c_points.size());*/			
		}
		size_t v_size = out_mesh.vertices.size();
		//size_t v_id = 1;

		//while (v_id < v_size)
		//{
		//	std::string new_line = "l " + std::to_string(v_id) + " " +std::to_string(v_id + 1);
		//	/*size_t next_id = v_id + 1;*/
		//	s << new_line << std::endl;
		//	v_id++;
		//}
		/*for (auto contour_size : contour_sizes)
		{
			size_t c_end = v_id + contour_size;
			size_t c_start = v_id;
			while (v_id < c_end - 1)
			{
				std::string new_line = "l " + std::to_string(v_id) + " ";
				size_t next_id = v_id + 1;
				if (v_id == c_end)
				{
					next_id = c_start;
				}
				new_line  += std::to_string(next_id);
				s << new_line << std::endl;
				v_id++;
			}
			
		}*/
		out_mesh.write(out_path);
	}

	void MyTriMesh::calTriangleNeigbors()
	{
		for (auto& triangle : this->mesh_triangles_)
		{
			triangle.calNeighborFaceIds(this->line_seg_map_);
		}
	}
}


//namespace CGAL_UTIL {
//
//	void load_mesh(Mesh& mesh, const std::string& mesh_path)
//	{
//		CGAL::IO::read_polygon_mesh(mesh_path, mesh);
//		auto face_range = mesh.faces();
//		auto f1 = face_range.begin();
//
//	}
//
//	void load_mesh(MyTriMesh* my_mesh, const std::string& mesh_path)
//	{
//		auto in_mesh_ptr = trimesh::TriMesh::read(mesh_path);
//		//	my_mesh->read()
//	}
//
//}

