#pragma once

#include <vector>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/iterator.h>
//#include <CGAL/point_generators_2.h>
////#include <CGAL/Simple_cartesian.h>
//#include <CGAL/AABB_tree.h>
//#include <CGAL/AABB_traits.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/AABB_face_graph_triangle_primitive.h>
//#include <list>

//#include <CGAL/Simple_cartesian.h>
////#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/iterator.h>
//#include <CGAL/Point_3.h>
//#include <CGAL/intersections.h>
//#include <CGAL/Surface_mesh.h>
#include <set>
#include <unordered_map>
#include "trimesh2/include/TriMesh.h"


namespace meshseg {

	typedef trimesh::TriMesh              TriMesh;
	typedef trimesh::point                Point3;
	typedef trimesh::TriMesh::Face        Face3;
	typedef trimesh::vec3                 Normal;
	typedef size_t                        PlaneID;
	typedef size_t                        LineID;

	struct IntersectRes
	{
		bool intersect_at_boarder = true;
		Point3 intersectP;
		size_t v_id;
		std::string line_key;
	};
	//typedef std::unordered_map <LineID, Point3> LineIntersect;
	typedef std::unordered_map <std::string, IntersectRes> LineIntersectMap;

	//typedef std::unordered_map <PlaneID, LineIntersect> PlaneIntersect;

	class MyPlane
	{

	};

	

	class LineSeg
	{
	public:
		LineSeg();
		LineSeg(size_t va_id, size_t vb_id, const Point3& va, const Point3& vb);
		~LineSeg();

		bool intersectPlane(float plane_z, IntersectRes& intersectPoint);
		void addFaceId(size_t face_id);
		static std::string generateLineSegKey(size_t va_id, size_t vb_id, const Point3& va, const Point3& vb);
		static bool PointOrder(const Point3& pa, const Point3& pb);
		void setMaxPlaneZ(size_t plane_num);
		void calLinePlaneIntersection();

	public:
		/*static size_t line_id;*/
		static std::vector<LineIntersectMap> plane_intersects;
		std::string s_key;
		
		size_t va_id;
		size_t vb_id;
		std::set<size_t> face_ids;
		bool intersect_at_boarder = true;

	private:
		size_t plane_num;
		Point3 pa;
		Point3 pb;
		//std::vector<PlaneIntersect>  plane_intersects;
	};

	class MyTriangle {
	public:
		MyTriangle();
		MyTriangle(size_t a_id, size_t b_id, size_t c_id,
			const Point3& pa, const Point3& pb, const Point3& pc);
		~MyTriangle();

		void intersectPlane();
		void assignLines(size_t a_id, size_t b_id, size_t c_id,
			const Point3& pa, const Point3& pb, const Point3& pc);

		bool generateMeshIntersectLine(LineIntersectMap& intersect_line_map);
		void calNeighborFaceIds(std::unordered_map<std::string, LineSeg>& line_seg_map);
		
	public:

		Point3 pa;
		Point3 pb;
		Point3 pc;
		Face3 tri_face;
	
		std::string key_ab;
		std::string key_bc;
		std::string key_ac;
		

		
		std::vector < std::string > lines_keys_;
		std::vector<std::string> intersect_line_keys_;
		std::set<size_t> neighbor_face_ids_;
		std::vector<size_t> inter_res_at_v_id_;
		std::vector<IntersectRes> intersect_res_;

	
	//private:

	};

	class MyTriMesh : public TriMesh {

	public:
		MyTriMesh();
		~MyTriMesh();
		void LoadMesh(const std::string& mesh_path);
		void generateLineSegs();
		void intersectPlanes(size_t plane_num);
		void calPlaneContour();
		void writeContourPoints(const std::string& out_path);
		void calTriangleNeigbors();

	public:
		//std::vector<LineSeg> line_segs;
		std::unordered_map<std::string, LineSeg> line_seg_map_;
		std::vector<std::vector<std::string>> faceID_lineSeg_vec_;
		std::vector<std::vector<Point3>> mesh_plane_contours_;
		std::vector<MyTriangle> mesh_triangles_;
		std::unordered_map<std::string, std::set<size_t>> line_face_ids_map_;
		std::unordered_map<size_t, std::set<size_t>> vid_to_face_ids_map_;
	};

}





	//// the custom triangles are stored into a vector
	//typedef std::vector<My_triangle>::const_iterator Iterator;
	//// The following primitive provides the conversion facilities between
	//// the custom triangle and point types and the CGAL ones
	//struct My_triangle_primitive {
	//public:
	//	// this is the type of data that the queries returns. For this example
	//	// we imagine that, for some reasons, we do not want to store the iterators
	//	// of the vector, but raw pointers. This is to show that the Id type
	//	// does not have to be the same as the one of the input parameter of the
	//	// constructor.
	//	typedef const My_triangle* Id;
	//	// CGAL types returned
	//	typedef K::Point_3    Point; // CGAL 3D point type
	//	typedef K::Triangle_3 Datum; // CGAL 3D triangle type
	//private:
	//	Id m_pt; // this is what the AABB tree stores internally
	//public:
	//	My_triangle_primitive();// default constructor needed
	//	// the following constructor is the one that receives the iterators from the
	//	// iterator range given as input to the AABB_tree
	//	My_triangle_primitive(Iterator it);
	//	const Id& id() const;
	//	// utility function to convert a custom
	//	// point type to CGAL point type.
	//	Point convert(const My_point* p) const;

	//	// on the fly conversion from the internal data to the CGAL types
	//	Datum datum() const;

	//	// returns a reference point which must be on the primitive
	//	Point reference_point() const;

	//};

	//typedef K::Ray_3 Ray;
	//typedef K::Point_3 Point;
	//typedef CGAL::AABB_traits<K, My_triangle_primitive> My_AABB_traits;
	//typedef CGAL::AABB_tree<My_AABB_traits> Tree;
	//typedef boost::optional< Tree::Intersection_and_primitive_id<Ray>::Type > Ray_intersection;

//}