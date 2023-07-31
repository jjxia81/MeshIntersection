#include <iostream>
#include "MyTriMesh.h"


void main()
{
	std::string mesh_path = "C:\\voxelcore\\Result_afterSmooth_7.ply";
	int plane_num = 200;
	meshseg::MyTriMesh my_mesh;
	my_mesh.LoadMesh(mesh_path);
	my_mesh.generateLineSegs();
	my_mesh.calTriangleNeigbors();
	my_mesh.intersectPlanes(plane_num);
	my_mesh.calPlaneContour();
	
	std::string contour_mesh_path = "C:\\voxelcore\\Result_afterSmooth_7_contour.ply";
	my_mesh.writeContourPoints(contour_mesh_path);

	std::cout << "hello world!" << std::endl;

}