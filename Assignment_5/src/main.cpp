// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;

// read
#include <fstream>

#include <Eigen/Geometry>

int main() 
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	/**
	 * @brief Add an option to tranlate the object toward the camera, 
	 * while rotating around its barycenter. 
	 * Produce 3 gif videos, one for each rendering type used above.
	 * 
	 * @brief Implement a perspective camera and add support for multiple resolutions: 
	 * the cameras should take into account the size of the framebuffer, 
	 * properly adapting the aspect ratio to not distort the image whenever the framebuffer is resized. 
	 * To check for correctness, we recommend to render a cube in wireframe mode.
	 */
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		// input is in the 3d position of a vertix in world space; output is in screen space
		// (1) World to camera coords
		Eigen::Vector3d e = uniform.cameraPosition;
		Eigen::Vector3d g = uniform.gazeDirection;
		Eigen::Vector3d t = uniform.viewUpVector;
		Eigen::Vector3d w = -(g.normalized());
		Eigen::Vector3d u = t.cross(w).normalized();
		Eigen::Vector3d v = w.cross(u);
		Eigen::Matrix4d Mcam;
		Mcam << u(0), v(0), w(0), e(0), 
				u(1), v(1), w(1), e(1),
				u(2), v(2), w(2), e(2),
				0,       0,    0,    1;
		Mcam = Mcam.inverse().eval();
		// (2) Orthographic projection
		Eigen::Vector3d lbn(-1,-1,-1); 		// left-bottom-near corner
		Eigen::Vector3d rtf(1,1,1); 		// right-top-far corner
		Eigen::Matrix4d Morth;
		Morth << 2/(rtf[0]-lbn[0]),    				0, 				   0, (rtf[0]+lbn[0])/(-rtf[0]+lbn[0]),
								 0,	2/(rtf[1]-lbn[1]),	 			   0, (rtf[1]+lbn[1])/(-rtf[1]+lbn[1]),
								 0, 	    		0, 2/(rtf[2]-lbn[2]), (rtf[2]+lbn[2])/(-rtf[2]+lbn[2]),
								 0, 				0, 				   0, 								 1;
		Eigen::Matrix4d Mfinal = Morth * Mcam;
		Eigen::Vector4d v2 = Mfinal * va.position;
		std::cout << v2 << "\n";
		return VertexAttributes(v2[0], v2[1], v2[2], 1);
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		// (3) Viewport transformation, [-1, 1] x [-1, 1] -> [0, nx] x [0, ny]
		double nx = 2; double ny = 2; double nz = 2;
		Eigen::Matrix4d Mup;
		Mup << nx/2,    	0,     0, (nx-1)/2,
			      0, 	 ny/2, 	   0, (ny-1)/2,
				  0,    	0,	nz/2, (nz-1)/2,
				  0, 		0, 	   0,		 1;
				  
		// Wireframe: only the edges of the triangles are drawn

		// Flat Shading: 
		return FragmentAttributes(1,0,0);

		// Per-Vertex Shading: the normals are specified on the vertices of the mesh, 
		// the color is computed for each vertex, and then interpolated in the interior of the triangle
		// you should draw the wireframe (using rasterize_lines)
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		// transparent color

		// original color
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// One triangle in the center of the screen
	vector<VertexAttributes> vertices;
	uniform.cameraPosition = Eigen::Vector3d(0,0,2);    // The camera locate at post 
	uniform.gazeDirection = Eigen::Vector3d(0,0,-1);  	// The camera points in the direction -z
	uniform.viewUpVector = Eigen::Vector3d(0,1,0);
	
	//
	// rasterize_lines();

	// Todo: extend the code to load the scene used in HW4
	// open .off files
	Eigen::MatrixXd vertices2; 	// n x 3 matrix (n points)
	Eigen::MatrixXi facets;   	// m x 3 matrix (m triangles)
	const std::string mesh_filename = "../data/triangle.off";
	std::ifstream in(mesh_filename);
	std::string token;
	in >> token; 
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	vertices2.resize(nv, 3);
    facets.resize(nf, 3);
	for (int i = 0; i < nv; ++i)
	{
		double a; double b; double c;
		in >> a >> b >> c;
		vertices2(i, 0) = a;
		vertices2(i, 1) = b;
		vertices2(i, 2) = c;
	}
	for (int i = 0; i < nf; ++i)
	{
		int s; // s == 3 for triag mesh
		int a; int b; int c;
		// load the triangle into vertices
		in >> s >> a >> b >> c;
		Eigen::Vector3d vertexA = vertices2.row(a);
		Eigen::Vector3d vertexB = vertices2.row(b);
		Eigen::Vector3d vertexC = vertices2.row(c);
		vertices.push_back(VertexAttributes(vertexA[0], vertexA[1], vertexA[2]));
		vertices.push_back(VertexAttributes(vertexB[0], vertexB[1], vertexB[2]));
		vertices.push_back(VertexAttributes(vertexC[0], vertexC[1], vertexC[2]));
	}

	// rasterization, every three points form a triangle
	printf("Start rasterizing\n");
	rasterize_triangles(program,uniform,vertices,frameBuffer);
	printf("Done rasterizing, start writing image\n");
	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	
	return 0;
}
