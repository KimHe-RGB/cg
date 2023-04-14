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

#include <set>

#include <gif.h>

int main() // mine implementation
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants
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
		// input va is in the 3d position of a vertex in world space; output is in screen space
		// [World space] --Mcam-> [Camera space] --Morth--> [CVV] --Mup-> [screen space] 
		// 									     --Mpersp-> [CVV] --Mup-> [screen space]
		Eigen::Matrix4d Mcam = uniform.getMcamInv().inverse().eval();
		Eigen::Matrix4d Morth = uniform.getMorth();
		Eigen::Matrix4d MPersp = uniform.getMpersp();
		Eigen::Matrix4d Mup = uniform.getMup();

		Eigen::Matrix4d Mfinal = Mup * Morth * Mcam;
		// Eigen::Matrix4d Mfinal = Mup * MPersp * Mcam;

		Eigen::Vector4d v2 = Mfinal * va.position;
		Eigen::Vector3d v3 (v2[0]/v2[3], v2[1]/v2[3], v2[2]/v2[3]);
		Eigen::Vector4d n2 = va.plane_normal;
		Eigen::Vector3d n3 (n2[0]/n2[3], n2[1]/n2[3], n2[2]/n2[3]);

		if (uniform.render_type == "triangle") std::cout << n3 << "\n";

		return VertexAttributes::buildWithFaceNormal(v3, n3);
	};

	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{  
		// input va is in the 3d position of a vertex in CVV(pixel); output is a color
		if (uniform.render_type == "line") return FragmentAttributes(0.,0.,0.);
		// Flat Shading: compute the per-face normal,
		// the lighting equation need to be computed in the camera space
		// [screen space] --MupInv-> [CVV] --MorthInv-> [Camera space]
		Eigen::Matrix4d McamInv = uniform.getMcamInv();
		Eigen::Matrix4d MorthInv = uniform.getMorth().inverse().eval();
		// in the camera space the position of the camera center is known to be (0,0,0)
		// Eigen::Vector3d cameraPosition(0,0,0);
		Eigen::Vector3d FragPos = (MorthInv * va.position).segment(0,3);
		Eigen::Vector3d norm = (McamInv * va.plane_normal).segment(0,3);
		Eigen::Vector3d color(1.,1.,1.);
		Eigen::Vector3d lightPos(0,0,2);
		Eigen::Vector3d lightDir = (lightPos - FragPos).normalized();
		// double intensity = normal_restored.dot((lightPos - va_restored).normalized());
		double diff = std::max(0., norm.dot(lightDir));
		// std::cout << (MorthInv * va.position) << "\n";
		// std::cout << norm << "\n";
		double R = color(0) * diff;
		double G = color(1) * diff;
		double B = color(2) * diff;

		FragmentAttributes out(R,G,B);
		out.position = va.position;
		return FragmentAttributes(R,G,B);

		// Per-Vertex Shading: the normals are specified on the vertices of the mesh, 
		// the color is computed for each vertex, and then interpolated in the interior of the triangle
		// you should draw the wireframe 
		// (using rasterize_lines)

	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		// original color
		if (fa.position[2] < previous.depth)
		{
			FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
			out.depth = fa.position[2];
			return out;
		}
		else
			return previous;
	};

	
	vector<VertexAttributes> vertices;
	uniform.cameraPosition = Eigen::Vector3d(0,0,1);    // The camera locate at post 
	uniform.gazeDirection = Eigen::Vector3d(0,0,-1);  	// The camera points in the direction -z
	uniform.viewUpVector = Eigen::Vector3d(0,1,0);
	uniform.lbn = Eigen::Vector3d(-0.1,-0.1, -2);
	uniform.rtf = Eigen::Vector3d( 0.1, 0.1,  2);

	//Lights
	std::vector<Eigen::Vector3d> light_positions;
	std::vector<Eigen::Vector4d> light_colors;
	uniform.light_positions = light_positions;
	uniform.light_colors = light_colors;

	// load .off files
	Eigen::MatrixXd vertices0; 	// n x 3 matrix (n points)
	Eigen::MatrixXi facets0;   	// m x 3 matrix (m triangles)
	const std::string mesh_filename = "../data/bunny.off";
	std::ifstream in(mesh_filename);
	std::string token;
	in >> token; 
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	vertices0.resize(nv, 3);
	facets0.resize(nf, 3);
	for (int i = 0; i < nv; ++i)
	{
		double a; double b; double c;
		in >> a >> b >> c;
		vertices0(i, 0) = a; vertices0(i, 1) = b; vertices0(i, 2) = c;
	}
	Eigen::MatrixXi new_facets(nf, 3);
	Eigen::MatrixXd new_vertices(nf*3, 3);
	Eigen::MatrixXd perFaceNormals(nf, 3);
	for (int i = 0; i < nf; ++i)
	{
		int s; int a; int b; int c;
		// load the triangle into vertices, which is a vector of VertexAttributes
		in >> s >> a >> b >> c;
		facets0(i,0) = a; facets0(i,1) = b; facets0(i,2) = c;
		Eigen::Vector3d posA = vertices0.row(a);
		Eigen::Vector3d posB = vertices0.row(b);
		Eigen::Vector3d posC = vertices0.row(c);
		Eigen::Vector3d AB = (posB - posA).segment(0,3);
		Eigen::Vector3d CB = (posB - posC).segment(0,3);
		Eigen::Vector3d faceNorm = AB.cross(CB).normalized();
		VertexAttributes vertexA = VertexAttributes::buildWithFaceNormal(posA, faceNorm);
		VertexAttributes vertexB = VertexAttributes::buildWithFaceNormal(posB, faceNorm);
		VertexAttributes vertexC = VertexAttributes::buildWithFaceNormal(posC, faceNorm);
		vertices.push_back(vertexA);
		vertices.push_back(vertexB);
		vertices.push_back(vertexC);

		new_vertices.row(i*3) = posA;
		new_vertices.row(i*3+1) = posB;
		new_vertices.row(i*3+2) = posC;
		new_facets.row(i) = Eigen::Vector3i(i*3, i*3+1, i*3+2);
	}
	uniform.facets = new_facets;
	uniform.vertices = new_vertices;
	uniform.render_type = "triangle";

	// rasterization, every three points form a triangle
	printf("Start rasterizing triangles\n");
	rasterize_triangles(program,uniform,vertices,frameBuffer);
	printf("Done rasterizing triangles\n");
	
	/**
	 * @brief Draw WireFrame:
	 * use rasterize_lines to only render the edges
	 */
	vector<VertexAttributes> vertices_line;
	for (int i = 0; i < nf; ++i)
	{
		// load the triangle into vertices, which is a vector of VertexAttributes
		Eigen::Vector3d vertexA = vertices0.row(facets0(i,0));
		Eigen::Vector3d vertexB = vertices0.row(facets0(i,1));
		Eigen::Vector3d vertexC = vertices0.row(facets0(i,2));
		// line seg 1
		vertices_line.push_back(VertexAttributes(vertexA[0], vertexA[1], vertexA[2]));
		vertices_line.push_back(VertexAttributes(vertexB[0], vertexB[1], vertexB[2]));
		// line seg 2
		vertices_line.push_back(VertexAttributes(vertexB[0], vertexB[1], vertexB[2]));
		vertices_line.push_back(VertexAttributes(vertexC[0], vertexC[1], vertexC[2]));
		// line seg 3
		vertices_line.push_back(VertexAttributes(vertexC[0], vertexC[1], vertexC[2]));
		vertices_line.push_back(VertexAttributes(vertexA[0], vertexA[1], vertexA[2]));
	}
	uniform.render_type = "line";

	printf("Start rasterizing lines\n");
	rasterize_lines(program,uniform,vertices_line,0.5,frameBuffer);
	printf("Done rasterizing lines, start writing image\n");

	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	

	/**
	 * @brief Ex.3 Rotation Gif
	 * 
	 * 
	 */
	// Compute the barycenter of the window
	// const char * fileName = "triangle.gif";
	// vector<uint8_t> image;
	// int delay = 25;
	// GifWriter g;
	// GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);
	// for (double i=0;i<1;i+=0.05)
	// {
	// 	frameBuffer.setConstant(FrameBufferAttributes());
	// 	// tranlate the object toward the camera

	// 	// and rotate around its barycenter
	// 	rasterize_triangles(program,uniform,vertices,frameBuffer);
	// 	framebuffer_to_uint8(frameBuffer,image);
	// 	GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
	// }
	return 0;
}
