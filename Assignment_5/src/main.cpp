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
		Eigen::Matrix4d MView = uniform.Mview;

		// Eigen::Matrix4d Mfinal = Eigen::MatrixXd::Identity(4,4);
		Eigen::Matrix4d Mfinal = MView * MPersp * Mcam;

		Eigen::Vector4d v2 = Mfinal * va.position;
		Eigen::Vector4d n2 = va.plane_normal;
		Eigen::Vector4d n3 (n2[0]/n2[3], n2[1]/n2[3], n2[2]/n2[3], 1);

		VertexAttributes out(v2[0]/v2[3], v2[1]/v2[3], v2[2]/v2[3], 1);
		out.plane_normal << n3;
		return out;
	};

	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{  
		// input va is in the 3d position of a vertex in CVV(pixel); output is a color
		if (uniform.render_type == "line"){
			// line color
			FragmentAttributes out(uniform.lineColor(0), uniform.lineColor(1), uniform.lineColor(2));
			out.position = va.position;
			return out;
		}
		// Flat Shading: compute the per-face normal,
		// the lighting equation need to be computed in the camera space
		// [CVV] --MorthInv/MPersp-> [Camera space]
		Eigen::Matrix4d Mcam = uniform.getMcamInv().inverse().eval();
		Eigen::Matrix4d Morth = uniform.getMorth();
		Eigen::Matrix4d MPersp = uniform.getMpersp();
		Eigen::Matrix4d MView = uniform.Mview;

		Eigen::Matrix4d MFinalInverse = (MView * MPersp * Mcam).inverse().eval();
		// in the camera space the position of the camera center is known
		Eigen::Vector3d FragPos = (MFinalInverse * va.position).segment(0,3);
		Eigen::Vector3d norm = (va.plane_normal).segment(0,3); // the plane normal stored is still in world space
		Eigen::Vector3d color(1.,1.,1.);
		Eigen::Vector3d lightPos = uniform.lightPos;
		Eigen::Vector3d lightDir = (lightPos - FragPos).normalized();
		double diff = std::max(0., norm.dot(lightDir));
		// std::cout << (MorthInv * va.position) << "\n";
		// std::cout << norm << "\n";
		double R = color(0) * diff;
		double G = color(1) * diff;
		double B = color(2) * diff;

		// [CVV] --MupInv-> [Screen space] 
		Eigen::Matrix4d Mup = uniform.getMup();
		FragmentAttributes out(R,G,B);

		out.position = Mup * va.position;
		return out;
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

	Eigen::MatrixXd vertex_normal_sum(nv, 3);
	std::vector<int> vertex_normal_count(nv);
	for (int i = 0; i < nf; ++i)
	{
		int s; int a; int b; int c;
		// load the triangle into vertices, which is a vector of VertexAttributes
		in >> s >> a >> b >> c;
		facets0(i,0) = a; facets0(i,1) = b; facets0(i,2) = c;
		// compute the per-face normal that attributes to the shader program.
		Eigen::Vector3d posA = vertices0.row(a);
		Eigen::Vector3d posB = vertices0.row(b);
		Eigen::Vector3d posC = vertices0.row(c);
		Eigen::Vector3d AB = (posB - posA);
		Eigen::Vector3d CB = (posB - posC);
		Eigen::Vector3d faceNorm = AB.cross(CB).normalized();
		/**
		 * @brief flat shading (1): 
		 * need to send per-face normal attributes to the shader program.
		 * build a new pair (V', F'), where every vertex from V' appears only once in F'.
		 */
		VertexAttributes vertexA = VertexAttributes::buildWithFaceNormal(posA, faceNorm);
		VertexAttributes vertexB = VertexAttributes::buildWithFaceNormal(posB, faceNorm);
		VertexAttributes vertexC = VertexAttributes::buildWithFaceNormal(posC, faceNorm);
		vertices.push_back(vertexA);
		vertices.push_back(vertexB);
		vertices.push_back(vertexC);
		/**
		 * @brief Per-Vertex Shading (1): 
		 * compute the per-face normals, and then average them on the neighboring vertices.
		 */
		// vertex_normal_sum.row(a) += faceNorm;
		// vertex_normal_count[a] += 1;
		// vertex_normal_sum.row(b) += faceNorm;
		// vertex_normal_count[b] += 1;
		// vertex_normal_sum.row(c) += faceNorm;
		// vertex_normal_count[c] += 1;
	}
	/**
	 * @brief Per-Vertex Shading (2): 
	 * at each vertex take the average of all facets that contains it
	 */
	// Eigen::MatrixXd vertex_Norm(nv, 3);
	// for (int i = 0; i < nv; ++i) 
	// 	vertex_Norm.row(i) = vertex_normal_sum.row(i) / vertex_normal_count[i];
	// for (int i = 0; i < nf; ++i)
	// {
	// 	int a = facets0(i,0); int b = facets0(i,1); int c = facets0(i,2);
	// 	Eigen::Vector3d posA = vertices0.row(a);
	// 	Eigen::Vector3d posB = vertices0.row(b);
	// 	Eigen::Vector3d posC = vertices0.row(c);
	// 	VertexAttributes vertexA = VertexAttributes::buildWithFaceNormal(posA, vertex_Norm.row(a));
	// 	VertexAttributes vertexB = VertexAttributes::buildWithFaceNormal(posB, vertex_Norm.row(b));
	// 	VertexAttributes vertexC = VertexAttributes::buildWithFaceNormal(posC, vertex_Norm.row(c));
	// 	vertices.push_back(vertexA);
	// 	vertices.push_back(vertexB);
	// 	vertices.push_back(vertexC);
	// }

	/**
	 * @brief camera setting
	 * set the size of the view volume box to scale the bunny by a factor of 0.2, 
	 * since max(abs(vertex)) ~ 0.2
	 */
	double scale = 0.2;
	uniform.cameraPosition = Eigen::Vector3d(0,0.1,1);    // The camera locate at post 
	uniform.gazeDirection = Eigen::Vector3d(0,0,-1);  	// The camera points in the direction -z
	uniform.viewUpVector = Eigen::Vector3d(0,1,0);		// positive y-direction is the up direction
	uniform.lbn = Eigen::Vector3d(-scale,-scale, -2);
	uniform.rtf = Eigen::Vector3d( scale, scale,  2);
	uniform.lightPos = Eigen::Vector3d(0, 0, -1);
	// aspect ratio resolution setting
	double aspect_ratio = double(frameBuffer.cols())/double(frameBuffer.rows());
	uniform.Mview <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	if (aspect_ratio < 1)
		uniform.Mview(0,0) = aspect_ratio;
	else
		uniform.Mview(1,1) = 1/aspect_ratio;

	
	/**
	 * @brief build the vector for WireFrame:
	 * use rasterize_lines to only render the edges
	 */
	vector<VertexAttributes> vertices_line;
	for (int i = 0; i < nf; ++i)
	{
		// load the triangle into vertices, which is a vector of VertexAttributes
		Eigen::Vector3d posA = vertices0.row(facets0(i,0));
		Eigen::Vector3d posB = vertices0.row(facets0(i,1));
		Eigen::Vector3d posC = vertices0.row(facets0(i,2));
		VertexAttributes vertexA(posA[0], posA[1], posA[2]);
		VertexAttributes vertexB(posB[0], posB[1], posB[2]);
		VertexAttributes vertexC(posC[0], posC[1], posC[2]);
		// line seg 1
		vertices_line.push_back(vertexA);
		vertices_line.push_back(vertexB);
		// line seg 2
		vertices_line.push_back(vertexB);
		vertices_line.push_back(vertexC);
		// line seg 3
		vertices_line.push_back(vertexC);
		vertices_line.push_back(vertexA);
	}

	/**
	 * @brief Ex.1,2 Draw the vertices in png file
	 * 
	 */
	uniform.render_type = "triangle";
	// std::printf("Start rasterizing triangles\n");
	// rasterize_triangles(program,uniform,vertices,frameBuffer);
	// std::printf("Done rasterizing triangles\n");
	
	/**
	 * @brief Ex.1,2 Draw the vertices as lines
	 */
	uniform.render_type = "line";
	// uniform.lineColor = Eigen::Vector3d(1,1,1);
	// std::printf("Start rasterizing lines\n");
	// rasterize_lines(program,uniform,vertices_line,1,frameBuffer);
	// std::printf("Done rasterizing lines, start writing image\n");

	// vector<uint8_t> image;
	// framebuffer_to_uint8(frameBuffer,image);
	// stbi_write_png("triangle.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
	

	/**
	 * @brief Ex.3 Draw the vertices in Gif
	 * 
	 * 
	 */
	const char * fileName = "out.gif";
	vector<uint8_t> image;
	int delay = 25; double PI = 3.141592653589793;
	GifWriter g;
	GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);
	for (double i=0;i<1;i+=0.05)
	{
		frameBuffer.setConstant(FrameBufferAttributes());

		/**
		 * @brief tranlate the object toward the camera, that is to move the camera closer in z-coordinate
		 * and rotate around its barycenter, 
		 * that we need to modify x,y coords of cameraPosition, and the viewUpVector direction
		 */
		double p = i*2*PI;
		Eigen::Matrix3d rotation;
		rotation << std::cos(p),	0,  std::sin(p),
					0, 				1, 			  0,
				   -std::sin(p),    0, 	std::cos(p);

		uniform.cameraPosition = rotation * Eigen::Vector3d(0,0.1,1-i);
		uniform.gazeDirection = rotation * Eigen::Vector3d(0,0,-1); // The camera points in the direction -z
		uniform.lightPos = rotation * Eigen::Vector3d(0, 0, -1); // also rotate the light source
		uniform.viewUpVector = rotation * Eigen::Vector3d(0,1,0);

		/** @brief render the triangle */
		uniform.render_type = "triangle";
		rasterize_triangles(program,uniform,vertices,frameBuffer);

		/** @brief render the lines */
		// uniform.render_type = "line";
		// uniform.lineColor = Eigen::Vector3d(1,1,1);
		// rasterize_lines(program,uniform,vertices_line,0.75,frameBuffer);

		framebuffer_to_uint8(frameBuffer,image);
		GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
	}
	return 0;
}
