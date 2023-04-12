// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

// Utilities for the Assignment
#include "./utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

// Q1.1 generic sphere
void raytrace_sphere() {
	
	std::cout << "Ray tracer: one sphere with #Orthographic# projection and Generic intersection determinant" << std::endl;

	const std::string filename("sphere_orthographic_general_intersec.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	// the sphere is now placed "deep" inside the screen, so the light source should light up most of its front side.
	const double sphere_radius = 0.4;
	Vector3d sphere_center(0.2, 0.2, -10);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			// Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			// const double sphere_radius = 0.9;

			// if (ray_on_xy.norm() < sphere_radius) {
			// 	// The ray hit the sphere, compute the exact intersection point
			// 	Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

			// 	// Compute normal at the intersection point
			// 	Vector3d ray_normal = ray_intersection.normalized();

			// 	// Simple diffuse model
			// 	C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

			// 	// Clamp to zero
			// 	C(i,j) = std::max(C(i,j),0.);

			// 	// Disable the alpha mask for this pixel
			// 	A(i,j) = 1;
			// }

			// 1. Replace the ad-hoc ray sphere intersection algorithm with the generic one derived in class.
			double b = ray_direction.dot(ray_origin - sphere_center) * 2;
			double a = ray_direction.squaredNorm();
			double c = (ray_origin - sphere_center).squaredNorm() - sphere_radius * sphere_radius;
			if (b * b - 4 * a * c > 0) {
				// the positive solution with smaller norm is the first time where ray intersects the sphere
				double t = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;
				Vector3d ray_intersection = ray_origin + ray_direction * t;

				Vector3d ray_normal = (ray_intersection-sphere_center).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;

			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

// Q1.2 parallelogram using two triangles
void raytrace_parallelogram() {
	std::cout << "Ray tracer: one parallelogram with #Orthographic# projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(400,400); // Store the color
	MatrixXd A = MatrixXd::Zero(400,400); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.75,-0.75,0); // point a
	Vector3d pgram_u(0.25,-0.25,-0.2); // point b
	Vector3d pgram_v(-0.25,0.25,-0.2); // point c
	// compute d: the top right corner which is diagonal of a. counterclock wise direction: b, d, c
	Vector3d pgram_d = pgram_u + pgram_v - pgram_origin; 
	Vector3d ba = pgram_u - pgram_origin;
	Vector3d ca = pgram_v - pgram_origin;
	Vector3d da = pgram_d - pgram_origin;

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// TODO: Check if the ray intersects with the parallelogram
			// triangle a-b-d
			Matrix3d AA;
			AA << -ba(0), -da(0), ray_direction(0), 
				  -ba(1), -da(1), ray_direction(1), 
				  -ba(2), -da(2), ray_direction(2);
			// triangle a-d-c
			Matrix3d AB;
			AB << -ca(0), -da(0), ray_direction(0), 
				  -ca(1), -da(1), ray_direction(1), 
				  -ca(2), -da(2), ray_direction(2);

			if (AA.determinant() == 0 || AB.determinant() == 0) continue;
			// if intersect with triangle a-b-d
			Vector3d uvt = AA.colPivHouseholderQr().solve(pgram_origin - ray_origin);
			if (uvt(2) > 0 && uvt(0) > 0 && uvt(1) > 0 && uvt(0)+uvt(1)<1) {

				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * uvt(2);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = (ba.cross(da)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
			// if intersect with triangle a-d-c
			Vector3d uvt2 = AB.colPivHouseholderQr().solve(pgram_origin - ray_origin);
			if (uvt2(2) > 0 && uvt2(0) > 0 && uvt2(1) > 0 && uvt2(0)+uvt2(1)<1) {

				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * uvt2(2);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = (da.cross(ca)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// Q1.3 perspective vs orthographic
void raytrace_perspective() {
	std::cout << "Ray tracer: one parallelogram with #Perspective# projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(400,400); // Store the color
	MatrixXd A = MatrixXd::Zero(400,400); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.75,-0.75,0); // point a
	Vector3d pgram_u(0.25,-0.25,-0.2); // point b
	Vector3d pgram_v(-0.25,0.25,-0.2); // point c
	// d: diagonal of a. counterclock wise direction: b, d, c
	Vector3d pgram_d = pgram_u + pgram_v - pgram_origin; 
	Vector3d ba = pgram_u - pgram_origin;
	Vector3d ca = pgram_v - pgram_origin;
	Vector3d da = pgram_d - pgram_origin;

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			// Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			// Vector3d ray_direction = RowVector3d(0,0,-1);
			Vector3d ray_origin = origin;
			Vector3d ray_direction = double(i)*x_displacement + double(j)*y_displacement - Vector3d(0,0,origin(2));

			// Check if the ray intersects with the parallelogram
			Matrix3d AA;
			AA << -ba(0), -da(0), ray_direction(0), 
				  -ba(1), -da(1), ray_direction(1), 
				  -ba(2), -da(2), ray_direction(2);
			Matrix3d AB;
			AB << -ca(0), -da(0), ray_direction(0), 
				  -ca(1), -da(1), ray_direction(1), 
				  -ca(2), -da(2), ray_direction(2);

			if (AA.determinant() == 0 || AB.determinant() == 0) continue;
			// triangle a-b-d
			Vector3d uvt = AA.colPivHouseholderQr().solve(pgram_origin - ray_origin);
			if (uvt(2) > 0 && uvt(0) > 0 && uvt(1) > 0 && uvt(0)+uvt(1)<1) {

				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * uvt(2);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = (ba.cross(da)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
			// triangle a-d-c
			Vector3d uvt2 = AB.colPivHouseholderQr().solve(pgram_origin - ray_origin);
			if (uvt2(2) > 0 && uvt2(0) > 0 && uvt2(1) > 0 && uvt2(0)+uvt2(1)<1) {

				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * uvt2(2);

				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = (da.cross(ca)).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}
	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// Q1.3
void raytrace_perspective_two_spheres() {
	std::cout << "Ray tracer: one sphere with #Perspective# projection" << std::endl;

	const std::string filename("sphere_perspective_general_intersec.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective
	Vector3d origin(-1,1,2);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,2);

	// this sphere's center has same xy coords as the camera, so there is no distortion
	// we cannot see the full sphere as the camera is centered at the top left corner, but it is clear that there is no distortion
	const double sphere_radius = 0.4;
	Vector3d sphere_center(-1, 1, 0);
	
	// this sphere's center has different xy coords as the camera, so there will be distortion
	const double sphere_radius2 = 0.4;
	Vector3d sphere_center2(0, 0, 0);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin;
			Vector3d ray_direction = double(i)*x_displacement + double(j)*y_displacement - Vector3d(0,0,origin(2));

			// test if it intersects the first ball
			double b = ray_direction.dot(ray_origin - sphere_center) * 2;
			double a = ray_direction.squaredNorm();
			double c = (ray_origin - sphere_center).squaredNorm() - sphere_radius * sphere_radius;
			if (b * b - 4 * a * c > 0) {
				// the positive solution with smaller norm is the first time where ray intersects the sphere
			 	double t1 = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;
				
				Vector3d ray_intersection = ray_origin + ray_direction * t1;

				Vector3d ray_normal = (ray_intersection-sphere_center2).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}

			// test if it intersects the second ball, 
			// we should compare t1 and t2 to see which ball the vision ray hit first, but they did not overlay each other so its fine here
			double b2 = ray_direction.dot(ray_origin - sphere_center2) * 2;
			double a2 = ray_direction.squaredNorm();
			double c2 = (ray_origin - sphere_center2).squaredNorm() - sphere_radius2 * sphere_radius2;
			if (b2 * b2 - 4 * a2 * c2 > 0) {
				// the positive solution with smaller norm is the first time where ray intersects the sphere
				double t2 = (-b2 - sqrt(b2 * b2 - 4 * a2 * c2)) / 2 / a2;
				Vector3d ray_intersection = ray_origin + ray_direction * t2;

				Vector3d ray_normal = (ray_intersection-sphere_center2).normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}
	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// Q2.1
void raytrace_shading(){
	std::cout << "Ray tracer: one sphere with ambient, specular, and diffuse shading" << std::endl;

	const std::string filename("shading_grey_scale.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	// sphere center and radius
	const double sphere_radius = 0.4;
	Vector3d sphere_center(0.2, 0.2, -10);

	// coeffs for greyscale panel
	double ambient = 0.25;
	double diffuse_coeff = 0.2;
	double specular_coeff = 0.9;
	double phong_expo = 12;

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			double b = ray_direction.dot(ray_origin - sphere_center) * 2;
			double a = ray_direction.squaredNorm();
			double c = (ray_origin - sphere_center).squaredNorm() - sphere_radius * sphere_radius;
			if (b * b - 4 * a * c > 0) {
				// take the positive solution with smallest absolute value as the time when ray first hit the sphere
				double t = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;
				
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * t;

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection-sphere_center).normalized();

				// add diffuse shading
				double diffuse_shading = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				diffuse_shading = std::max(0., diffuse_shading) * diffuse_coeff;
				
				// add specular shading
				double specular_shading = (light_position-ray_intersection + ray_origin-ray_intersection).normalized().transpose() * ray_normal;
				specular_shading = pow(std::max(0., specular_shading), phong_expo) * specular_coeff;

				// sum up shading effects to the output
				C(i,j) = ambient + diffuse_shading + specular_shading;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

// Q2.2
void raytrace_colored_shading(){
	std::cout << "Ray tracer: one sphere with RGB ambient, specular, and diffuse shading" << std::endl;

	const std::string filename("shading_colored.png");
	// MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd CR = MatrixXd::Zero(800,800); // Store the R panel
	MatrixXd CG = MatrixXd::Zero(800,800); // Store the G panel
	MatrixXd CB = MatrixXd::Zero(800,800); // Store the B panel
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/CR.cols(),0,0);
	Vector3d y_displacement(0,-2.0/CR.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	// sphere center and radius

	// coeffs for RGB panel -- this gives you a small golden ball far away from the light source
	const double sphere_radius = 0.4;
	Vector3d sphere_center(0.2, 0.2, -4);
	double ambientR = 0.25, ambientG = 0.15, ambientB = 0.025;
	double diffuseR = 0.5, diffuseG = 0.3, diffuseB = 0.05;
	double specularR = 1, specularG = 0.6, specularB = 0.1;
	double phong_expo = 12;

	// coeffs for RGB panel -- this gives you a large red ball close to the ligh source
	// const double sphere_radius = 0.5;
	// Vector3d sphere_center(-0.2, -0.2, 0);
	// double ambientR = 0.025, ambientG = 0, ambientB = 0;
	// double diffuseR = 0.5, diffuseG = 0, diffuseB = 0;
	// double specularR = 1, specularG = 0.8, specularB = 0.8;
	// double phong_expo = 6;

	for (unsigned i=0; i < CR.cols(); ++i) {
		for (unsigned j=0; j < CR.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			double b = ray_direction.dot(ray_origin - sphere_center) * 2;
			double a = ray_direction.squaredNorm();
			double c = (ray_origin - sphere_center).squaredNorm() - sphere_radius * sphere_radius;
			if (b * b - 4 * a * c > 0) {

				// take the positive solution with smallest absolute value as the time when ray first hit the sphere
				double t = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;
				
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection = ray_origin + ray_direction * t;

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection-sphere_center).normalized();

				// add diffuse shading
				double diffuse_shading = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				diffuse_shading = std::max(0., diffuse_shading);
				
				// add specular shading
				double specular_shading = (light_position-ray_intersection + ray_origin-ray_intersection).normalized().transpose() * ray_normal;
				specular_shading = pow(std::max(0., specular_shading), phong_expo);

				// sum up shading effects to R,G,B panels in the output
				CR(i,j) = ambientR + diffuse_shading * diffuseR + specular_shading * specularR;
				CG(i,j) = ambientG + diffuse_shading * diffuseG + specular_shading * specularG;
				CB(i,j) = ambientB + diffuse_shading * diffuseB + specular_shading * specularB;

				// Clamp to zero
				CR(i,j) = std::max(CR(i,j),0.);
				CG(i,j) = std::max(CG(i,j),0.);
				CB(i,j) = std::max(CB(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(CR,CG,CB,A,filename);
}

int main() {
	raytrace_sphere(); 				// sphere_orthographic_general_intersec.png
	raytrace_parallelogram(); 		// plane_orthographic.png
	raytrace_perspective(); 			// plane_perspective.png
	raytrace_perspective_two_spheres();	// spheres_perspective_general_intersec.png
	raytrace_shading(); 				// shading_grey_scale.png
	raytrace_colored_shading(); 		// shading_colored.png, depending on the coefficient setting

	return 0;
}
