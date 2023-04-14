#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
class VertexAttributes
{
	public:
	VertexAttributes(double x = 0, double y = 0, double z = 0, double w = 1)
	{
		position << x,y,z,w;
	}
	static VertexAttributes buildWithFaceNormal(Eigen::Vector3d pos, Eigen::Vector3d norm) 
	{
		VertexAttributes r;
		r.position = pos.homogeneous();
		r.plane_normal = norm.homogeneous();
		return r;
	}

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const double alpha, 
        const double beta, 
        const double gamma
    ) 
    {
        VertexAttributes r;
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
		// compute the normal vector
		// Eigen::Vector3d ra = (a.position - r.position).head<3>();
		// Eigen::Vector3d rc = (c.position - r.position).head<3>();
		// Eigen::Vector3d normal = ra.cross(rc).normalized();
		// r.plane_normal = normal.homogeneous(); // 3d -> NDC
		// this normal is still in world space
		r.plane_normal = a.plane_normal; 
		// std::cout << normal.dot(ra) << "=" << normal.dot(rc) << " = 0\n";
        return r;
    }

	Eigen::Vector4d position;
	Eigen::Vector4d plane_normal;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(double r = 0, double g = 0, double b = 0, double a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4d color;
	Eigen::Vector4d position;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
		depth = 2; // this value should be between -1 and 1; 2 is further than the visible range
	}

	Eigen::Matrix<uint8_t,4,1> color;
	double depth;
	
};

class UniformAttributes
{
	public:
	UniformAttributes()
	{
		
	}
	Eigen::Vector3d cameraPosition;
	Eigen::Vector3d gazeDirection;
	Eigen::Vector3d viewUpVector;
	Eigen::Vector3d lbn;
	Eigen::Vector3d rtf;

	Eigen::MatrixXi facets;
	Eigen::MatrixXd vertices;

	std::vector<Eigen::Vector3d> light_positions;
	std::vector<Eigen::Vector4d> light_colors;
	std::string render_type;

	/**
	 * @brief Get the Morth matrix that maps from camera space to CVV, orthornormal view
	 * 
	 * @return Eigen::Matrix4d 
	 */
	Eigen::Matrix4d getMorth() const
	{
		Eigen::Matrix4d Morth;
		Morth << 2/(rtf[0]-lbn[0]),  0, 	0, (rtf[0]+lbn[0])/(-rtf[0]+lbn[0]),
				0,	2/(rtf[1]-lbn[1]),	    0, (rtf[1]+lbn[1])/(-rtf[1]+lbn[1]),
				0, 	0,    -2/(rtf[2]-lbn[2]),  (rtf[2]+lbn[2])/(rtf[2]-lbn[2]),
				0, 	0,	0,	1;
		return Morth;
	}

	/**
	 * @brief Get the Mpersp matrix maps from camera space to CVV, perspective view
	 * 
	 * @return Eigen::Matrix4d 
	 */
	Eigen::Matrix4d getMpersp() const
	{
		Eigen::Matrix4d Mp;
		Mp << lbn[2]/rtf[0],	 0,		   		0,				0,
			  0,	lbn[2]/rtf[1],		   		0,				0,
			  0,	     0,	(lbn[2]+rtf[2])/(lbn[2]-rtf[2]), 2*lbn[2]*rtf[2]/(lbn[2]-rtf[2]),
			  0,		 0,				-1,				0;
		return Mp;
	}

	/**
	 * @brief Get the Mcam matrix's inverse, that maps from camera space to world space
	 * @remark remember to take inverse in vertexShader
	 * 
	 * @return Eigen::Matrix4d 
	 */
	Eigen::Matrix4d getMcamInv() const
	{
		Eigen::Vector3d e = cameraPosition;
		Eigen::Vector3d g = gazeDirection;
		Eigen::Vector3d t = viewUpVector;
		Eigen::Vector3d w = -(g.normalized());
		Eigen::Vector3d u = t.cross(w).normalized();
		Eigen::Vector3d v = w.cross(u);
		Eigen::Matrix4d McamInv;
		McamInv << u(0), v(0), w(0), e(0), 
				u(1), v(1), w(1), e(1),
				u(2), v(2), w(2), e(2),
				0,       0,    0,    1;
		return McamInv;
	}

	/**
	 * @brief Get the Mup matrix that maps from canonical view volumn to screen space
	 * 
	 * @return Eigen::Matrix4d 
	 */
	Eigen::Matrix4d getMup() const
	{
		double nx = 2; double ny = 2; double nz = 2;
		Eigen::Matrix4d Mup;
		Mup << nx/2,    	0,     0, 	       0,
			      0, 	 ny/2, 	   0, 	   -ny/2,
				  0,    	0,	   1,          0,
				  0, 		0, 	   0,		   1;	
		return Mup;
	}
};