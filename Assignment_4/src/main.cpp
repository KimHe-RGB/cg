////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
// const std::string mesh_filename(data_dir + "dodeca.off");
// const std::string filename("raytrace_dodeca.png");
// const std::string mesh_filename(data_dir + "cube.off");
// const std::string filename("raytrace_cube.png");
// const std::string mesh_filename(data_dir + "bunny.off");
// const std::string filename("raytrace_bunny.png");
const std::string mesh_filename(data_dir + "dragon.off");
const std::string filename("raytrace_dragon.png");



//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}


int RecursiveBuildTree(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, 
std::vector<AABBTree::Node> &nodes, std::vector<int> &S, int parent)
{
    // Create a new node for the current subtree
    AABBTree::Node node;
    node.parent = parent;
    node.left = -1; node.right = -1; node.triangle = -1;
    int node_index = nodes.size();
    nodes.push_back(node);
        
    // If there is only one triangle in the set, assign it to the node and return
    if (S.size() == 1) {
        int start = S.at(0);
        nodes[node_index].triangle = start; 
        nodes[node_index].bbox = bbox_from_triangle(V.row(F(start, 0)), V.row(F(start, 1)), V.row(F(start, 2)));
        return node_index;
    }

    // Compute the bounding box of all centroids of triags in this subset
    MatrixXd sub_centroids(S.size(), 3);
    for (unsigned i = 0; i < S.size(); i++){
        sub_centroids.row(i) = centroids.row(S.at(i));
    }
    Vector3d max_xyz = sub_centroids.colwise().maxCoeff(); // [max_x, max_y, max_z]
    Vector3d min_xyz = sub_centroids.colwise().minCoeff(); // [min_x, min_y, min_z]
    Eigen::Index maxRow, maxCol;
    double max_axis = (max_xyz - min_xyz).maxCoeff(&maxRow, &maxCol);
    int maxComp = maxRow; // x=0, y=1, z=2
    double max_axis_center = (max_xyz(maxRow) + min_xyz(maxRow)) / 2;

    // 1. Split the input set of triangles into two sets S1 and S2.
    std::vector<int> S1;
    std::vector<int> S2;
    for (unsigned i = 0; i < S.size(); i++)
    {
        int triag_index = S.at(i);
        // S1 will hold the left half (rounded up), 
        if(centroids(triag_index, maxComp) <= max_axis_center){
            S1.push_back(triag_index);
        } 
        // and S2 will hold the right half (rounded down).
        else {
            S2.push_back(triag_index);
        }
    }
    // we should ensure that recursion is making progress reducing the size of the set
    assert(S1.size() > 0 && S2.size() > 0);
    
    // 2. Recursively build the subtree T1 corresponding to S1.
    // 3. Recursively build the subtree T2 corresponding to S2.
    int left_child = RecursiveBuildTree(V, F, centroids, nodes, S1, node_index);
    int right_child = RecursiveBuildTree(V, F, centroids, nodes, S2, node_index);
    assert(left_child >= 0); assert(right_child >= 0);
    nodes[node_index].left = left_child;
    nodes[node_index].right = right_child;

    // 4. Update the box of the current node by merging boxes of the root of T1 and T2.
    if (left_child == -1) {
        nodes[node_index].bbox = nodes[right_child].bbox;
    }
    else if (right_child == -1){
        nodes[node_index].bbox = nodes[left_child].bbox;
    }
    else {
        nodes[node_index].bbox = nodes[left_child].bbox.merged(nodes[right_child].bbox);
    }
    return node_index;
}


AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // DONE
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    nodes = std::vector<Node>();
    // S is the set of indexes of triangles, indexing from 0 to F.rows()
    std::vector<int> S(F.rows());
    std::iota(S.begin(), S.end(), 0);
    root = RecursiveBuildTree(V, F, centroids, nodes, S, -1);    
    std::cout << "Finish building AABB tree\n";
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // DONE
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    Matrix3d AA;
    Vector3d ba = b - a;
    Vector3d ca = c - a;
    AA << -ba(0), -ca(0), ray_direction(0), 
          -ba(1), -ca(1), ray_direction(1), 
          -ba(2), -ca(2), ray_direction(2);
    if (AA.determinant() == 0) return -1;
	Vector3d uvt = AA.colPivHouseholderQr().solve(a - ray_origin);
    if (uvt(2) > 0 && uvt(0) > 0 && uvt(1) > 0 && uvt(0)+uvt(1)<1) {
        p = ray_origin + ray_direction * uvt(2);
        N = (ba.cross(ca)).normalized();
        
        // return intersection time
        return uvt(2);
    }

    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // DONE
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.

    int BLF = box.BottomLeftFloor;
    int TRC = box.TopRightCeil;

    double tmin = -INFINITY, tmax = INFINITY;
    for (unsigned i = 0; i < 3; i++)
    {
        // x=0, y=1, z=2
        double dir_comp = ray_direction[i];
        if (dir_comp != 0.0) {
            // BottomLeftFloor = (min_x, min_y, min_z)
            double bbox_min_x = box.corner(box.BottomLeftFloor)(i);
            // TopRightCeil = (max_x, max_y, max_z)
            double bbox_max_x = box.corner(box.TopRightCeil)(i);

            double tx1 = (bbox_min_x - ray_origin(i)) / dir_comp;
            double tx2 = (bbox_max_x - ray_origin(i)) / dir_comp;

            tmin = std::max(tmin, std::min(tx1, tx2));
            tmax = std::min(tmax, std::max(tx1, tx2));
        }
    }
    return tmax >= tmin;
}

bool traverseBVH(AABBTree::Node node, const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N, double *min_t) 
{
    // no need to proceed if ray does not interset the box
    if (!ray_box_intersection(ray_origin, ray_direction, node.bbox)) 
        return false;

    // if leaf, test if hit the triangle
    if (node.left < 0 && node.right < 0) {
        assert(node.triangle >= 0);
        const Vector3i abc = facets.row(node.triangle); 
        const Vector3d a = vertices.row(abc(0));
        const Vector3d b = vertices.row(abc(1));
        const Vector3d c = vertices.row(abc(2));
        Vector3d tmp_p, tmp_N;
        const double intersection_t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
        bool found_hit = false;
        if (intersection_t > 0)
        {   
            if (intersection_t < min_t[0])
            {
                found_hit = true;
                min_t[0] = intersection_t;
                p = tmp_p;
                N = tmp_N;
            }
        } 
        return found_hit;
    }
    else {
        // fork into one or two recursive calls;
        bool Lhit = false;
        bool Rhit = false;
        if (node.left >= 0)
            Lhit = traverseBVH(bvh.nodes[node.left], ray_origin, ray_direction, p, N, min_t);
        if (node.right >= 0) 
            Rhit = traverseBVH(bvh.nodes[node.right], ray_origin, ray_direction, p, N, min_t);
        
        return Lhit || Rhit;
    }
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    // std::cout << bvh.nodes[bvh.root].parent << std::endl;
    // std::cout << bvh.nodes.size() << "\n";

    // Method (1): Traverse every triangle and return the closest hit.
    // double min_t = INFINITY;
    // bool found_hit = false;
    // for (unsigned i = 0; i < facets.rows(); i++)
    // {
    //     const Vector3d a = vertices.row(facets(i, 0));
    //     const Vector3d b = vertices.row(facets(i, 1));
    //     const Vector3d c = vertices.row(facets(i, 2));
        
    //     const double intersection_t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
    //     if (intersection_t > 0)
    //     {   
    //         if (intersection_t < min_t)
    //         {
    //             found_hit = true;
    //             min_t = intersection_t;
    //             p = tmp_p;
    //             N = tmp_N;
    //         }
    //     }
    // }
    // return found_hit;

    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    double min_t[1] = {INFINITY};
    return traverseBVH(bvh.nodes[bvh.root], ray_origin, ray_direction, p, N, min_t);
}
////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    // DONE
	double tan_half_view = std::tan(field_of_view/2);
    double L = 2 * tan_half_view * focal_length;
    double image_y = L / aspect_ratio;  // DONE: compute the correct pixels size
    double image_x = L;                 // DONE: compute the correct pixels size

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
    std::cout << "Done" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}