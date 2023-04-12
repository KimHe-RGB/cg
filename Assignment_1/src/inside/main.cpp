////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// u = (x1, y1); v = (x2, y2);
	// compute the determinant of:
	// 	|x1 y1|    |x1 x2|
	// 	|x2 y2| == |y1 y2|
	double x1 = u.real(), y1 = u.imag(), x2 = v.real(), y2 = v.imag();
	return x1 * y2 - x2 * y1;
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	// TODO
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment

	double x1 = a.real(), y1 = a.imag(), x2 = b.real(), y2 = b.imag();
	double x3 = c.real(), y3 = c.imag(), x4 = d.real(), y4 = d.imag();

	double t = det(a-c, c-d) / det(a-b, c-d);
	double u = det(a-c, a-b) / det(a-b, c-d);

	if (t > 1 || t < 0 || u > 1 || u < 0) return false;
	ans = Point(x1+t*(x2-x1), y1+t*(y2-y1));
	return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	// TODO
	// find the max y and x is sufficient to find a point outside the polygon
	double max_y = 0, max_x = 0;
	for (size_t i = 0; i < poly.size(); i++) {
		if (poly.at(i).real() > max_x) max_x = poly.at(i).real();
		if (poly.at(i).imag() > max_y) max_y = poly.at(i).imag();
	}
	Point outside = Point(max_x + 100, max_y + 100);
	
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	// TODO
	int count = 0;
	Point ans;
	for (size_t i = 0; i < poly.size()-1; i++)
	{
		if (intersect_segment(outside, query, poly.at(i), poly.at(i+1), ans)) count++;
	}
	if (intersect_segment(outside, query, poly.at(poly.size()-2), poly.at(poly.size()-1), ans)) count++;

	return count % 2 == 1;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);

	// TODO
	if (in.fail()) throw std::runtime_error("failed to open file " + filename);
	int N;
	in >> N; // number of points
	for (int i = 0; i < N; i++)
	{
		double x, y, z;
		in >> x >> y >> z;
		Point p = Point(x, y);
		points.push_back(p);
	}
	in.close();
	return points;
}

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	// TODO
	if (in.fail()) throw std::runtime_error("failed to open file " + filename);

	Polygon poly;
	char T;
	in >> T; // type of input
	while (T == 'v') // List of geometric vertices
	{
		double x, y, z;
		in >> x >> y >> z;
		Point p = Point(x, y);
		poly.push_back(p);
		in >> T;
	}
	in.close();
	return poly;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	// TODO
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	out << points.size() << "\n";
	for (const auto &v : points) {
		out << v.real() << ' ' << v.imag() << " 0\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz polygon.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
