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
	//
	// 		  p3(-1,2) o        o p2(2,1)
	//					^       ^
	// 		             \     / 
	//		  p4(-1,1) o  \   /  o p1(1,1)
	// 				    `  \ /  `
	//     			   	    o p0(0,0)
	// 			order: p1 -> p2 -> p3 -> p4
	//
	// det(u, v) < 0 mean u -> v is a counterclockwise angle, vise versa
	double x1 = u.real(), y1 = u.imag(), x2 = v.real(), y2 = v.imag();
	return x1 * y2 - x2 * y1;
}

// Comparator used for std::sort
struct Compare {
	Point p0; // bottom-most point of the points cloud as the base point

	// Compare(p1, p2) = True if p1-p0 form a larger counter-clockwise angle than p2-p0
	// 				   = False vise versa
	bool operator ()(const Point &p1, const Point &p2) {
		return det(p1-p0, p2-p0) > 0;
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	double x1 = a.real(), y1 = a.imag(), x2 = b.real(), y2 = b.imag(), x3 = c.real(), y3 = c.imag();
	// True if a->b->c forms a salient angle, i.e make a left turn
	// False vice versa.
	return (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1) > 0;
}

// util function to get the second top element from the stack
Point pointUnderStackTop(std::stack<Point> &S)
{
    Point p = S.top();
    S.pop();
    Point res = S.top();
    S.push(p);
    return res;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// 1. Find the bottom point in the input point cloud P0.
	double ymin = points.at(0).imag();
	int min = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
    	int y = points.at(i).imag();
    	// Pick the bottom-most point. choose the left most point when y equal
     	if ((y < ymin) || (ymin == y && 
     		points.at(i).real() < points.at(i).real())) 
			ymin = points.at(i).imag(), min = i;
    }
	/* 
		San check here
	*/
	// std::cout << "ymin: " << ymin << " min pos: " << min << "\n";
	// std::cout << "min point: " << points.at(min) << "\n";

	// 2. Sort all input points counter-clockwise with respect to P0. Smaller counterclock-wise angle goes first
	order.p0 = points.at(min);
	std::sort(points.begin(), points.end(), order);
	// notice here the bottom-most point is not at pos 0 bc the determinant evals to be 0

	/* 
		San check here
	*/
	// for (int i = 0; i < 8; i++)
	// {
	// 	std::cout << "pos" << i << " : " << points.at(i) << "\n";
	// }

	Polygon hull;
	// create stack, initialize with the bottom-most and two following points in sorting order, containing the temporary hull
	std::stack<Point> stk;
	stk.push(order.p0);
	stk.push(points.at(0));
	stk.push(points.at(1));
	// 3. Greedily compute the convex-hull polygon by iterating over the sorted points.
	for (size_t i = 2; i < points.size(); i++)
	{
		Point top = stk.top();
		Point undertop = pointUnderStackTop(stk);
		// 3. Whenever a "right-turn" is encountered, pop the middle-point from the hull (temporary).
		while (stk.size() > 0 && !salientAngle(undertop, top, points.at(i)) ){
			stk.pop();
			top = stk.top();
			undertop = pointUnderStackTop(stk);
		}
		// notice here the left-bottom point is not at pos 0 bc the determinant evals to be 0
		if (points.at(i) != order.p0) stk.push(points.at(i));
	}

	// dump all points remaining to the output hull
	while (!stk.empty())
	{
		Point p = stk.top();
		hull.push_back(p);
		stk.pop();
	}

	// std::cout << hull.size() << "Points \n";
	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	 
	// TODO
	if (in.fail()) std::cerr << "fail to open file \n", exit(-1);
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

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " ../../data/points.xyz ../../output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
