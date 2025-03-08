#include "uav_utilities.h"

// check if two values are close enough to each other (dist < epsilon)
bool equals_epsilon(const double a, const double b, const double epsilon) {
	return fabs(a - b) < epsilon;
}

// standard vector distance
double vec2DDist(const double x1, const double y1, const double x2, const double y2) {
	const double diffx = x2 - x1;
	const double diffy = y2 - y1;
	return sqrt(diffx * diffx + diffy * diffy);
}

// standard vector dot product
double dotProduct2D(const double x1, const double y1, const double x2, const double y2) {
	return x1 * x2 + y1 * y2;
}

// provides angle between two vectors in degrees(!)
double getAngleBetweenTwoVectors(const double x1, const double y1, const double x2, const double y2) {
	// details (from the stack-overflow page)
	// v1=vector1, v2=vector2
	//res = -atan2(v1[0] * v2[1] - v2[0] * v1[1], sum(a * b for a, b in zip(v1, v2)))
	//angle = (-180 / pi * res) % 360
	// this gives us the angle in degrees, if it's > 180, we turn clockwise.
	const double res = atan2(x1 * y2 - x2 * y1, x1 * x2 + y1 * y2) * 180. / M_PI;
	const double tmp = fmod(res, 360.);
	return res < 0 ? tmp + 360. : tmp;  // double modulo implementation for clamping angle between 0 and 360
}

// dot product of the directions of the vectors, used to avoid scale issues
double normalizedDotProduct2D(const double x1, const double y1, const double x2, const double y2) {
	const double magProd = (sqrt(x1 * x1 + y1 * y1) * sqrt(x2 * x2 + y2 * y2));
	return ((x1 * x2) / magProd) + ((y1 * y2) / magProd);
}