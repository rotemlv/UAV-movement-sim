#ifndef UAV_UTILITIES_H
#define UAV_UTILITIES_H

#include "project_headers.h"

// helper functions used by the UAV class, which are not directly object-related
// possible suggestion is to define a Vector class which implements these instead
bool equals_epsilon(const double a, const double b, const double epsilon);
double vec2DDist(const double x1, const double y1, const double x2, const double y2);
double dotProduct2D(const double x1, const double y1, const double x2, const double y2);
double getAngleBetweenTwoVectors(const double x1, const double y1, const double x2, const double y2);
double normalizedDotProduct2D(const double x1, const double y1, const double x2, const double y2);
#endif