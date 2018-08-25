#include "intersections.h"
#include <math.h>

//Some vector operations which might be useful

double dot_product(vector a, vector b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

vector cross_product(vector a,vector b) {
  vector result;
  result.x = a.y*b.z - b.y*a.z;
  result.y = a.z*b.x - a.x*b.z;
  result.z = a.x*b.y - a.y*b.x;
  return result;
}

double distance(vector a, vector b) {
  return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

double magnitude(vector a) {
  return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

vector sum(vector a, vector b) {
  vector result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

vector difference(vector a, vector b) {
  vector result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}

vector scaled_sum(double c1, vector a, double c2, vector b) {
  vector result;
  result.x = c1 * a.x + c2 * b.x;
  result.y = c1 * a.y + c2 * b.y;
  result.z = c1 * a.z + c2 * b.z;
  return result;
}

vector scaled_difference(double c1, vector a, double c2, vector b) {
  vector result;
  result.x = c1 * a.x - c2 * b.x;
  result.y = c1 * a.y - c2 * b.y;
  result.z = c1 * a.z - c2 * b.z;
  return result;
}

vector scalar_product(double c, vector a) {
  vector result;
  result.x = c*a.x;
  result.y = c*a.y;
  result.z = c*a.z;
  return result;
}

void copy_vector(vector a, vector *result) {
  result->x = a.x;
  result->y = a.y;
  result->z = a.z;
}

//Functions used to create new objects
vector new_vector(double x, double y, double z) {
  vector result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

ray new_ray(vector start, vector dir) {
  ray result;
  result.start = start;
  result.dir = dir;
  return result;
}

disk new_disk(double radius, vector center, vector normal) {
  disk result;
  result.radius = radius;
  result.center = center;
  result.normal = normal;
  return result;
}

cylinder new_cylinder(double radius, double height, vector center, vector axis) {
  cylinder result;
  result.radius = radius;
  result.height = height;
  result.center = center;
  result.axis = axis;
  return result;
}

cone new_cone(double radius, double height, vector vertex, vector axis) {
  cone result;
  result.radius = radius;
  result.height = height;
  result.vertex = vertex;
  result.axis = axis;
  return result;
}

sphere new_sphere(double radius, vector center) {
  sphere result;
  result.radius = radius;
  result.center = center;
  return result;
}
