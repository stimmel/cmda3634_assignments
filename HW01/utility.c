#include "intersections.h"
#include <math.h>

//Some vector operations which might be useful

double dot_product(vector_t a, vector_t b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

vector_t cross_product(vector_t a,vector_t b) {
  vector_t result;
  result.x = a.y*b.z - b.y*a.z;
  result.y = a.z*b.x - a.x*b.z;
  result.z = a.x*b.y - a.y*b.x;
  return result;
}

double distance(vector_t a, vector_t b) {
  return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

double magnitude(vector_t a) {
  return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

vector_t sum(vector_t a, vector_t b) {
  vector_t result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

vector_t difference(vector_t a, vector_t b) {
  vector_t result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}

vector_t scaled_sum(double c1, vector_t a, double c2, vector_t b) {
  vector_t result;
  result.x = c1 * a.x + c2 * b.x;
  result.y = c1 * a.y + c2 * b.y;
  result.z = c1 * a.z + c2 * b.z;
  return result;
}

vector_t scaled_difference(double c1, vector_t a, double c2, vector_t b) {
  vector_t result;
  result.x = c1 * a.x - c2 * b.x;
  result.y = c1 * a.y - c2 * b.y;
  result.z = c1 * a.z - c2 * b.z;
  return result;
}

vector_t scalar_product(double c, vector_t a) {
  vector_t result;
  result.x = c*a.x;
  result.y = c*a.y;
  result.z = c*a.z;
  return result;
}

void copy_vector(vector_t a, vector_t *result) {
  result->x = a.x;
  result->y = a.y;
  result->z = a.z;
}

//Functions used to create new objects
vector_t new_vector(double x, double y, double z) {
  vector_t result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

ray_t new_ray(vector_t start, vector_t dir) {
  ray_t result;
  result.start = start;
  result.dir = dir;
  return result;
}

disk_t new_disk(double radius, vector_t center, vector_t normal) {
  disk_t result;
  result.radius = radius;
  result.center = center;
  result.normal = normal;
  return result;
}

cylinder_t new_cylinder(double radius, double height, vector_t center, vector_t axis) {
  cylinder_t result;
  result.radius = radius;
  result.height = height;
  result.center = center;
  result.axis = axis;
  return result;
}

cone_t new_cone(double radius, double height, vector_t vertex, vector_t axis) {
  cone_t result;
  result.radius = radius;
  result.height = height;
  result.vertex = vertex;
  result.axis = axis;
  return result;
}

sphere_t new_sphere(double radius, vector_t center) {
  sphere_t result;
  result.radius = radius;
  result.center = center;
  return result;
}
