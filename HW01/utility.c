#include "intersections.h"
#include <math.h>

double dot_product(double *a, double *b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void cross_product(double *a,double *b, double *result) {
  result[0] = a[1]*b[2] - b[1]*a[2];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

double distance(double *a, double *b) {
  return sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2]));
}

double magnitude(double *a) {
  return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

void sum(double *a, double *b, double *result) {
  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];
  result[2] = a[2] + b[2];
}

void difference(double *a, double *b, double *result) {
  result[0] = a[0] - b[0];
  result[1] = a[1] - b[1];
  result[2] = a[2] - b[2];
}

void scaled_sum(double c1, double *a, double c2, double *b,double *result) {
  result[0] = c1 * a[0] + c2 * b[0];
  result[1] = c1 * a[1] + c2 * b[1];
  result[2] = c1 * a[2] + c2 * b[2];
}

void scaled_difference(double c1, double *a, double c2, double *b,double *result) {
  result[0] = c1 * a[0] - c2 * b[0];
  result[1] = c1 * a[1] - c2 * b[1];
  result[2] = c1 * a[2] - c2 * b[2];
}

void scalar_product(double c, double *a, double *result) {
  result[0] = c*a[0];
  result[1] = c*a[1];
  result[2] = c*a[2];
}

void copy_vector(double *a, double *result) {
  result[0] = a[0];
  result[1] = a[1];
  result[2] = a[2];
}
