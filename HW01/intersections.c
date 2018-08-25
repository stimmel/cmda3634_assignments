#include <math.h>
#include "intersections.h"
#include <stdio.h>

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_sphere_intersection(ray observer, sphere obj, vector *intersection) {
  
  //linear term inside the norm
  vector h_term = observer.dir;
  //constant term inside the norm
  vector const_term = difference(observer.start,obj.center);
  
  //quadratic term in the expansion
  double a = dot_product(h_term,h_term);
  //linear term in the expansion
  double b = 2*dot_product(h_term,const_term);
  //constant term in the expansion
  double c = dot_product(const_term,const_term) - obj.radius*obj.radius;

  double discriminant = b*b - 4*a*c;

  //make sure the equation has real roots
  if (discriminant < 0.) return 0;
  //find the smallest positive h
  double h;
  if ((-1*b - sqrt(discriminant))/(2*a) > 0) h = (-1*b - sqrt(discriminant))/(2*a); //sphere in front of us
  else if ((-1*b + sqrt(discriminant))/(2*a) > 0) h = (-1*b + sqrt(discriminant))/(2*a); //we are inside the sphere
  else return 0; //no positive roots, so the sphere is behind us

  vector solution = scaled_sum(1.,observer.start,h,observer.dir);

  copy_vector(solution,intersection);
  return 1;  
}

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_disk_intersection(ray observer, disk obj, vector *intersection) {
  //Question 3: Modify this function to compute an intersection
  return 0;
}

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_cylinder_intersection(ray observer, cylinder obj, vector *intersection) {
  //Question 5: Modify this function to compute an intersection
  return 0;
}

int ray_cone_intersection(ray observer, cone obj, vector *intersection) {
  //Question 7: Modify this function to compute an intersection
  return 0;
}
