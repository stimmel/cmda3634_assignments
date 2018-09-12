#include <math.h>
#include "intersections.h"
#include <stdio.h>

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_sphere_intersection(ray_t observer, sphere_t obj, vector_t *intersection) {
  
  //linear term inside the norm
  vector_t h_term = observer.dir;
  //constant term inside the norm
  vector_t const_term = difference(observer.start,obj.center);
  
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

  vector_t solution = scaled_sum(1.,observer.start,h,observer.dir);

  copy_vector(solution,intersection);
  return 1;  
}

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_disk_intersection(ray_t observer, disk_t obj, vector_t *intersection) {
  double tol = 1e-8;

  //First make sure we intersect the plane containing our disk
  double numerator = dot_product(obj.normal,obj.center) - dot_product(obj.normal,observer.start);
  double denominator = dot_product(obj.normal,observer.dir);
  
  if (fabs(denominator) < tol) {
    return 0;
  }

  if (numerator/denominator < 0) return 0;
  
  //Find the disk/plane intersection
  vector_t test_pt = scaled_sum(1.,observer.start,numerator/denominator,observer.dir);

  if (distance(test_pt,obj.center) > obj.radius) return 0;

  //we have a match, so store and return
  copy_vector(test_pt,intersection);

  return 1;
}

//if there is no intersection, this function should return 0
//Otherwise, populate 'intersection' with the point of intersection and return 1
int ray_cylinder_intersection(ray_t observer, cylinder_t obj, vector_t *intersection) {
  double tol = 1e-8; //tolerance for floating point numbers
  double ray_dist;

  vector_t h_term; //linear term inside our dot product
  vector_t const_term; //constant term inside our dot product
  vector_t projected_height; //vector from the base of the cylinder to the intersection point
  vector_t test_pt; //guess for the intersection point
  
  //First make sure we intersect an infinite cylinder
  //Use a projection operation to divide a quadratic equation
  h_term = scaled_difference(1.,observer.dir, \
			     dot_product(observer.dir,obj.axis),obj.axis);
  const_term = scaled_difference(1., difference(observer.start, obj.center), \
		    dot_product(obj.axis, difference(observer.start, obj.center)),obj.axis);
  
  double a = dot_product(h_term,h_term);

  double b = 2*dot_product(h_term,const_term);

  double c = dot_product(const_term,const_term) - obj.radius*obj.radius;

  //no intersection at all
  if (b*b - 4*a*c < 0.) return 0;
  //the cylinder is behind us
  if (-1*b + sqrt(fabs(b*b - 4*a*c)) < tol) return 0;

  if (-1*b > sqrt(b*b - 4*a*c)) {
    ray_dist = (-1*sqrt(b*b - 4*a*c) - b)/(2*a);
    
    test_pt = scaled_sum(1,observer.start,	\
			 ray_dist,observer.dir);
    
    projected_height = difference(test_pt,obj.center);
    
    //check to make sure we are in the bounded cylinder
    if (dot_product(projected_height,obj.axis) > 0 && 
	dot_product(projected_height,obj.axis) < obj.height) {
      copy_vector(test_pt,intersection);
      return 1;
    }
  }

  ray_dist = (sqrt(b*b - 4*a*c) - b)/(2*a);
  
  test_pt = scaled_sum(1,observer.start,		\
		       ray_dist,observer.dir);
  
  projected_height = difference(test_pt,obj.center);
  
  //check to make sure we are in the bounded cylinder
  if (dot_product(projected_height,obj.axis) > 0 && 
      dot_product(projected_height,obj.axis) < obj.height) {
    copy_vector(test_pt,intersection);
    return 1;
  }
  
  return 0;
}

int ray_cone_intersection(ray_t observer, cone_t obj, vector_t *intersection) {
  //Question 7: Modify this function to compute an intersection
  return 0;
}
