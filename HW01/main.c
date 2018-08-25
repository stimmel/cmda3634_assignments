#include "intersections.h"
#include <math.h>
#include <stdio.h>

int main(int argc, char ** argv) {

  //temporary variables used in testing
  vector center;
  vector vertex;
  vector normal;
  vector axis;
  
  //an observer peering in a random direction
  vector obs_start = new_vector(0.1,0.5,1.);
  vector obs_dir = new_vector(0.8,-0.6,0.);
  ray observer = new_ray(obs_start,obs_dir);

  //a sphere in his way
  center = new_vector(2.5,-1.3,1.);
  sphere yes_sphere = new_sphere(1.,center);

  vector sphere_intersection_pt = new_vector(1.7,-0.7,1.);

  //a sphere out of his way
  center = new_vector(3.,-1.7,2.);
  sphere no_sphere_1 = new_sphere(0.5,center);

  //a sphere behind him
  center = new_vector(-0.7,1.1,1.);
  sphere no_sphere_2 = new_sphere(0.5,center);
  
  //a disk in his way
  center = new_vector(0.8,0.,1.2);
  normal = new_vector(0.96,-0.28,0.);
  disk yes_disk = new_disk(2.5,center,normal);

  vector disk_intersection_pt = new_vector(0.794017094,-0.020512821,1.);
  
  //a disk out of the way
  center = new_vector(0.9,-0.1,2.2);
  normal = new_vector(0.96,0.,-0.28);
  disk no_disk_1 = new_disk(1.,center,normal);

  //a disk behind him
  center = new_vector(-0.7,1.1,1.);
  normal = new_vector(0.96,-0.28,0.);
  disk no_disk_2 = new_disk(2.5,center,normal);

  //a disk parallel to him, but off to the side
  center = new_vector(2.5,-1.3,1.2);
  normal = new_vector(0.,0.,1.);
  disk no_disk_3 = new_disk(2.,center,normal);

  //a cylinder in his way
  center = new_vector(0.9,-0.7,0.4);
  axis = new_vector(0.8,0.,0.6); 
  cylinder yes_cylinder = new_cylinder(0.5, 2.,center,axis);
  
  vector cylinder_intersection_pt = new_vector(1.179421,-0.309566,1.);

  //a cylinder not in his way
  center = new_vector(0.9,1.4,0.3);
  axis = new_vector(0.8,0.,0.6);
  cylinder no_cylinder_1 = new_cylinder(0.5, 2., center, axis);

  //a cylinder behind him
  center = new_vector(-2.3,1.7,0.4);
  axis = new_vector(-2.3,1.7,0.4);
  cylinder no_cylinder_2 = new_cylinder(0.5, 2., center, axis);

  //a cylinder parallel to him
  center = new_vector(0.9,-0.1,1.);
  axis = new_vector(0.8,-0.6,0.);
  cylinder no_cylinder_3 = new_cylinder(0.5, 2., center, axis);
  
  //a cone in his way
  vertex = new_vector(0.9,-0.7,0.4);
  axis = new_vector(0.8,0.,0.6);
  cone yes_cone = new_cone(1., 2., vertex, axis);

  vector cone_intersection_pt = new_vector(1.33248,-0.42436,1.);
  
  //a cone behind him
  vertex = new_vector(-2.3,1.7,0.4);
  normal = new_vector(0.8,0.,0.6);
  cone no_cone_1 = new_cone(1., 2., vertex, axis);

  //a cone not in his way
  vertex = new_vector(0.9,1.4,0.3);
  axis = new_vector(0.8,0.,0.6);
  cone no_cone_2 = new_cone(0.5, 2., vertex, axis);

  //a cone barely not in his way (cylinder vs cone)
  vertex = new_vector(0.9,0.,0.4);
  axis = new_vector(0.8,0.,0.6);
  cone no_cone_3 = new_cone(2.0, 4., vertex, axis);

  vector intersection;
  double test_tol = 1e-4;

  printf("\n\n Sphere tests: \n\n");

  if (!ray_sphere_intersection(observer,yes_sphere,&intersection))
    printf("Test 1 failed: This sphere should intersect the ray\n");
  else if (distance(intersection,sphere_intersection_pt) > test_tol)
    printf("Test 1 failed: Intersection point off by %g\n",distance(intersection,sphere_intersection_pt));
  else printf("Test 1 passed\n");

  if (ray_sphere_intersection(observer,no_sphere_1,&intersection))
    printf("Test 2 failed: This sphere should not intersect the ray\n");
  else printf("Test 2 Passed\n");

  if (ray_sphere_intersection(observer,no_sphere_2,&intersection))
    printf("Test 3 failed: This sphere should not intersect the ray\n");
  else printf("Test 3 Passed\n");
  
  printf("\n\n Disk tests: \n\n");

  if (!ray_disk_intersection(observer,yes_disk,&intersection))
    printf("Test 4 failed: This disk should intersect the ray\n");
  else if (distance(intersection,disk_intersection_pt) > test_tol)
    printf("Test 4 failed: Intersection point off by %g\n",distance(intersection,disk_intersection_pt));
  else printf("Test 4 Passed\n");

  if (ray_disk_intersection(observer,no_disk_1,&intersection))
    printf("Test 5 failed: This disk should not intersect the ray\n");
  else printf("Test 5 Passed\n");

  if (ray_disk_intersection(observer,no_disk_2,&intersection))
    printf("Test 6 failed: This object should be directly behind the observer.\n");
  else printf("Test 6 Passed\n");

  if (ray_disk_intersection(observer,no_disk_3,&intersection))
    printf("Test 7 failed: What happens when your disk is parallel?.\n");
  else printf("Test 7 Passed\n");
  
  printf("\n\n Cylinder tests: \n\n");
  
  if (!ray_cylinder_intersection(observer,yes_cylinder,&intersection))
    printf("Test 8 failed: This cylinder should intersect the ray\n");
  else if (distance(intersection,cylinder_intersection_pt) > test_tol)
    printf("Test 8 failed: Intersection point off by %g\n",distance(intersection,cylinder_intersection_pt));
  else printf("Test 8 Passed\n");
  
  if (ray_cylinder_intersection(observer,no_cylinder_1,&intersection))
    printf("Test 9 failed: This cylinder should not intersect the ray\n");
  else printf("Test 9 Passed\n");

  if (ray_cylinder_intersection(observer,no_cylinder_2,&intersection))
    printf("Test 10 failed: This cylinder should be behind the observer.\n");
  else printf("Test 10 Passed\n");

  if (ray_cylinder_intersection(observer,no_cylinder_3,&intersection))
    printf("Test 11 failed: Make sure your cylinder is open.\n");
  else printf("Test 11 Passed\n");
  
  printf("\n\n Cone tests: \n\n");
  
  if (!ray_cone_intersection(observer,yes_cone,&intersection))
    printf("Test 12 failed: This cone should intersect the ray\n");
  else if (distance(intersection,cone_intersection_pt) > test_tol)
    printf("Test 12 failed: Intersection point off by %g\n",distance(intersection,cone_intersection_pt));
  else printf("Test 12 Passed\n");
  
  if (ray_cone_intersection(observer,no_cone_1,&intersection))
    printf("Test 13 failed: This cone should not intersect the ray\n");
  else printf("Test 13 Passed\n");

  if (ray_cone_intersection(observer,no_cone_2,&intersection))
    printf("Test 14 failed: This cone should be behind the observer.\n");
  else printf("Test 14 Passed\n");

  if (ray_cone_intersection(observer,no_cone_3,&intersection))
    printf("Test 15 failed: Check to make sure your cone is not a cylinder.\n");
  else printf("Test 15 Passed\n");
  
  return 0;
}
