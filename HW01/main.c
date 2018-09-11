#include "intersections.h"
#include <math.h>
#include <stdio.h>

int main(int argc, char ** argv) {

  //temporary variables used in testing
  vector_t center;
  vector_t vertex;
  vector_t normal;
  vector_t axis;
  
  //an observer peering in a predictable direction
  vector_t obs_start = new_vector(0.1,0.5,1.);
  vector_t obs_dir = new_vector(0.8,-0.6,0.);
  ray_t observer = new_ray(obs_start,obs_dir);

  //second observer, more random direction
  obs_start = new_vector(0.5,-0.2,-0.8);
  obs_dir = new_vector(0.707106781,0.577350269,0.40824829);
  ray_t observer2 = new_ray(obs_start,obs_dir);

  //third observer, perfect diagonal
  obs_start = new_vector(-2.,3.,5.);
  obs_dir = new_vector(0.577350269,0.577350269,0.577350269);
  ray_t observer3 = new_ray(obs_start,obs_dir);
  
  vector_t intersection;
  double test_tol = 1e-4;

  printf("\n\n Sphere tests: \n\n");

  //positive test, observer 1
  center = new_vector(2.5,-1.3,1.);
  sphere_t yes_sphere = new_sphere(1.,center);

  vector_t sphere_intersection_pt = new_vector(1.7,-0.7,1.);
  
  if (!ray_sphere_intersection(observer,yes_sphere,&intersection))
    printf("Test 1 failed: This sphere should intersect the ray\n");
  else if (distance(intersection,sphere_intersection_pt) > test_tol)
    printf("Test 1 failed: Intersection point off by %g\n",distance(intersection,sphere_intersection_pt));
  else printf("Test 1 passed\n");

  //negative test, observer 1
  center = new_vector(3.,-1.7,2.);
  sphere_t no_sphere_1 = new_sphere(0.5,center);
  
  if (ray_sphere_intersection(observer,no_sphere_1,&intersection))
    printf("Test 2 failed: This sphere should not intersect the ray\n");
  else printf("Test 2 Passed\n");

  //edge case: a sphere behind observer 1
  center = new_vector(-0.7,1.1,1.);
  sphere_t no_sphere_2 = new_sphere(0.5,center);
  
  if (ray_sphere_intersection(observer,no_sphere_2,&intersection))
    printf("Test 3 failed: This sphere should not intersect the ray\n");
  else printf("Test 3 Passed\n");
  
  printf("\n\n Disk tests: \n\n");

  //positive test, observer 1
  center = new_vector(0.8,0.,1.2);
  normal = new_vector(0.96,-0.28,0.);
  disk_t yes_disk_1 = new_disk(2.5,center,normal);

  vector_t disk_intersection_pt1 = new_vector(0.794017094,-0.020512821,1.);
  
  if (!ray_disk_intersection(observer,yes_disk_1,&intersection))
    printf("Test 4 failed: This disk should intersect the ray\n");
  else if (distance(intersection,disk_intersection_pt1) > test_tol)
    printf("Test 4 failed: Intersection point off by %g\n",distance(intersection,disk_intersection_pt1));
  else printf("Test 4 Passed\n");

  //positive test, observer 2
  center = new_vector(1.1,0.3,-0.4);
  normal = new_vector(0.707106781,0.577350269,-0.40824829);
  disk_t yes_disk_2 = new_disk(1.2,center,normal);

  vector_t disk_intersection_pt2 = new_vector(1.08298,0.276002,-0.463416);
  
  if (!ray_disk_intersection(observer2,yes_disk_2,&intersection))
    printf("Test 5 failed: This disk should intersect the ray\n");
  else if (distance(intersection,disk_intersection_pt2) > test_tol)
  printf("Test 5 failed: Intersection point off by %g\n",distance(intersection,disk_intersection_pt2));
  else printf("Test 5 Passed\n");

  //positive test, observer 3
  center = new_vector(-0.3,4.5,6.9);
  normal = new_vector(0.6,0.3,0.741619849);
  disk_t yes_disk_3 = new_disk(1.2,center,normal);

  vector_t disk_intersection_pt3 = new_vector(-0.246197,4.7538,6.7538);
  
  if (!ray_disk_intersection(observer3,yes_disk_3,&intersection))
    printf("Test 6 failed: This disk should intersect the ray\n");
  else if (distance(intersection,disk_intersection_pt3) > test_tol)
  printf("Test 6 failed: Intersection point off by %g\n",distance(intersection,disk_intersection_pt3));
  else printf("Test 6 Passed\n");

  //negative test, observer 1
  center = new_vector(0.9,-0.1,2.2);
  normal = new_vector(0.96,0.,-0.28);
  disk_t no_disk_1 = new_disk(1.,center,normal);
  
  if (ray_disk_intersection(observer,no_disk_1,&intersection))
    printf("Test 7 failed: This disk should not intersect the ray\n");
  else printf("Test 7 Passed\n");

  //negative test, observer 2
  center = new_vector(1.1,-0.3,-0.4);
  normal = new_vector(0.707106781,0.577350269,-0.40824829);
  disk_t no_disk_2 = new_disk(0.5,center,normal);
  
  if (ray_disk_intersection(observer2,no_disk_2,&intersection))
    printf("Test 8 failed: This disk should not intersect the ray\n");
  else printf("Test 8 Passed\n");

  //negative test, observer 3
  center = new_vector(1.3,4.5,6.9);
  normal = new_vector(0.6,0.3,0.741619849);
  disk_t no_disk_3 = new_disk(0.4,center,normal);
  
  if (ray_disk_intersection(observer3,no_disk_3,&intersection))
    printf("Test 9 failed: This disk should not intersect the ray\n");
  else printf("Test 9 Passed\n");

  //edge case, disk behind observer 1
  center = new_vector(-0.7,1.1,1.);
  normal = new_vector(0.96,-0.28,0.);
  disk_t no_disk_4 = new_disk(2.5,center,normal);
    
  if (ray_disk_intersection(observer,no_disk_4,&intersection))
    printf("Test 10 failed: This object should be directly behind the observer.\n");
  else printf("Test 10 Passed\n");

  //edge case, disk to the side of observer 1
  center = new_vector(2.5,-1.3,1.2);
  normal = new_vector(0.,0.,1.);
  disk_t no_disk_5 = new_disk(2.,center,normal);
  
  if (ray_disk_intersection(observer,no_disk_5,&intersection))
    printf("Test 11 failed: What happens when your disk is parallel?.\n");
  else printf("Test 11 Passed\n");
  
  printf("\n\n Cylinder tests: \n\n");

  //positive test, observer 1
  center = new_vector(0.9,-0.7,0.4);
  axis = new_vector(0.8,0.,0.6); 
  cylinder_t yes_cylinder_1 = new_cylinder(0.5, 2.,center,axis);
  
  vector_t cylinder_intersection_pt1 = new_vector(1.179421,-0.309566,1.);
  
  if (!ray_cylinder_intersection(observer,yes_cylinder_1,&intersection))
    printf("Test 12 failed: This cylinder should intersect the ray\n");
  else if (distance(intersection,cylinder_intersection_pt1) > test_tol)
    printf("Test 12 failed: Intersection point off by %g\n",distance(intersection,cylinder_intersection_pt1));
  else printf("Test 12 Passed\n");

  //positive test, observer 2
  center = new_vector(3.3,0.7,-0.1);
  axis = new_vector(-0.707106781,0.577350269,0.40824829);
  cylinder_t yes_cylinder_2 = new_cylinder(1.,3.,center,axis);

  vector_t cylinder_intersection_pt2 = new_vector(1.76334,0.831516,-0.0706083);
  
  if (!ray_cylinder_intersection(observer2,yes_cylinder_2,&intersection))
    printf("Test 13 failed: This cylinder should intersect the ray\n");
  else if (distance(intersection,cylinder_intersection_pt2) > test_tol)
    printf("Test 13 failed: Intersection point off by %g\n",distance(intersection,cylinder_intersection_pt2));
  else printf("Test 13 Passed\n");

  //positive test, observer 3
  center = new_vector(0.4,5.7,8.74);
  axis = new_vector(0.6,0.3,-0.741619849);
  cylinder_t yes_cylinder_3 = new_cylinder(1.,1.4,center,axis);

  vector_t cylinder_intersection_pt3 = new_vector(0.419612,5.41961,7.41961);
  
  if (!ray_cylinder_intersection(observer3,yes_cylinder_3,&intersection))
    printf("Test 14 failed: This cylinder should intersect the ray\n");
  else if (distance(intersection,cylinder_intersection_pt3) > test_tol)
    printf("Test 14 failed: Intersection point off by %g\n",distance(intersection,cylinder_intersection_pt3));
  else printf("Test 14 Passed\n");

  //negative test, observer 1
  center = new_vector(0.9,1.4,0.3);
  axis = new_vector(0.8,0.,0.6);
  cylinder_t no_cylinder_1 = new_cylinder(0.5, 2., center, axis);
  
  if (ray_cylinder_intersection(observer,no_cylinder_1,&intersection))
    printf("Test 15 failed: This cylinder should not intersect the ray\n");
  else printf("Test 15 Passed\n");

  //negative test, observer 2
  center = new_vector(1.3,-1.7,1.1);
  axis = new_vector(-0.707106781,0.577350269,0.40824829);
  cylinder_t no_cylinder_2 = new_cylinder(1.,3.,center,axis);
  
  if (ray_cylinder_intersection(observer2,no_cylinder_2,&intersection))
    printf("Test 16 failed: This cylinder should not intersect the ray\n");
  else printf("Test 16 Passed\n");
  
  //negative test, observer 3
  center = new_vector(0.4,3.7,8.74);
  axis = new_vector(0.6,0.3,-0.741619849);
  cylinder_t no_cylinder_3 = new_cylinder(1.,1.4,center,axis);

  if (ray_cylinder_intersection(observer3,no_cylinder_3,&intersection))
    printf("Test 17 failed: This cylinder should not intersect the ray\n");
  else printf("Test 17 Passed\n");

  //edge case: cylinder behind observer 1
  center = new_vector(-2.3,1.7,0.4);
  axis = new_vector(-2.3,1.7,0.4);
  cylinder_t no_cylinder_4 = new_cylinder(0.5, 2., center, axis);
    
  if (ray_cylinder_intersection(observer,no_cylinder_4,&intersection))
    printf("Test 18 failed: This cylinder should be behind the observer.\n");
  else printf("Test 18 Passed\n");

  //edge case: observer 1 looking along the axis
  center = new_vector(0.9,-0.1,1.);
  axis = new_vector(0.8,-0.6,0.);
  cylinder_t no_cylinder_5 = new_cylinder(0.5, 2., center, axis);
  
  if (ray_cylinder_intersection(observer,no_cylinder_5,&intersection))
    printf("Test 19 failed: Make sure your cylinder is open.\n");
  else printf("Test 19 Passed\n");

  //edge case: observer 3 peering under the lower rim
  center = new_vector(2.,7.,9.);
  axis = new_vector(0.6,-0.3,0.741619849);
  cylinder_t yes_cylinder_4 = new_cylinder(1.,2.4,center,axis);

  vector_t cylinder_intersection_pt4 = new_vector(2.72262,7.72262,9.72262);
  
  if (!ray_cylinder_intersection(observer3,yes_cylinder_4,&intersection))
    printf("Test 20 failed: This cylinder should intersect the ray.  Check your bounds.\n");
  else if (distance(intersection,cylinder_intersection_pt4) > test_tol)
    printf("Test 20 failed: Intersection point off by %g.  Check your bounds.\n",distance(intersection,cylinder_intersection_pt4));
  else printf("Test 20 Passed\n");

  //edge case: observer 3 peering over the upper rim
  center = new_vector(0.8,7.6,10.4832);
  axis = new_vector(0.6,-0.3,-0.741619849);
  cylinder_t yes_cylinder_5 = new_cylinder(1.,2.,center,axis);

  vector_t cylinder_intersection_pt5 = new_vector(2.59707,7.59707,9.59707);
  
  if (!ray_cylinder_intersection(observer3,yes_cylinder_5,&intersection))
    printf("Test 21 failed: This cylinder should intersect the ray.  Check your bounds\n");
  else if (distance(intersection,cylinder_intersection_pt5) > test_tol)
    printf("Test 21 failed: Intersection point off by %g.  Check your bounds.\n",distance(intersection,cylinder_intersection_pt5));
  else printf("Test 21 Passed\n");
    
  printf("\n\n Cone tests: \n\n");

  //positive test, observer 1
  vertex = new_vector(0.9,-0.7,0.4);
  axis = new_vector(0.8,0.,0.6);
  cone_t yes_cone_1 = new_cone(1., 2., vertex, axis);

  vector_t cone_intersection_pt1 = new_vector(1.33248,-0.42436,1.);
  
  if (!ray_cone_intersection(observer,yes_cone_1,&intersection))
    printf("Test 22 failed: This cone should intersect the ray\n");
  else if (distance(intersection,cone_intersection_pt1) > test_tol)
    printf("Test 22 failed: Intersection point off by %g\n",distance(intersection,cone_intersection_pt1));
  else printf("Test 22 Passed\n");

  //positive test, observer 2
  vertex = new_vector(1.9,1.8,-0.1);
  axis = new_vector(0.707106781,-0.577350269,0.40824829);
  cone_t yes_cone_2 = new_cone(2.,2.,vertex,axis);

  vector_t cone_intersection_pt2 = new_vector(1.976,1.00515,0.0521684);
  
  if (!ray_cone_intersection(observer2,yes_cone_2,&intersection))
    printf("Test 23 failed: This cone should intersect the ray\n");
  else if (distance(intersection,cone_intersection_pt2) > test_tol)
    printf("Test 23 failed: Intersection point off by %g\n",distance(intersection,cone_intersection_pt2));
  else printf("Test 23 Passed\n");

  //positive test, observer 3
  vertex = new_vector(0.4,5.7,8.74);
  axis = new_vector(0.6,0.3,-0.741619849);
  cone_t yes_cone_3 = new_cone(2.,4.,vertex,axis);

  vector_t cone_intersection_pt3 = new_vector(0.722587,5.72259,7.72259);
  
  if (!ray_cone_intersection(observer3,yes_cone_3,&intersection))
    printf("Test 24 failed: This cone should intersect the ray\n");
  else if (distance(intersection,cone_intersection_pt3) > test_tol)
    printf("Test 24 failed: Intersection point off by %g\n",distance(intersection,cone_intersection_pt3));
  else printf("Test 24 Passed\n");

  //negative test: observer 1
  vertex = new_vector(0.9,1.4,0.3);
  axis = new_vector(0.8,0.,0.6);
  cone_t no_cone_1 = new_cone(0.5, 2., vertex, axis);
  
  if (ray_cone_intersection(observer,no_cone_1,&intersection))
    printf("Test 25 failed: This cone should not intersect the ray\n");
  else printf("Test 25 Passed\n");

  //negative test, observer 2
  vertex = new_vector(-0.9,2.8,3.1);
  axis = new_vector(0.707106781,-0.577350269,0.40824829);
  cone_t no_cone_2 = new_cone(1.,2.,vertex,axis);
  
  if (ray_cone_intersection(observer2,no_cone_2,&intersection))
    printf("Test 26 failed: This cone should not intersect the ray\n");
  else printf("Test 26 Passed\n");

  //negative test, observer 3
  vertex = new_vector(1.4,4.3,3.3);
  axis = new_vector(0.6,-0.3,0.741619849);
  cone_t no_cone_3 = new_cone(1.,2.,vertex,axis);
  
  if (ray_cone_intersection(observer3,no_cone_3,&intersection))
    printf("Test 27 failed: This cone should not intersect the ray\n");
  else printf("Test 27 Passed\n");

  //edge case: a cone behind observer 1
  vertex = new_vector(-2.3,1.7,0.4);
  normal = new_vector(0.8,0.,0.6);
  cone_t no_cone_4 = new_cone(1., 2., vertex, axis);
    
  if (ray_cone_intersection(observer,no_cone_4,&intersection))
    printf("Test 28 failed: This cone should be behind the observer.\n");
  else printf("Test 28 Passed\n");

  //edge case: a cone not in view of observer 1 whose circumscribed cylinder is in view
  vertex = new_vector(0.9,0.,0.4);
  axis = new_vector(0.8,0.,0.6);
  cone_t no_cone_5 = new_cone(2.0, 4., vertex, axis);
  
  if (ray_cone_intersection(observer,no_cone_5,&intersection))
    printf("Test 29 failed: Check to make sure your cone is not a cylinder.\n");
  else printf("Test 29 Passed\n");

  //edge case: observer 3 peering over the rim
  vertex = new_vector(0.8,7.6,10.4832);
  axis = new_vector(0.6,-0.3,-0.741619849);
  cone_t yes_cone_4 = new_cone(1.,2.,vertex,axis);

  vector_t cone_intersection_pt4 = new_vector(2.52752,7.52752,9.52752);
  
  if (!ray_cone_intersection(observer3,yes_cone_4,&intersection))
    printf("Test 30 failed: This cone should intersect the ray.  Check your bounds\n");
  else if (distance(intersection,cone_intersection_pt4) > test_tol)
    printf("Test 30 failed: Intersection point off by %g.  Check your bounds\n",distance(intersection,cone_intersection_pt4));
  else printf("Test 30 Passed\n");

  return 0;
}
