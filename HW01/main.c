#include "intersections.h"
#include <math.h>
#include <stdio.h>

int main(int argc, char ** argv) {

  //an observer peering in a random direction
  struct ray observer = {
    .start = {0.1,0.5,1.},
    .dir = {0.8,-0.6,0.}
  };

  //a disk in his way
  struct disk yes_disk = {.radius = 2.5,
			  .center = {0.8,0.,1.2},
			  .normal = {0.96,-0.28,0.}};

  double disk_intersection_pt[3] = {0.794017094,-0.020512821,1.};
  
  //a disk out of the way
  struct disk no_disk_1 = {.radius = 1.,
			   .center = {0.9,-0.1,2.2},
			   .normal = {0.96,0.,-0.28}};

  //a disk behind him
  struct disk no_disk_2 = {.radius = 2.5,
			   .center = {-0.7,1.1,1.},
			   .normal = {0.96,-0.28,0}};

  //a disk parallel to him, but off to the side
  struct disk no_disk_3 = {.radius = 2,
			   .center = {2.5,-1.3,1.2},
			   .normal = {0.,0.,1.}};

  //a cylinder in his way
  struct cylinder yes_cylinder = {.radius = 0.5,
				  .height = 2.,
				  .normal = {0.8,0.,0.6},
				  .center = {0.9,-0.7,0.4}};
  
  double cylinder_intersection_pt[3] = {1.179421,-0.309566,1.};

  //a cylinder not in his way
  struct cylinder no_cylinder_1 = {.radius = 0.5,
				   .height = 2.,
				   .normal = {0.8,0.,0.6},
				   .center = {0.9,1.4,0.3}};

  //a cylinder behind him
  struct cylinder no_cylinder_2 = {.radius = 0.5,
				   .height = 2.,
				   .normal = {0.8,0.,0.6},
				   .center = {-2.3,1.7,0.4}};


  //a cylinder parallel to him
  struct cylinder no_cylinder_3 = {.radius = 0.5,
				   .height = 2.,
				   .center = {0.9,-0.1,1.},
				   .normal = {0.8,-0.6,0.}};
  
  //a cone in his way
  struct cone yes_cone = {.radius = 1.,
			  .height = 2.,
			  .vertex = {0.9,-0.7,0.4},
			  .normal = {0.8,0.,0.6}};

  double cone_intersection_pt[3] = {1.33248,-0.42436,1.};
  
  //a cone behind him
  struct cone no_cone_1 = {.radius = 1,
			   .height = 2.,
			   .normal = {0.8,0.,0.6},
			   .vertex = {-2.3,1.7,0.4}};

  //a cone not in his way
  struct cone no_cone_2 = {.radius = 0.5,
			   .height = 2.,
			   .normal = {0.8,0.,0.6},
			   .vertex = {0.9,1.4,0.3}};

  //a cone barely not in his way (cylinder vs cone)
  struct cone no_cone_3 = {.radius = 2.0,
			   .height = 4.,
			   .vertex = {0.9,0.,0.4},
			   .normal = {0.8,0.,0.6}};


  double intersection[3];
  double test_tol = 1e-4;

  printf("\n\n Disk tests: \n\n");

  if (!ray_disk_intersection(observer,yes_disk,intersection))
    printf("Test 1 failed: This disk should intersect the ray\n");
  else if (distance(intersection,disk_intersection_pt) > test_tol)
    printf("Test 1 failed: Intersection point off by %g\n",distance(intersection,disk_intersection_pt));
  else printf("Test 1 Passed\n");

  if (ray_disk_intersection(observer,no_disk_1,intersection))
    printf("Test 2 failed: This disk should not intersect the ray\n");
  else printf("Test 2 Passed\n");

  if (ray_disk_intersection(observer,no_disk_2,intersection))
    printf("Test 3 failed: This object should be directly behind the observer.\n");
  else printf("Test 3 Passed\n");

  if (ray_disk_intersection(observer,no_disk_3,intersection))
    printf("Test 4 failed: What happens when your disk is parallel?.\n");
  else printf("Test 4 Passed\n");
  
  printf("\n\n Cylinder tests: \n\n");
  
  if (!ray_cylinder_intersection(observer,yes_cylinder,intersection))
    printf("Test 5 failed: This cylinder should intersect the ray\n");
  else if (distance(intersection,cylinder_intersection_pt) > test_tol)
    printf("Test 5 failed: Intersection point off by %g\n",distance(intersection,cylinder_intersection_pt));
  else printf("Test 5 Passed\n");
  
  if (ray_cylinder_intersection(observer,no_cylinder_1,intersection))
    printf("Test 6 failed: This cylinder should not intersect the ray\n");
  else printf("Test 6 Passed\n");

  if (ray_cylinder_intersection(observer,no_cylinder_2,intersection))
    printf("Test 7 failed: This cylinder should be behind the observer.\n");
  else printf("Test 7 Passed\n");

  if (ray_cylinder_intersection(observer,no_cylinder_3,intersection))
    printf("Test 8 failed: Make sure your cylinder is open.\n");
  else printf("Test 8 Passed\n");
  
  printf("\n\n Cone tests: \n\n");
  
  if (!ray_cone_intersection(observer,yes_cone,intersection))
    printf("Test 9 failed: This cone should intersect the ray\n");
  else if (distance(intersection,cone_intersection_pt) > test_tol)
    printf("Test 9 failed: Intersection point off by %g\n",distance(intersection,cone_intersection_pt));
  else printf("Test 9 Passed\n");
  
  if (ray_cone_intersection(observer,no_cone_1,intersection))
    printf("Test 10 failed: This cone should not intersect the ray\n");
  else printf("Test 10 Passed\n");

  if (ray_cone_intersection(observer,no_cone_2,intersection))
    printf("Test 11 failed: This cone should be behind the observer.\n");
  else printf("Test 11 Passed\n");

  if (ray_cone_intersection(observer,no_cone_3,intersection))
    printf("Test 12 failed: Check to make sure your cone is not a cylinder.\n");
  else printf("Test 12 Passed\n");
  
  return 0;
}
