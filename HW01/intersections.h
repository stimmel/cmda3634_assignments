//Define vectors component-wise
typedef struct vector {
  double x;
  double y;
  double z;
} vector;

//Rays are determined by their start point and direction vector
typedef struct ray {
  vector start;
  vector dir;
} ray;
  
//A disk is determined by its radius, center, and normal vector
typedef struct disk {
  double radius;
  vector center;
  vector normal;
} disk;

//A open cylinder is determined by its radius, height, the center of its base,
//and a normal vector to its base
typedef struct cylinder {
  double radius;
  double height;
  vector center;
  vector axis;
} cylinder;

//An open right cone is determined by radius, height, its vertex, and a vector from the vertex normal to the base
typedef struct cone {
  double radius;
  double height;
  vector vertex;
  vector axis;
} cone;

//a sphere is determined by its center and radius
typedef struct sphere {
  double radius;
  vector center;
} sphere;

//utility functions: Basic vector operations
double dot_product(vector a, vector b);
vector cross_product(vector a, vector b);
double distance(vector a, vector b);
double magnitude(vector a);
vector sum(vector a, vector b);
vector difference(vector a, vector b);
vector scaled_sum(double c1, vector a, double c2, vector b);
vector scaled_difference(double c1, vector a, double c2, vector b);
vector scalar_product(double c, vector a);
void copy_vector(vector a, vector *result);

//utility functions: object constructors
vector new_vector(double x, double y, double z);
ray new_ray(vector start, vector dir);
disk new_disk(double radius, vector center, vector normal);
cylinder new_cylinder(double radius, double height, vector center, vector axis);
cone new_cone(double radius, double height, vector vertex, vector axis);
sphere new_sphere(double radius, vector center);

//Object intersections
int ray_sphere_intersection(ray observer, sphere obj, vector *intersection);
int ray_disk_intersection(ray observer, disk obj, vector *intersection);
int ray_cylinder_intersection(ray observer, cylinder obj, vector *intersection);
int ray_cone_intersection(ray observer, cone object, vector *intersection);
