//Define vectors component-wise
typedef struct vector_t {
  double x;
  double y;
  double z;
} vector_t;

//Rays are determined by their start point and direction vector
typedef struct ray_t {
  vector_t start;
  vector_t dir;
} ray_t;
  
//A disk is determined by its radius, center, and normal vector
typedef struct disk_t {
  double radius;
  vector_t center;
  vector_t normal;
} disk_t;

//A open cylinder is determined by its radius, height, the center of its base,
//and a normal vector to its base
typedef struct cylinder_t {
  double radius;
  double height;
  vector_t center;
  vector_t axis;
} cylinder_t;

//An open right cone is determined by radius, height, its vertex, and a vector from the vertex normal to the base
typedef struct cone_t {
  double radius;
  double height;
  vector_t vertex;
  vector_t axis;
} cone_t;

//a sphere is determined by its center and radius
typedef struct sphere_t {
  double radius;
  vector_t center;
} sphere_t;

//utility functions: Basic vector operations
double dot_product(vector_t a, vector_t b);
vector_t cross_product(vector_t a, vector_t b);
double distance(vector_t a, vector_t b);
double magnitude(vector_t a);
vector_t sum(vector_t a, vector_t b);
vector_t difference(vector_t a, vector_t b);
vector_t scaled_sum(double c1, vector_t a, double c2, vector_t b);
vector_t scaled_difference(double c1, vector_t a, double c2, vector_t b);
vector_t scalar_product(double c, vector_t a);
void copy_vector(vector_t a, vector_t *result);

//utility functions: object constructors
vector_t new_vector(double x, double y, double z);
ray_t new_ray(vector_t start, vector_t dir);
disk_t new_disk(double radius, vector_t center, vector_t normal);
cylinder_t new_cylinder(double radius, double height, vector_t center, vector_t axis);
cone_t new_cone(double radius, double height, vector_t vertex, vector_t axis);
sphere_t new_sphere(double radius, vector_t center);

//Object intersections
int ray_sphere_intersection(ray_t observer, sphere_t obj, vector_t *intersection);
int ray_disk_intersection(ray_t observer, disk_t obj, vector_t *intersection);
int ray_cylinder_intersection(ray_t observer, cylinder_t obj, vector_t *intersection);
int ray_cone_intersection(ray_t observer, cone_t object, vector_t *intersection);
