//Rays are determined by their start point and direction vector
struct ray {
  double start[3];
  double dir[3];
};
  
//A disk is determined by its radius, center, and normal vector
struct disk {
  double radius;
  double center[3];
  double normal[3];
};

//A open cylinder is determined by its radius, height, the center of its base,
//and a normal vector to its base
struct cylinder {
  double radius;
  double height;
  double center[3];
  double normal[3];
};

//An open right cone is determined by radius, height, its vertex, and a vector from the vertex normal to the base
struct cone {
  double radius;
  double height;
  double vertex[3];
  double normal[3];
};

//utility functions
double dot_product(double *a, double *b);
void cross_product(double *a, double *b, double *result);
double distance(double *a, double *b);
double magnitude(double *a);
void sum(double *a, double *b, double *result);
void difference(double *a, double *b, double *result);
void scaled_sum(double c1, double *a, double c2, double *b, double *result);
void scaled_difference(double c1, double *a, double c2, double *b, double *result);
void scalar_product(double c, double *a, double *result);
void copy_vector(double *a, double *result);

//Object intersections
int ray_disk_intersection(struct ray observer, struct disk obj, double *intersection);
int ray_cylinder_intersection(struct ray observer, struct cylinder obj, double *intersection);
int ray_cone_intersection(struct ray observer, struct cone object, double *intersection);
