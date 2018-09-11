#include "simpleRayTracer.h"

bbox_t createBoundingBoxTriangle(triangle_t triangle){

  bbox_t bbox;

  bbox.xmin = min(triangle.vertices[0].x, min(triangle.vertices[1].x, triangle.vertices[2].x));
  bbox.xmax = max(triangle.vertices[0].x, max(triangle.vertices[1].x, triangle.vertices[2].x));

  bbox.ymin = min(triangle.vertices[0].y, min(triangle.vertices[1].y, triangle.vertices[2].y));
  bbox.ymax = max(triangle.vertices[0].y, max(triangle.vertices[1].y, triangle.vertices[2].y));
  
  bbox.zmin = min(triangle.vertices[0].z, min(triangle.vertices[1].z, triangle.vertices[2].z));
  bbox.zmax = max(triangle.vertices[0].z, max(triangle.vertices[1].z, triangle.vertices[2].z));

  return bbox;
}

bbox_t createBoundingBoxSphere(sphere_t sphere){

  bbox_t bbox;

  bbox.xmin = sphere.pos.x - sphere.radius;
  bbox.xmax = sphere.pos.x + sphere.radius;

  bbox.ymin = sphere.pos.y - sphere.radius;
  bbox.ymax = sphere.pos.y + sphere.radius;

  bbox.zmin = sphere.pos.z - sphere.radius;
  bbox.zmax = sphere.pos.z + sphere.radius;

  return bbox;
}

bbox_t createBoundingBoxCylinder(cylinder_t cylinder){

  bbox_t bbox;

  vector_t c = cylinder.center;
  vector_t a = cylinder.axis;
  dfloat   R = cylinder.radius;
  dfloat   H = cylinder.height;

  // xmax = c.x + (H/2)*a.x + (H/2)*|a.x| + R*|cross(e_x,a) |
  bbox.xmax = c.x + (H/2)*a.x + (H/2)*fabs(a.x) + R*sqrtf(a.y*a.y + a.z*a.z);
  bbox.xmin = c.x + (H/2)*a.x - (H/2)*fabs(a.x) - R*sqrtf(a.y*a.y + a.z*a.z);

  bbox.ymax = c.y + (H/2)*a.y + (H/2)*fabs(a.y) + R*sqrtf(a.x*a.x + a.z*a.z);
  bbox.ymin = c.y + (H/2)*a.y - (H/2)*fabs(a.y) - R*sqrtf(a.x*a.x + a.z*a.z);

  bbox.zmax = c.z + (H/2)*a.z + (H/2)*fabs(a.z) + R*sqrtf(a.x*a.x + a.y*a.y);
  bbox.zmin = c.z + (H/2)*a.z - (H/2)*fabs(a.z) - R*sqrtf(a.x*a.x + a.y*a.y);

  return bbox;
}

bbox_t createBoundingBoxCone(cone_t cone){

  bbox_t bbox;

  vector_t v = cone.vertex;
  vector_t a = cone.axis;
  dfloat   R = cone.radius;
  dfloat   H = cone.height;

  bbox.xmax = max(v.x, v.x + H*a.x + R*sqrtf(a.y*a.y + a.z*a.z));
  bbox.xmin = min(v.x, v.x + H*a.x - R*sqrtf(a.y*a.y + a.z*a.z));

  bbox.ymax = max(v.y, v.y + H*a.y + R*sqrtf(a.x*a.x + a.z*a.z));
  bbox.ymin = min(v.y, v.y + H*a.y - R*sqrtf(a.x*a.x + a.z*a.z));

  bbox.zmax = max(v.z, v.z + H*a.z + R*sqrtf(a.x*a.x + a.y*a.y));
  bbox.zmin = min(v.z, v.z + H*a.z - R*sqrtf(a.x*a.x + a.y*a.y));
  
  return bbox;
}



bbox_t createBoundingBoxRectangle(rectangle_t rectangle){

  bbox_t bbox;

  vector_t C = rectangle.center;
  vector_t A1 = rectangle.axis[0];
  vector_t A2 = rectangle.axis[1];
  vector_t n = vectorCrossProduct(A1, A2);

  dfloat   L1 = rectangle.length[0];
  dfloat   L2 = rectangle.length[1];
  A1 = vectorScale(L1/2., A1);
  A2 = vectorScale(L2/2., A2);
  
  dfloat delta = 1e-1;
  vector_t dn = vectorScale(delta, n);
  vector_t Cdown = vectorSub(C, dn);
  vector_t Cup = vectorAdd(C, dn);

  vector_t v[8];
  
  // C - delta*n + A1*(-L1/2) + A2*(-L2/2)
  v[0] = vectorSub(Cdown, vectorAdd(A1, A2));
  // C - delta*n + A1*(+L1/2) + A2*(-L2/2)
  v[1] = vectorAdd(Cdown, vectorSub(A1, A2));
  // C - delta*n + A1*(+L1/2) + A2*(+L2/2)
  v[2] = vectorAdd(Cdown, vectorAdd(A1, A2));
  // C - delta*n + A1*(-L1/2) + A2*(+L2/2)
  v[3] = vectorAdd(Cdown, vectorSub(A2, A1));

  // C + delta*n + A1*(-L1/2) + A2*(-L2/2)
  v[4] = vectorSub(Cup, vectorAdd(A1, A2));
  // C + delta*n + A1*(+L1/2) + A2*(-L2/2)
  v[5] = vectorAdd(Cup, vectorSub(A1, A2));
  // C + del6a*n + A1*(+L1/2) + A2*(+L2/2)
  v[6] = vectorAdd(Cup, vectorAdd(A1, A2));
  // C + delta*n + A1*(-L1/2) + A2*(+L2/2)
  v[7] = vectorAdd(Cup, vectorSub(A2, A1));

  bbox.xmin = 1e9;
  bbox.ymin = 1e9;
  bbox.zmin = 1e9;
  bbox.xmax = -1e9;
  bbox.ymax = -1e9;
  bbox.zmax = -1e9;

  for(int n=0;n<8;++n){ // bound over all vertices
    bbox.xmin = min(bbox.xmin, v[n].x);
    bbox.ymin = min(bbox.ymin, v[n].y);
    bbox.zmin = min(bbox.zmin, v[n].z);
    bbox.xmax = max(bbox.xmax, v[n].x);
    bbox.ymax = max(bbox.ymax, v[n].y);
    bbox.zmax = max(bbox.zmax, v[n].z);
  }

  return bbox;
}

bbox_t createBoundingBoxDisk(disk_t disk){

  bbox_t bbox;

  vector_t n = disk.normal;
  vector_t c = disk.center;
  dfloat   R = disk.radius;
  dfloat   H = .1; // assert thickness in normal

  // xmax = c.x + (H/2)*a.x + (H/2)*|a.x| + R*|cross(e_x,a) |
  bbox.xmax = c.x  + (H/2)*fabs(n.x) + R*sqrtf(n.y*n.y + n.z*n.z);
  bbox.xmin = c.x  - (H/2)*fabs(n.x) - R*sqrtf(n.y*n.y + n.z*n.z);

  bbox.ymax = c.y  + (H/2)*fabs(n.y) + R*sqrtf(n.x*n.x + n.z*n.z);
  bbox.ymin = c.y  - (H/2)*fabs(n.y) - R*sqrtf(n.x*n.x + n.z*n.z);

  bbox.zmax = c.z  + (H/2)*fabs(n.z) + R*sqrtf(n.x*n.x + n.y*n.y);
  bbox.zmin = c.z  - (H/2)*fabs(n.z) - R*sqrtf(n.x*n.x + n.y*n.y);

  return bbox;
}

// compute bounding box for one of supported shapes
bbox_t createBoundingBoxShape(const grid_t grid, shape_t shape){

  bbox_t bbox;

  switch(shape.type){
  case TRIANGLE:    bbox = createBoundingBoxTriangle(shape.triangle);   break;
  case SPHERE:      bbox = createBoundingBoxSphere(shape.sphere);       break;
  case RECTANGLE:   bbox = createBoundingBoxRectangle(shape.rectangle); break;
  case CYLINDER:    bbox = createBoundingBoxCylinder(shape.cylinder);   break;
  case DISK:        bbox = createBoundingBoxDisk(shape.disk);           break;
  case CONE:        bbox = createBoundingBoxCone(shape.cone);           break;
  }

  int imin = (grid.invdx*(bbox.xmin-grid.xmin));
  int imax = (grid.invdx*(bbox.xmax-grid.xmin));
  int jmin = (grid.invdy*(bbox.ymin-grid.ymin));
  int jmax = (grid.invdy*(bbox.ymax-grid.ymin));
  int kmin = (grid.invdz*(bbox.zmin-grid.zmin));
  int kmax = (grid.invdz*(bbox.zmax-grid.zmin));
  
  bbox.imin = iclamp(imin, 0, grid.NI-1);
  bbox.imax = iclamp(imax, 0, grid.NI-1);

  bbox.jmin = iclamp(jmin, 0, grid.NJ-1);
  bbox.jmax = iclamp(jmax, 0, grid.NJ-1);

  bbox.kmin = iclamp(kmin, 0, grid.NK-1);
  bbox.kmax = iclamp(kmax, 0, grid.NK-1);
  
  return bbox;
}
