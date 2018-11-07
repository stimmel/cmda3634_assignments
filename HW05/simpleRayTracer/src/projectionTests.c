#include "simpleRayTracer.h"

/* find closest point on rectangle to a point */
dfloat projectPointRectangle(const vector_t p, const rectangle_t rect, vector_t *closest){

  vector_t C  = rect.center;
  vector_t A1 = rect.axis[0];
  vector_t A2 = rect.axis[1];
  dfloat   L1 = rect.length[0];
  dfloat   L2 = rect.length[1];

  // project p onto rectangle plane
  // C + ((p-C).A1)*A1 + ((p-C).A2)*A2)  clamped
  vector_t pMC = vectorSub(p, C);

  dfloat u = vectorDot(pMC, A1)/L1;
  dfloat v = vectorDot(pMC, A2)/L2;
  
  u = L1*clamp(u, -0.5f, 0.5f);
  v = L2*clamp(v, -0.5f, 0.5f);

  *closest = vectorAdd(C, vectorAdd(vectorScale(u, A1), vectorScale(v, A2)));

  dfloat dist = vectorNorm(vectorSub(*closest, p));
  
  return dist;
}


/* find closest point on disk to a point */
dfloat projectPointDisk(const vector_t p, const disk_t disk, vector_t *closest){

  vector_t C = disk.center;
  vector_t n = disk.normal;
  dfloat   r = disk.radius;

  // project p onto rectangle plane
  vector_t pMC = vectorSub(p, C);
  // d =  (p-C) - ((p-C).n)n
  // d <= r*d/||d||
  // closest = C + d

  vector_t d = vectorOrthogonalize(pMC, n);

  dfloat normd = vectorNorm(d);
  dfloat scale = (normd>r) ? r/normd : 1;

  d = vectorScale(scale, d);
  
  *closest = vectorAdd(C, d);
  
  dfloat dist = vectorNorm(vectorSub(*closest, p));
  
  return dist;
}


/* find closest point on an open, finite cylinder to a point */
/* !!!!! */
/* not well defined for points on axis */
dfloat projectPointCylinder(const vector_t p, const cylinder_t cylinder, vector_t *closest){

  vector_t c = cylinder.center; // center of base
  vector_t a = cylinder.axis;
  dfloat   R = cylinder.radius;
  dfloat   H = cylinder.height;

  // z = (p - c).a  => clamp
  vector_t pMc = vectorSub(p, c);

  // axis location
  dfloat z = clamp(vectorDot(pMc, a), 0.f, H);
  
  // plane location
  // d = p - (c + z*a)
  vector_t cPza = vectorAdd(c, vectorScale(z, a));
  vector_t d = vectorSub(p, cPza);
  dfloat normd = vectorNorm(d) + 1e-6;
  dfloat scale = R/normd; 

  // closest = c + z*a + scale*d;
  *closest = vectorAdd(cPza, vectorScale(scale,d));
  
  dfloat dist = vectorNorm(vectorSub(*closest, p));
  
  return dist;
}


dfloat projectPointCone(vector_t p, const cone_t cone, vector_t *closest){

  vector_t v = cone.vertex; // vertex
  vector_t a0 = cone.axis;
  dfloat   R = cone.radius;
  dfloat   H = cone.height;
  
  vector_t pMv = vectorSub(p, v);

  // compute coordinates in a, ((p-v)xa)x
  vector_t a1 = vectorNormalize(vectorCrossProduct(pMv, a0)); // generates zero vectors if colinear
  vector_t a2 = vectorNormalize(vectorCrossProduct(a0,  a1));

  // project onto coordinate system
  dfloat h = vectorDot(pMv, a0);
  dfloat z = vectorDot(pMv, a1);
  dfloat r = vectorDot(pMv, a2);

  if(h<=0 && fabs(r)<R*fabs(h)/H){
    *closest = v;
    return vectorNorm(pMv);
  }

  // do left-half r-h plane using symmetry
  int reflectFlag = 0;
  if(r<0) {
    r *= -1;
    reflectFlag = 1;
  }

  dfloat rI, zI, hI;

  // top of cone divider  r+ R*h/H = 2*R
  if(r>=(2.f*R - (R*h/H))){
    rI = R; // rim
    zI = z;
    hI = H;
  }
  else{
    dfloat s = (R*r+H*h)/(H*H+R*R);

    rI = R*s;
    zI = z;
    hI = H*s;
  }

  if(reflectFlag == 1)
    rI = -rI;

  vector_t  cl = v;

  cl = vectorAdd(cl, vectorScale(hI, a0));
  cl = vectorAdd(cl, vectorScale(zI, a1));
  cl = vectorAdd(cl, vectorScale(rI, a2));

  *closest = cl;
  dfloat dist = vectorNorm(vectorSub(cl,p));

  return dist;

}


// http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm

dfloat projectPointTriangle(vector_t p, const triangle_t triangle, vector_t *closest){

  vector_t v0 = triangle.vertices[0];
  vector_t v1 = triangle.vertices[1];
  vector_t v2 = triangle.vertices[2];
  
  vector_t v01 = vectorSub(v0,v1);
  vector_t v02 = vectorSub(v0,v2);
  vector_t v12 = vectorSub(v1,v2);

  vector_t p0 = vectorSub(p,v0);
  vector_t p1 = vectorSub(p,v1);
  vector_t p2 = vectorSub(p,v2);
  
  // project onto plane containing triangle
  // this is a least squares projection into barycentric coordinates
  // LS:  p_{perp} = L0*v0 + L1*v1 + L2*v2  [ L2 = 1-L0-L1 ]
  //               = L0*(v0-v2) + L1*(v1-v2) + v2
  //      p_{perp} - v2 = L0*(v0-v2) + L1*(v1-v2)
  dfloat A00 =  vectorDot(v02, v02);
  dfloat A01 =  vectorDot(v02, v12);
  dfloat A11 =  vectorDot(v12, v12);

  dfloat R0 =  vectorDot(v02, p2);
  dfloat R1 = -vectorDot(v12, p2);

  dfloat J = A00*A11-A01*A01;
  dfloat L0 = ( A11*R0 - A01*R1)/J;
  dfloat L1 = (-A01*R0 + A00*R1)/J;
  dfloat L2 = 1.f-L0-L1;

  vector_t cl;
  dfloat dist;

  vector_t p2d = vectorScale(L0, v0);
  p2d = vectorAdd(p2d, vectorScale(L1, v1));
  p2d = vectorAdd(p2d, vectorScale(L2, v2));
  
  if(L0>=0 && L1>=0 && L2>=0){ // projected point is in triangle

    dist = vectorNorm(vectorSub(p2d,p));
  }
  else{     // projected point is outside triangle
    
    // compute param for intersection on each edge
    vector_t d0 = vectorSub(p2d,v0);
    vector_t d1 = vectorSub(p2d,v1);
    vector_t d2 = vectorSub(p2d,v2);

    dfloat e0 =  vectorDot(d1, v01)/vectorDot(v01,v01);
    dfloat e1 =  vectorDot(d2, v12)/vectorDot(v12,v12);
    dfloat e2 =  vectorDot(d2, v02)/vectorDot(v02,v02);
    
    e0 = clamp(e0,0,1);
    e1 = clamp(e1,0,1);
    e2 = clamp(e2,0,1);

    vector_t c0 = vectorAdd(v1, vectorScale(e0, v01));
    vector_t c1 = vectorAdd(v2, vectorScale(e1, v12));
    vector_t c2 = vectorAdd(v2, vectorScale(e2, v02));

    dfloat dist0 = vectorNorm(vectorSub(c0,p));
    dfloat dist1 = vectorNorm(vectorSub(c1,p));
    dfloat dist2 = vectorNorm(vectorSub(c2,p));

    if(dist0<=dist1 && dist0<=dist2){
      p2d = c0;
      dist = dist0;
    }else if(dist1<=dist0 && dist1<=dist2){
      p2d = c1;
      dist = dist1;
    }else{
      p2d = c2;
      dist = dist2;
    }
  }
  
  *closest = p2d;

  return dist;
}


dfloat projectPointShape(const vector_t p, const shape_t shape, vector_t *closest){

  switch(shape.type){
  case DISK:      return projectPointDisk(p, shape.disk, closest);
  case RECTANGLE: return projectPointRectangle(p, shape.rectangle, closest);
  case CYLINDER:  return projectPointCylinder(p, shape.cylinder, closest);
  case CONE:      return projectPointCone(p, shape.cone, closest);
  case TRIANGLE:  return projectPointTriangle(p, shape.triangle, closest);
  }

  return 1000000;
}

