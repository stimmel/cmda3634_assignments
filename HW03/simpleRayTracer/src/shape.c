#include "simpleRayTracer.h"

vector_t shapeComputeNormal(const vector_t v, const shape_t s){
  vector_t n = vectorCreate(0,0,0);
  
  /* Find the normal for this new vector_t at the point of intersection */
  switch(s.type){
  case SPHERE:
    {
      n = vectorSub(v, s.sphere.pos);
      break;
    }
  case TRIANGLE:
    {
      vector_t a = vectorSub(s.triangle.vertices[2], s.triangle.vertices[0]); 
      vector_t b = vectorSub(s.triangle.vertices[1], s.triangle.vertices[0]);
      
      n = vectorCrossProduct(a, b);
      break;
    }
  case CONE:
    {
      // n = (v-vertex) x ( a x (v-vertex) )
      vector_t vMinusVertex = vectorSub(v, s.cone.vertex);

      // axis location
      dfloat H = s.cone.height;
      dfloat z = vectorDot(vMinusVertex, s.cone.axis);
      
      // problematic if axis is parallel to v-Vertex
      if(z>p_projectDelta && z<H-p_projectDelta)
	n = vectorCrossProduct( vMinusVertex, vectorCrossProduct(s.cone.axis, vMinusVertex));

      break;
    }
  case DISK:
    {
      vector_t vMc = vectorSub(v, s.disk.center);
      
      dfloat   R = s.disk.radius;

      vector_t tmp = vectorOrthogonalize(vMc, s.disk.normal);
      
      dfloat z = vectorNorm(tmp);

      if(z<R-p_projectDelta)
	n =  s.disk.normal;

      break;
    }
  case CYLINDER:
    {
      // z = (v - c).a  => clamp

      vector_t vMc = vectorSub(v, s.cylinder.center);

      dfloat   H = s.cylinder.height;
      dfloat z = vectorDot(vMc, s.cylinder.axis);

      if(z>p_projectDelta && z<H-p_projectDelta)
	n = vectorOrthogonalize(vMc, s.cylinder.axis);
      
      break;
    }
  case RECTANGLE:
    {
      n = vectorCrossProduct(s.rectangle.axis[0], s.rectangle.axis[1]);
      break;
    }
  }

  // normalize when normal is not degenerate
  dfloat tmp = vectorNorm(n);
  if(tmp)
    n = vectorScale(1./tmp, n);
  
  return n;
}

material_t shapeComputeMaterial(const int Nmaterials, const material_t *materials,
				const vector_t v, const shape_t s){
  material_t m;
  
  switch(s.type){
  case SPHERE:
  case TRIANGLE:
  case DISK:
  case CYLINDER:
  case CONE:
    {
      m = materials[s.material];
      break;
    }
#if 0
    // extra code that adds stripes to cone and cylinder
  case CYLINDER:
    {
      vector_t c = s.cylinder.center;
      vector_t a = s.cylinder.axis;
      vector_t vMc = vectorSub(v, c);
      dfloat H = s.cylinder.height;
      dfloat h = vectorDot(vMc, a);

      int i = (int) (8.f*(h/H)); // checkerboard material selector
      
      int idM = 10*((i%2)); // 1 if either i is odd or j is even
      m = materials[idM];
      break;
    }
  case CONE:
    {
      vector_t c = s.cone.vertex;
      vector_t a = s.cone.axis;
      vector_t vMc = vectorSub(v, c);
      dfloat H = s.cone.height;
      dfloat h = vectorDot(vMc, a);

      int i = (int) (8.f*(h/H)); // checkerboard material selector
      
      int idM = 20*((i%2)); // 1 if either i is odd or j is even
      m = materials[idM];
      break;
    }
#endif    
  case RECTANGLE:
    {


      vector_t C  = s.rectangle.center;
      vector_t A1 = s.rectangle.axis[0];
      vector_t A2 = s.rectangle.axis[1];
      dfloat   L1 = s.rectangle.length[0];
      dfloat   L2 = s.rectangle.length[1];

      // X = v - C
      vector_t X = vectorSub(v, C);
      
      dfloat h1 = vectorDot(A1, X)+0.5*L1; // shift
      dfloat h2 = vectorDot(A2, X)+0.5*L2; // shift

      int i = (int) (8.f*(h1/L1)); // checkerboard material selector
      int j = (int) (8.f*(h2/L2));
      
      int idM = 2*((i%2) ^ ((j+1)%2)); // 1 if either i is odd or j is even
      //      printf("i=%d, j=%d, h1=%g, h2=%g, L1=%g, L2=%g, idM = %d\n", i, j, h1, h2, L1, L2, idM);
      m = materials[idM];
    }
  }

  return m;
  
}
