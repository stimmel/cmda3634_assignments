#include <time.h>
#include "simpleRayTracer.h"

vector_t vectorCreate(dfloat x, dfloat y, dfloat z){
  vector_t v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

/* Subtract two vectors and return the resulting vector_t */
vector_t vectorSub(const vector_t v1, const vector_t v2){
  return vectorCreate(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

/* Multiply two vectors and return the resulting scalar (dot product) */
dfloat vectorDot(const vector_t v1, const vector_t v2){
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

vector_t vectorCrossProduct(const vector_t v1, const vector_t v2){
  return vectorCreate(v1.y*v2.z-v1.z*v2.y,
		      v1.z*v2.x-v1.x*v2.z,
		      v1.x*v2.y-v1.y*v2.x);
}


/* Calculate Vector_T x Scalar and return resulting Vector*/ 
vector_t vectorScale(const dfloat c, const vector_t v){
  return vectorCreate(v.x * c, v.y * c, v.z * c);
}

/* Add two vectors and return the resulting vector_t */
vector_t vectorAdd(const vector_t v1, const vector_t v2){
  return vectorCreate(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

dfloat vectorTripleProduct(const vector_t a, const vector_t b, const vector_t c){

  const vector_t aXb = vectorCrossProduct(a, b);
  
  return vectorDot(aXb, c); 
}

// assume b is unit vector
vector_t vectorOrthogonalize(const vector_t a, const vector_t b){

  dfloat adotb = vectorDot(a, b);

  return  vectorSub(a, vectorScale(adotb, b));
}

dfloat vectorNorm(const vector_t a){
  return  sqrt(vectorDot(a,a));
}

// return orthonormalized vector
vector_t vectorNormalize(const vector_t a){

  dfloat d = vectorNorm(a);
  if(d)
    return vectorScale(1./d, a);
  else
    return vectorCreate(0,0,0);
}

dfloat clamp(dfloat x, dfloat xmin, dfloat xmax){

  x = min(x, xmax);
  x = max(x, xmin);
  
  return x;
}


int iclamp(dfloat x, dfloat xmin, dfloat xmax){

  x = min(x, xmax);
  x = max(x, xmin);
  
  return floor(x);
}

static clock_t tic, toc;

void initTimer(){
  tic = 0;
  toc = 0;
}

void ticTimer(){
  
  tic = clock();
}

void tocTimer(const char *message){

  toc = clock();
  
  double elapsed = (toc-tic)/CLOCKS_PER_SEC;

  printf("Kernel %s took %g seconds\n", message, elapsed);
}


// https://www.scratchapixel.com/code.php?id=10&origin=/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes
// roots of a*t^2 + 2*b*t + c = 0
bool solveQuadratic(const dfloat a, const dfloat b, const dfloat c, dfloat *x0, dfloat *x1){
  
  dfloat discr = b * b - a * c;

  if (discr < 0) return false;
  else if (discr == 0) {
    x0[0] = x1[0] = - b / a;
  }
  else {
    dfloat sqrtdiscr = sqrt(discr);
    dfloat q = (b > 0) ?
      -(b + sqrtdiscr) :
      -(b - sqrtdiscr);
    x0[0] = q / a;
    x1[0] = c / q;
  }

  dfloat xmin = min(x0[0], x1[0]);
  dfloat xmax = max(x0[0], x1[0]);
  x0[0] = xmin;
  x1[0] = xmax;
  
  return true; 
}

