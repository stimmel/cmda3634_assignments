#include "simpleRayTracer.h"

vector_t sensorLocation(const int NI,
			const int NJ,
			const int I,
			const int J,
			const sensor_t sensor){
  
  vector_t sensorX = sensor.eyeX;
  
  dfloat r = I/(dfloat)(NI-1);
  dfloat s = J/(dfloat)(NJ-1);
  
  r = (r-0.5f)*sensor.Ilength;
  s = (s-0.5f)*sensor.Jlength;

  vector_t sensorNormal =
    vectorCrossProduct(sensor.Idir, sensor.Jdir);
  
  sensorX.x += sensorNormal.x*sensor.offset;
  sensorX.y += sensorNormal.y*sensor.offset;
  sensorX.z += sensorNormal.z*sensor.offset;

  sensorX.x += r*sensor.Idir.x;
  sensorX.y += r*sensor.Idir.y;
  sensorX.z += r*sensor.Idir.z;
  
  sensorX.x += s*sensor.Jdir.x;
  sensorX.y += s*sensor.Jdir.y;
  sensorX.z += s*sensor.Jdir.z;

  return sensorX;
}

void sensorMultipleLocations(const int NI,
			     const int NJ,
			     const int I,
			     const int J,
			     const sensor_t &sensor, 
			     vector_t *sensorsX){
  
  for(int samp=0;samp<p_Nsamples;++samp){
    sensorsX[samp] = sensor.eyeX;
    
    dfloat r = I/(dfloat)(NI-1);
    dfloat s = J/(dfloat)(NJ-1);

    dfloat theta = 2.f*M_PI*samp/(dfloat)p_Nsamples;
    
    // circle of samples around sensor pixel
    dfloat delta = .5; // scatter pixel radius
    r = (r-0.5f+delta*cosf(theta)/NI)*sensor.Ilength;
    s = (s-0.5f+delta*sinf(theta)/NJ)*sensor.Jlength;

    vector_t sensorNormal = vectorCrossProduct(sensor.Idir, sensor.Jdir);

    sensorsX[samp].x += sensorNormal.x*sensor.offset;
    sensorsX[samp].y += sensorNormal.y*sensor.offset;
    sensorsX[samp].z += sensorNormal.z*sensor.offset;
    
    sensorsX[samp].x += r*sensor.Idir.x;
    sensorsX[samp].y += r*sensor.Idir.y;
    sensorsX[samp].z += r*sensor.Idir.z;

    sensorsX[samp].x += s*sensor.Jdir.x;
    sensorsX[samp].y += s*sensor.Jdir.y;
    sensorsX[samp].z += s*sensor.Jdir.z;
  }

}

