#include "simpleRayTracer.h"

void renderKernel(const int NI,
		  const int NJ,
		  scene_t scene,
		  const sensor_t sensor,
		  const dfloat costheta,
		  const dfloat sintheta,
		  const dfloat *randomNumbers,
		  unsigned char *img){
  
  const colour_t bg = sensor.bg;

  // unpack contents of scene
  grid_t     *grid      = scene.grid;
  material_t *materials = scene.materials;
  shape_t    *shapes    = scene.shapes;
  light_t    *lights    = scene.lights;

  const int Nlights = scene.Nlights;
  const int Nmaterials = scene.Nmaterials;
  const int Nshapes    = scene.Nshapes;

  // (I,J) loop over pixels in image

  for(int J=0;J<NJ;++J){
    for(int I=0;I<NI;++I){
      ray_t r;
      
      dfloat coef = 1.0;
      int level = 0;
      
      // 2.5 location of sensor pixel
      colour_t c;
      
      dfloat x0 = sensor.eyeX.x;
      dfloat y0 = sensor.eyeX.y;
      dfloat z0 = sensor.eyeX.z;
      
      // multiple rays emanating from sensor, passing through lens and focusing at the focal plane
      // 1. compute intersection of ray passing through lens center to focal plane
      
      // (sensorX + alpha*(lensC -sensorX)).sensorN = focalPlaneOffset
      // alpha = (focalOffset-s.sensorN)/( (lensC-s).sensorN) [ . dot product ]
      
      dfloat cx = BOXSIZE/2., cy =  BOXSIZE/2., cz = BOXSIZE/2;
      
      vector_t sensorN = vectorCrossProduct(sensor.Idir, sensor.Jdir);
      vector_t sensorX = sensorLocation(NI, NJ, I, J, sensor);
      dfloat   focalPlaneOffset = sensor.focalPlaneOffset;
      vector_t centralRayDir = vectorSub(sensor.lensC, sensorX);
      dfloat alpha = (focalPlaneOffset - vectorDot(sensorX, sensorN))/vectorDot(centralRayDir, sensorN);
      
      // 2. target
      vector_t targetX = vectorAdd(sensorX, vectorScale(alpha, centralRayDir));
      
      x0 = sensorX.x;
      y0 = sensorX.y;
      z0 = sensorX.z;
      
      // 3.  loop over vertical offsets on lens (thin lens)
      c.red = 0; c.green = 0; c.blue = 0;
      
      for(int samp=0;samp<p_Nsamples;++samp){

	// aperture width
	int sampId = (I+J*NI + samp*25*25)%NRANDOM;
	dfloat offI = randomNumbers[2*sampId+0]*p_apertureRadius;
	dfloat offJ = randomNumbers[2*sampId+1]*p_apertureRadius; 
	
	// choose random starting point on lens (assumes lens and sensor arre parallel)
	if(samp>0) { // primary ray
	  x0 = sensor.lensC.x + offI*sensor.Idir.x + offJ*sensor.Jdir.x;
	  y0 = sensor.lensC.y + offI*sensor.Idir.y + offJ*sensor.Jdir.y;
	  z0 = sensor.lensC.z + offI*sensor.Idir.z + offJ*sensor.Jdir.z;
	}
	
	dfloat dx0 = targetX.x - x0;
	dfloat dy0 = targetX.y - y0;
	dfloat dz0 = targetX.z - z0;
	
	dfloat L0 = sqrt(dx0*dx0+dy0*dy0+dz0*dz0);
	dx0 = dx0/L0;
	dy0 = dy0/L0;
	dz0 = dz0/L0;
	
	r.start.x = costheta*(x0-cx) - sintheta*(z0-cz) + cx;
	r.start.y = y0;
	r.start.z = sintheta*(x0-cx) + costheta*(z0-cz) + cz;
	
	r.dir.x = costheta*dx0 - sintheta*dz0;
	r.dir.y = dy0;
	r.dir.z = sintheta*dx0 + costheta*dz0;

	// trace ray through scene (possibly with multipathing, reflection, refraction)
	colour_t newc =
	  gridTrace(grid[0], Nshapes, shapes, Nlights, lights, Nmaterials, materials, r, level, coef, bg);

	// add colors to final intensity for IJ pixel
	dfloat sc = (samp==0) ? p_primaryWeight: 1.f;
	c.red   += sc*newc.red;
	c.green += sc*newc.green;
	c.blue  += sc*newc.blue;
	
      }
      
      // primary weighted average
      c.red   /= (p_primaryWeight+p_Nsamples-1);
      c.green /= (p_primaryWeight+p_Nsamples-1);
      c.blue  /= (p_primaryWeight+p_Nsamples-1);
      
      // store pixel rgb intensities (reverse vertical because of lensing)
      img[(I + (NJ-1-J)*NI)*3 + 0] = (unsigned char)min(  c.red*255.0f, 255.0f);
      img[(I + (NJ-1-J)*NI)*3 + 1] = (unsigned char)min(c.green*255.0f, 255.0f);
      img[(I + (NJ-1-J)*NI)*3 + 2] = (unsigned char)min( c.blue*255.0f, 255.0f);
    }  
  }
}
