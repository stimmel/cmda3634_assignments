#include "simpleRayTracer.h"

void renderKernel2(const int NI,
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

  #pragma omp parallel
  for(int K=omp_get_thread_num();K<NI*NJ;K+=omp_get_num_threads()){
    if (omp_get_thread_num() == 0) continue;
    int I = K%NI;
    int J = K/NI;
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
      
      dfloat cx = BOXSIZE/2., cy =  HEIGHT, cz = BOXSIZE/2;
      
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
	dfloat offI = p_apertureRadius;
	dfloat offJ = p_apertureRadius; 
	
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

      if (c.red*255.0f < 1 && c.green*255.0f < 1 && c.blue*255.0f < 1) {
	c.red = 1.1f/255;
	c.green = 1.1f/255;
	c.blue = 1.1f/255;
      }
      
      // store pixel rgb intensities (reverse vertical because of lensing)
      img[(I + (NJ-1-J)*NI)*3 + 0] = (unsigned char)min(  c.red*255.0f, 255.0f);
      img[(I + (NJ-1-J)*NI)*3 + 1] = (unsigned char)min(c.green*255.0f, 255.0f);
      img[(I + (NJ-1-J)*NI)*3 + 2] = (unsigned char)min( c.blue*255.0f, 255.0f);
  }
}

void interpolateScene(const int NI,
		      const int NJ,
		      unsigned char *img) {
  for (int J = 0; J < NJ; ++J) {
    for (int I = 0; I < NI; ++I) {

      unsigned char this_red = img[(I + (NJ-1-J)*NI)*3 + 0];
      unsigned char this_green = img[(I + (NJ-1-J)*NI)*3 + 1];
      unsigned char this_blue = img[(I + (NJ-1-J)*NI)*3 + 2];

      if (this_red != 0 || this_green != 0 || this_blue != 0) continue;
      
      unsigned char up_red = 0;
      unsigned char up_green = 0;
      unsigned char up_blue = 0;

      unsigned char down_red = 0;
      unsigned char down_green = 0;
      unsigned char down_blue = 0;

      unsigned char left_red = 0;
      unsigned char left_green = 0;
      unsigned char left_blue = 0;

      unsigned char right_red = 0;
      unsigned char right_green = 0;
      unsigned char right_blue = 0;

      if (I > 0) {
	left_red = img[(I -1+ (NJ-1-J)*NI)*3 + 0];
        left_green = img[(I -1+ (NJ-1-J)*NI)*3 + 1];
	left_blue = img[(I -1+ (NJ-1-J)*NI)*3 + 2];
      }
      if (I < NI - 1) {
	right_red = img[(I + 1 + (NJ-1-J)*NI)*3 + 0];
	right_green = img[(I + 1 +(NJ-1-J)*NI)*3 + 1];
	right_blue = img[(I + 1 +(NJ-1-J)*NI)*3 + 2];
      }
      if (J < NJ - 1) {
	down_red = img[(I + (NJ-J)*NI)*3 + 0];
	down_green = img[(I + (NJ-J)*NI)*3 + 1];
	down_blue = img[(I + (NJ-J)*NI)*3 + 2];
      }
      if (J > 0) {
	up_red = img[(I + (NJ-2-J)*NI)*3 + 0];
	up_green = img[(I + (NJ-2-J)*NI)*3 + 1];
	up_blue = img[(I + (NJ-2-J)*NI)*3 + 2];
      }

      int nnz = 4;
      if (left_red == 0 && left_green == 0 && left_blue == 0) nnz--;
      if (right_red == 0 && right_green == 0 && right_blue == 0) nnz--;
      if (up_red == 0 && up_green == 0 && up_blue == 0) nnz--;
      if (down_red == 0 && down_green == 0 && down_blue == 0) nnz--;
      if (nnz == 0) continue;
      
      img[(I + (NJ-1-J)*NI)*3 + 0] = (left_red + right_red + up_red + down_red)/nnz;
      img[(I + (NJ-1-J)*NI)*3 + 1] = (left_green + right_green + up_green + down_green)/nnz;
      img[(I + (NJ-1-J)*NI)*3 + 2] = (left_blue + right_blue + up_blue + down_blue)/nnz;
    }
  }
}
