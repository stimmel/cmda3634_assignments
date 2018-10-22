#include "simpleRayTracer.h"

// grid search
bool gridRayIntersectionSearch(const ray_t r,
			       const int Nshapes, const shape_t *shapes, const  grid_t grid,
			       dfloat *t, int *currentShape){

  // is start of ray in a grid cell ?
  vector_t s = r.start; // will modify ray through s
  vector_t d = r.dir;
    
  // if ray is outside grid then project onto grid
  if(s.x<grid.xmin){
    if(d.x<=0) return false; // pointing away or grazing from grid
    dfloat t0 = -(s.x-grid.xmin)/d.x;
    s.x = grid.xmin;
    s.y += t0*d.y;
    s.z += t0*d.z;
  }

  if(s.x>grid.xmax){
    if(d.x>=0) return false; 
    dfloat t0 = -(s.x-grid.xmax)/d.x;
    s.x = grid.xmax;
    s.y += t0*d.y;
    s.z += t0*d.z;
  }

  if(s.y<grid.ymin){
    if(d.y<=0) return false; // pointing away or grazing from grid
    dfloat t0 = -(s.y-grid.ymin)/d.y;
    s.y = grid.ymin;
    s.x += t0*d.x;
    s.z += t0*d.z;
  }

  if(s.y>grid.ymax){
    if(d.y>=0) return false; 
    dfloat t0 = -(s.y-grid.ymax)/d.y;
    s.y = grid.ymax;
    s.x += t0*d.x;
    s.z += t0*d.z;
  }

  if(s.z<grid.zmin){
    if(d.z<=0) return false; // pointing away or grazing from grid
    dfloat t0 = -(s.z-grid.zmin)/d.z;
    s.z = grid.zmin;
    s.x += t0*d.x;
    s.y += t0*d.y;
  }

  if(s.z>grid.zmax){
    if(d.z>=0) return false; 
    dfloat t0 = -(s.z-grid.zmax)/d.z;
    s.z = grid.zmax;
    s.x += t0*d.x;
    s.y += t0*d.y;
  }
  
  // now the ray start must be on the surface of the grid or in a cell

  int cellI = iclamp((s.x-grid.xmin)*grid.invdx,0,grid.NI-1); // assumes grid.NI
  int cellJ = iclamp((s.y-grid.ymin)*grid.invdy,0,grid.NJ-1);
  int cellK = iclamp((s.z-grid.zmin)*grid.invdz,0,grid.NK-1);
  
  ray_t newr = r;
  newr.start = s;
  newr.dir = d;

  *currentShape = -1;
  
  do{
    int cellID = cellI + grid.NI*cellJ + grid.NI*grid.NJ*cellK;
    
    *t = 20000; // TW ?

    int start = grid.boxStarts[cellID];
    int end   = grid.boxStarts[cellID+1];
    for(int offset=start;offset<end;++offset){
      const int obj = grid.boxContents[offset];
      const shape_t shape = shapes[obj];
      if(intersectRayShape(r, shape, t)){
	vector_t intersect = vectorAdd(r.start, vectorScale(*t, r.dir));
	
	if(intersectPointGridCell(grid, intersect, cellI, cellJ, cellK)){
	  *currentShape = obj;
	}
      }
    }
    
    if(*currentShape != -1){
      return true;
    }
    
    // find faces that ray passes through
    unsigned int face = intersectRayBox(&newr,grid.bboxes[cellID]);
    
    if(face&1) --cellK; // face 0
    if(face&2) --cellJ; // face 1
    if(face&4) ++cellI; // face 2
    if(face&8) ++cellJ; // face 3
    if(face&16) --cellI;// face 4
    if(face&32) ++cellK;// face 5

    if(face==0){
      break;
    }
  }while(cellI>=0 && cellI<grid.NI &&
	 cellJ>=0 && cellJ<grid.NJ &&
	 cellK>=0 && cellK<grid.NK);

  return false;
}

colour_t gridTrace(const grid_t grid,
		   const int Nshapes,
		   const shape_t *shapes,
		   const int Nlights,
		   const light_t *lights,
		   const int Nmaterials,
		   const material_t *materials,
		   ray_t  r,
		   int    level,
		   dfloat coef,
		   colour_t bg){
  
  colour_t black;
  black.red = 0;
  black.green = 0;
  black.blue = 0;

  // initialize color as black
  colour_t c = black;
  
  int Nrays = 0, rayID = 0;
  ray_t rayStack[p_maxNrays];

  // add initial ray to stack
  rayID = 0;
  r.level = 0;
  r.coef = coef;
  rayStack[Nrays] = r;
  ++Nrays;

  // keep looping until the stack is exhausted or the maximum number of rays is reached
  while(rayID<Nrays && Nrays<p_maxNrays){

    // get ray
    r = rayStack[rayID];
    
    // look for intersection of this ray with shapes
    int currentShapeID = -1;
    dfloat t = 20000.f;

    // look through grid to find intersections with ray
    gridRayIntersectionSearch(r, Nshapes, shapes, grid, &t, &currentShapeID);
    
    // none found
    if(currentShapeID == -1){
      if(rayID==0)
	c = bg;

      // go to next ray
      ++rayID;
      continue;
    }

    // shape at nearest ray intersection
    shape_t currentShape = shapes[currentShapeID];
    
    // compute intersection location
    vector_t intersection = vectorAdd(r.start, vectorScale(t, r.dir));
    
    // find unit surface normal
    vector_t n = shapeComputeNormal(intersection, currentShape);
    
    /* use shadow tracing to determine color contribution from this intersection */
    dfloat rdotn = vectorDot(r.dir, n);

    /* Find the material to determine the colour */
    material_t currentMat = shapeComputeMaterial(Nmaterials, materials, intersection, currentShape);

    // test for reflection
    info_t info = currentMat.info;

    if(info.emitter==1){
      dfloat lambert = rdotn * r.coef; 
      c.red   += lambert * currentMat.diffuse.red;
      c.green += lambert * currentMat.diffuse.green;
      c.blue  += lambert * currentMat.diffuse.blue;
    }
    else{

      if(info.reflector==1){

	dfloat newcoef = r.coef;
	
	/* start ray slightly off surface */
	dfloat sc = p_shadowDelta;
	if(rdotn>0) // reverse offset if inside 
	  sc *= -1.f; // sign ? was -1
	
	vector_t shadowStart = vectorAdd(intersection, vectorScale(sc, n)); // HACK to shift ray start off service
	ray_t lightRay;
	lightRay.start = shadowStart;
	
	/* Find the value of the light at this point */
	for(unsigned int j=0; j < Nlights; j++){
	  
	  light_t currentLight = lights[j];
	  
	  vector_t dist = vectorSub(currentLight.pos, shadowStart);
	  if(vectorDot(n, dist) <= 0) continue;

	  dfloat lightDist = vectorNorm(dist);

	  dfloat tshadow = lightDist;
	  if(tshadow <= 0) continue;
	  
	  lightRay.dir = vectorScale((1.f/tshadow), dist);
	  
	  /* search in light ray direction for object */
	  int shadowShapeID = -1;
	  gridRayIntersectionSearch(lightRay, Nshapes, shapes, grid, &tshadow, &shadowShapeID);

	  // check for objects in path of shadow ray
	  bool inShadow = false;	  
	  if(shadowShapeID==-1) // no object causes shadow
	    inShadow = false;
	  else if(tshadow >= 0 && tshadow < lightDist) // 
	    inShadow = true;

	  if(inShadow==false){
	    /* Lambert diffusion */
	    dfloat lambert = vectorDot(lightRay.dir, n) * newcoef; 
	    c.red   += lambert * currentLight.intensity.red   * currentMat.diffuse.red;
	    c.green += lambert * currentLight.intensity.green * currentMat.diffuse.green;
	    c.blue  += lambert * currentLight.intensity.blue  * currentMat.diffuse.blue;
	  }
	}

	// reduce reflected coefficient
	newcoef *= currentMat.reflection;
      
	if((r.level+1<p_maxLevel) && Nrays<p_maxNrays) {
	  ray_t reflectRay;
	  // create new ray starting from offset intersection, with ray direction reflected in normal
	  reflectRay.start = shadowStart;
	  reflectRay.dir   = vectorAdd(r.dir, vectorScale(-2.0f*rdotn, n)); 

	  // increment level for new ray
	  reflectRay.level = r.level+1;
	  reflectRay.coef = newcoef;
	
	  // launch new ray
	  rayStack[Nrays] = reflectRay;
	  // increment ray counter
	  ++Nrays;
	}
      }

      // https://www.scratchapixel.com/code.php?id=13&origin=/lessons/3d-basic-rendering/introduction-to-shading
      // test for refraction
      if(info.refractor==1){
	// can we add a new refraction ray to the stack ?
	if((r.level+1<p_maxLevel) && Nrays<p_maxNrays){
	
	  // push ray onto other side of surface
	  dfloat sc = -p_shadowDelta; // reverse number above
	  if(rdotn>0) 
	    sc *= -1;

	  // HACK to shift ray start off service
	  vector_t shadowStart = vectorAdd(intersection, vectorScale(sc, n)); 
	
	  // get index of refraction
	  dfloat eta = currentMat.eta; 
	
	  if(rdotn>0){
	    rdotn *= -1;
	  }else{
	    eta = 1.f/eta;
	  }
	
	  dfloat kappa = 1.f - eta*eta*(1.f - rdotn*rdotn);
	
	  if(kappa>0){
	    // create new refraction ray
	    ray_t refractRay;
	    dfloat fac = eta*rdotn-sqrt(kappa); 
	    refractRay.start = shadowStart;
	    refractRay.dir = vectorNormalize(vectorAdd(vectorScale(eta, r.dir), vectorScale(fac, n)));
	    refractRay.level = r.level+1;
	    refractRay.coef = r.coef; // ?
	    rayStack[Nrays] = refractRay;
	    ++Nrays;
	  }
	}
      }
    }
    
    // go to next ray on stack
    ++rayID;
  }

  return c;
  
}


// returns the cumulative sum
int gridScan(const int N, const int *v, int *scanv){

  scanv[0] = 0;
  for(int n=0;n<N;++n){
    scanv[n+1] = v[n]+scanv[n];
  }
  
  return scanv[N];
}


void gridCountShapesInCellsKernel(const grid_t grid, const int Nshapes, shape_t *shapes, int *counts){

  int N = Nshapes;
  for(int n=0;n<N;++n){

    shape_t *shape = shapes+n;
    shape->bbox = createBoundingBoxShape(grid, *shape);
    
    const  int imin = shape->bbox.imin;
    const  int imax = shape->bbox.imax;
    const  int jmin = shape->bbox.jmin;
    const  int jmax = shape->bbox.jmax;
    const  int kmin = shape->bbox.kmin;
    const  int kmax = shape->bbox.kmax;
    
    for(int k=kmin;k<=kmax;++k){
      for(int j=jmin;j<=jmax;++j){
	for(int i=imin;i<=imax;++i){
	  int id = i + j*grid.NI + k*grid.NI*grid.NJ;
	  ++counts[id];
	}
      }
    }
  }
}


void gridAddShapesInCellsKernel(const grid_t grid, const int Nshapes, const shape_t *shapes, int *boxCounters, int *boxContents){
  
  for(int n=0;n<Nshapes;++n){
    const shape_t *shape = shapes+n;

    const  int imin = shape->bbox.imin;
    const  int imax = shape->bbox.imax;
    const  int jmin = shape->bbox.jmin;
    const  int jmax = shape->bbox.jmax;
    const  int kmin = shape->bbox.kmin;
    const  int kmax = shape->bbox.kmax;
    
    for(int k=kmin;k<=kmax;++k){
      for(int j=jmin;j<=jmax;++j){
	for(int i=imin;i<=imax;++i){
	  // box	  
	  const int id = i + j*grid.NI + k*grid.NI*grid.NJ;
	  
	  // index in this box (post decremented)
	  // grab counter for this cell into index, then increment counter for this cell
	  boxContents[boxCounters[id]] = shape->id;
	  ++boxCounters[id];
	}
      }
    }
  }
}

void gridPopulate(grid_t *grid, int Nshapes, shape_t *shapes){

  if(grid->boxContents){
    free(grid->boxContents);
    free(grid->boxStarts);
  }
  
  // how many cells in grid
  int Nboxes = grid->NI*grid->NJ*grid->NK;

  // count how many objects overlap each cell
  int *boxCounts  = (int*) calloc(Nboxes+1, sizeof(int));
  gridCountShapesInCellsKernel (*grid, Nshapes, shapes, boxCounts);

  // make cumulative count
  grid->boxStarts  = (int*) calloc(Nboxes+1, sizeof(int));  
  int Nentries = gridScan(Nboxes, boxCounts, grid->boxStarts);
  
  // make a copy of boxCounts
  int *boxCounters = (int*) calloc(Nboxes+1, sizeof(int));
  memcpy(boxCounters, grid->boxStarts, (Nboxes+1)*sizeof(int));

  // accumulate all object indices for each cell
  grid->boxContents = (int*) calloc(Nentries, sizeof(int));
  
  // add each shape to every box that intersects the shape's bounding box
  gridAddShapesInCellsKernel (*grid, Nshapes, shapes, boxCounters, grid->boxContents);

  free(boxCounts);
  free(boxCounters);
  
}
