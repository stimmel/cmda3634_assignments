#include "simpleRayTracer.h"

void sphereCollisions(const grid_t *grid,
		      const dfloat dt,
		      const dfloat g,
		      const int Nshapes,
		      shape_t *shapes){

  // loop over all shapes
  for(int id=0;id<Nshapes;++id){
    shape_t &shape = shapes[id];

    // only do something if shape is a sphere
    if(shape.type==SPHERE){
      
      // sum up all sphere-sphere interactions
      // do hard reflections for collision with nearest non-sphere
      shape.sphere.newVelocity = shape.sphere.velocity;

      // find range of cells that overlap with sphere
      const  int imin = shape.bbox.imin;
      const  int imax = shape.bbox.imax;
      const  int jmin = shape.bbox.jmin;
      const  int jmax = shape.bbox.jmax;
      const  int kmin = shape.bbox.kmin;
      const  int kmax = shape.bbox.kmax;
	
      int NcollisionSpheres = 0;
      int collisionSphereIds[p_maxNcollisions];
	
      int collisionShapeId = -1;
      int m;
      dfloat mindist = 1e9;

      // loop over cells that overlap with sphere
      for(int k=kmin;k<=kmax;++k){
	for(int j=jmin;j<=jmax;++j){
	  for(int i=imin;i<=imax;++i){
	      
	    const int cellID = i + grid->NI*j + grid->NI*grid->NJ*k;
	    const int start = grid->boxStarts[cellID];
	    const int   end = grid->boxStarts[cellID+1];

	    // check for collision with objects that are contained in this cell
	    for(int offset=start;offset<end;++offset){
	      const int otherShapeId = grid->boxContents[offset];
	      const shape_t otherShape = shapes[otherShapeId];

	      // do not collide sphere with self
	      if(shape.id != otherShapeId){

		// if other shape is sphere do Newtonian collision with old velocity
		if(otherShape.type == SPHERE && NcollisionSpheres<p_maxNcollisions){
		  
		  vector_t dX = vectorSub(otherShape.sphere.pos, shape.sphere.pos);
		  dfloat dist = vectorNorm(dX);
		    
		  if(dist< shape.sphere.radius+otherShape.sphere.radius){
		      
		    // use rough sphere colliding model
		    dfloat vdotdX1 = vectorDot(     shape.sphere.velocity,dX)/(dist*dist + p_eps);
		    dfloat vdotdX2 = vectorDot(otherShape.sphere.velocity,dX)/(dist*dist + p_eps);
		      
		    // spheres not diverging to avoid capture
		    if(!(vdotdX1<0 && vdotdX2>0)){
			
		      // check that this collision did not already get recorded
		      for(m=0;m<NcollisionSpheres;++m)
			if(collisionSphereIds[m] == otherShapeId)
			  break;
			
		      if(m==NcollisionSpheres)
			collisionSphereIds[NcollisionSpheres++] = otherShapeId;
		    }
		  }
		}
		else if(otherShape.type != SPHERE){
		  // if other shape is not  a sphere then
		  // check if this is the nearest other object to sphere
		  vector_t closest;
		  dfloat dist = projectPointShape(shape.sphere.pos, otherShape, &closest);
		    
		  if(dist<shape.sphere.radius && dist<mindist){ // find maximum penetration
		    collisionShapeId = otherShapeId;
		    mindist = dist;
		  }
		}
	      }
	    }
	  }
	}
      }
	
      // collide with nearest spheres
      for(m=0;m<NcollisionSpheres;++m){
	const int collisionSphereId = collisionSphereIds[m];
	const shape_t otherShape = shapes[collisionSphereId];
	vector_t dX = vectorSub(otherShape.sphere.pos, shape.sphere.pos);
	  
	dfloat dist = vectorNorm(dX);
	  
	// use rough sphere colliding model
	dfloat vdotdX1 = vectorDot(     shape.sphere.velocity,dX)/(dist*dist + p_eps);
	dfloat vdotdX2 = vectorDot(otherShape.sphere.velocity,dX)/(dist*dist + p_eps);
	  
	// spheres not diverging to avoid capture
	if(!(vdotdX1<0 && vdotdX2>0)){
	  
	  dfloat dVdotdX = vdotdX2-vdotdX1;
	  dfloat bounceFactor = 1;
	    
	  shape.sphere.newVelocity =
	    vectorAdd(shape.sphere.newVelocity, vectorScale(bounceFactor*dVdotdX, dX));
	    
	}
      }

      vector_t gforce = vectorCreate(0,g,0);

      // perform collision with nearest non-sphere if any
      if(collisionShapeId!=-1){
	const shape_t otherShape = shapes[collisionShapeId];
	
	// bounce back 
	vector_t closest;
	dfloat dist = projectPointShape(shape.sphere.pos, otherShape, &closest);
	  
	if(dist<shape.sphere.radius){
	  vector_t disp = vectorSub(shape.sphere.pos, closest);
	  dfloat vdotdisp = vectorDot(shape.sphere.velocity, disp);
	    
	  vector_t n = shapeComputeNormal(closest, otherShape);
	    
	  if(vdotdisp<0){
	      
	    dfloat vdotn = vectorDot(shape.sphere.velocity, n);
	    dfloat bounceFactor = 1.8; // sticky at 1, bounce at 2
	      
	    shape.sphere.newVelocity =
	      vectorAdd(shape.sphere.newVelocity, vectorScale(-bounceFactor*vdotn, n));
	  }
	    
	  // project out gravity when in collision state
	  dfloat gdotdisp = vectorDot(gforce, disp);
	  //	      if(gdotdisp<0){
	  {
	    gforce = vectorSub(gforce, vectorScale(vectorDot(n, gforce), n));
	  }
	}
      }
	
      shape.sphere.force = gforce;
    }
  }
}

void sphereUpdates(grid_t *grid,
		   const dfloat dt,
		   const dfloat g,
		   const int Nshapes,
		   shape_t *shapes){
  
  for(int id=0;id<Nshapes;++id){
    shape_t &shape = shapes[id];
    if(shape.type == SPHERE){
      sphere_t sphere = shape.sphere;
      
      sphere.velocity = sphere.newVelocity;
      
      // Euler-forward update position
      shape.sphere.pos = vectorAdd(sphere.pos,
				   vectorScale(dt, sphere.velocity));
      
      // Euler-forward accelerate (assume gravity in y )
      shape.sphere.velocity = vectorAdd(sphere.velocity,
					vectorScale(dt, sphere.force));
    }
  }
}

