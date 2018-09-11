#include "simpleRayTracer.h"

dfloat drandRange48(dfloat dmin, dfloat dmax){

  return dmin + drand48()*(dmax-dmin);
}

// set up scene to render
// Notes:
//
// a. scene is assumed to reside in "worldspace" given by
//      [x,y,z] \in   [0,BOXSIZE] x [0,BOXSIZE] x [0,BOXSIZE]
// b. with current viewport settings y=0 is the towards the top of the render
// c. currently using 64 randomly generated materials

scene_t *sceneSetup(){
  int i;

  dfloat L = BOXSIZE;
  
  int Nmaterials = 64;
  material_t *materials = (material_t*) calloc(Nmaterials, sizeof(material_t));

  // base material - highly reflective, non-refracting
  materials[0].diffuse.red   = 1;
  materials[0].diffuse.green = 1;
  materials[0].diffuse.blue  = 1;
  materials[0].reflection = 1;
  materials[0].eta = 1;
  materials[0].refraction = 0;

  materials[0].info.refractor = 0;
  materials[0].info.reflector = 1;
  materials[0].info.emitter = 0;

  // randomly generate rest of materials
  for(i=1;i<Nmaterials;++i){
    
    // randomly choose rgb components each in the range [0.125,0.8]
    // [ generates nice pastel shades ]
    materials[i].diffuse.red   = drandRange48(0.125,0.8);
    materials[i].diffuse.green = drandRange48(0.125,0.8);
    materials[i].diffuse.blue  = drandRange48(0.125,0.8);

    // index of reflection used at material interface
    materials[i].eta = 2;
    materials[i].refraction = 1; // transmission coeff

    if(drand48() > .5){
      materials[i].reflection = .9;
      materials[i].info.reflector = 1;
    }
    if(drand48() > .5){
      materials[i].refraction = .9;
      materials[i].info.refractor = 1;
    }

    if(!materials[i].info.refractor && !materials[i].info.reflector){
      materials[i].info.reflector = 1;
    }
  }

  // read bunny.ply 
  int Ntriangles; 
  triangle_t *triangles;
  readPlyModel("bunny.ply", &Ntriangles, &triangles);

  int Nbunny = 10;   // will use 10 copies of bunny
  int Nspheres = 100; // 100 random spheres
  int Ncones = 20;    // 20 random cones
  int Ncylinders = 20;// 20 random cylinders
  int Nrectangles = 1;// 1  ground plane rectangle

  int Nshapes = Nspheres + Nbunny*Ntriangles + Ncones + 3*Ncylinders + Nrectangles; // each cylinder has two end disks

  shape_t *shapes = (shape_t*) calloc(Nshapes, sizeof(shape_t));

  // clone the bunny Nbunny times
  int bcnt = 0;
  for(int b=0;b<Nbunny;++b){
    dfloat boffx = drandRange48(50, L-50);
    dfloat boffy = drandRange48(50, L-50);
    dfloat boffz = drandRange48(50, L-50);
    dfloat bscal = .2;
    for(i=0;i<Ntriangles;++i){
      shapes[bcnt].triangle = triangles[i];

      // scale and rotate bunny onto ground plane
      for(int v=0;v<3;++v){
	shapes[bcnt].triangle.vertices[v].x *= bscal;
	shapes[bcnt].triangle.vertices[v].y = L - bscal*(L-shapes[bcnt].triangle.vertices[v].y);
	shapes[bcnt].triangle.vertices[v].z *= bscal;

	shapes[bcnt].triangle.vertices[v].x += boffx;
	shapes[bcnt].triangle.vertices[v].z += boffz;
      }

      // choose material for bunny
      shapes[bcnt].material = 10;
      shapes[bcnt].type = TRIANGLE;
      shapes[bcnt].id = bcnt;
      ++bcnt;
    }
  }

  Ntriangles *= Nbunny;
  printf("Ntriangles = %d\n", Ntriangles);

  int cnt = Ntriangles;

  // generate random cones
  for(i=0;i<Ncones;++i){

    shapes[cnt].cone.radius = 140 + 40*drand48();
    shapes[cnt].cone.height = 280 + 40*drand48();

    shapes[cnt].cone.axis.x = 0; // unit vector
    shapes[cnt].cone.axis.y = -1;
    shapes[cnt].cone.axis.z = 0;
    
    shapes[cnt].cone.vertex.x = 0.5*L + (L)*(drand48()-0.5);
    shapes[cnt].cone.vertex.y = L; // -shapes[cnt].cone.height;
    shapes[cnt].cone.vertex.z = 0.5*L + (L)*(drand48()-0.5);
    
    shapes[cnt].material = 1 + (Nmaterials-2)*drand48();
    shapes[cnt].type = CONE;
    shapes[cnt].id = cnt;
    ++cnt;

  }

  // generate random spheres
  for(i=0;i<Nspheres;++i){
    
    shapes[cnt].sphere.radius = 35 + 20*drand48();

    if(i<Ncones){
      shapes[cnt].sphere.pos.x = shapes[cnt-Ncones].cone.vertex.x;
      shapes[cnt].sphere.pos.y = L/4-100;
      shapes[cnt].sphere.pos.z = shapes[cnt-Ncones].cone.vertex.z;
    }else{
      shapes[cnt].sphere.pos.x = L*(drand48());
      shapes[cnt].sphere.pos.y = L/4-shapes[cnt].sphere.radius;
      shapes[cnt].sphere.pos.z = L*(drand48());
    }
    shapes[cnt].sphere.pos.y += 400*drand48(); // drop from up to 150 pixels

    shapes[cnt].sphere.velocity = vectorCreate(0,0,0);
    shapes[cnt].sphere.newVelocity = vectorCreate(0,0,0);

    shapes[cnt].material = 1 + (Nmaterials-2)*drand48();
    shapes[cnt].type = SPHERE;
    shapes[cnt].id = cnt;
    ++cnt;
    
  }

  // generate random cylinders
  for(i=0;i<Ncylinders;++i){

    shapes[cnt].cylinder.radius = 100 + 10*drand48();
    shapes[cnt].cylinder.height = 380 + 40*drand48();

    dfloat phi = 0.25*M_PI*drand48();
    shapes[cnt].cylinder.axis.x = 0; // unit vector
    shapes[cnt].cylinder.axis.y = cos(phi);
    shapes[cnt].cylinder.axis.z = sin(phi);
    
    shapes[cnt].cylinder.center.x = L/2 + (L)*(drand48()-0.5);
    shapes[cnt].cylinder.center.y = L-shapes[cnt].cylinder.height;
    shapes[cnt].cylinder.center.z = L/2 + (L)*(drand48()-0.5);
    
    shapes[cnt].material = 1 + (Nmaterials-2)*drand48();
    shapes[cnt].type = CYLINDER;
    shapes[cnt].id = cnt;
    ++cnt;

    cylinder_t oldCylinder = shapes[cnt-1].cylinder;

    // end cap
    shapes[cnt].disk.radius = oldCylinder.radius;
    shapes[cnt].disk.normal = vectorScale(+1.,oldCylinder.axis); // why ?
    shapes[cnt].disk.center = oldCylinder.center;
    shapes[cnt].material = 10;
    shapes[cnt].type = DISK;
    shapes[cnt].id = cnt;
    ++cnt;

    // end cap
    shapes[cnt].disk.radius = oldCylinder.radius;
    shapes[cnt].disk.normal = vectorScale(-1.,oldCylinder.axis); // why ?
    shapes[cnt].disk.center = 
      vectorAdd(oldCylinder.center, vectorScale(oldCylinder.height, oldCylinder.axis));
    
    shapes[cnt].material = 10;
    shapes[cnt].type = DISK;
    shapes[cnt].id = cnt;
    ++cnt;
    
  }
  
  // add the rectangle ground plane
  if(Nrectangles){

    // dimensiojns for gound plane
    vector_t a = vectorCreate(0, L, 0);
    vector_t b = vectorCreate(0, L, L);
    vector_t c = vectorCreate(L, L, L);
    vector_t d = vectorCreate(L, L, 0);

    vector_t ab = vectorSub(d,a);
    vector_t ad = vectorSub(b,a);
    shapes[cnt].rectangle.length[0] = vectorNorm(ab);
    shapes[cnt].rectangle.length[1] = vectorNorm(ad);
    shapes[cnt].rectangle.axis[0] = vectorNormalize(ab);
    shapes[cnt].rectangle.axis[1] = vectorNormalize(ad);
    shapes[cnt].rectangle.center  = vectorScale(0.25, vectorAdd(vectorAdd(a,b),vectorAdd(c,d)));
    shapes[cnt].material = 2;
    shapes[cnt].type = RECTANGLE;
    shapes[cnt].id = cnt;    
    ++cnt;
  }

  // generate specific lights
  int Nlights = 5;
  light_t *lights = (light_t*) calloc(Nlights, sizeof(light_t));
	
  lights[0].pos.x = L/2;
  lights[0].pos.y = 0;
  lights[0].pos.z = -100;
  lights[0].intensity.red = 1;
  lights[0].intensity.green = 1;
  lights[0].intensity.blue = 1;
	
  lights[1].pos.x = 3200;
  lights[1].pos.y = 3000;
  lights[1].pos.z = -1000;
  lights[1].intensity.red = 0.6;
  lights[1].intensity.green = 0.7;
  lights[1].intensity.blue = 1;

  lights[2].pos.x = 600;
  lights[2].pos.y = 0;
  lights[2].pos.z = -100;
  lights[2].intensity.red = 0.3;
  lights[2].intensity.green = 0.5;
  lights[2].intensity.blue = 1;

  lights[3].pos.x = L/2;
  lights[3].pos.y = 0;
  lights[3].pos.z = L/2;
  lights[3].intensity.red = 0.8;
  lights[3].intensity.green = 0.8;
  lights[3].intensity.blue = 1;

  lights[4].pos.x = L;
  lights[4].pos.y = L;
  lights[4].pos.z = -1000;
  lights[4].intensity.red = 1;
  lights[4].intensity.green = 1;
  lights[4].intensity.blue = 1;

  // sort the objects into a regular grid for faster look up
  grid_t *grid = (grid_t*) calloc(1, sizeof(grid_t));
  dfloat delta = 100;
  grid->xmin = -delta;
  grid->xmax = L + delta;
  grid->ymin = -delta;
  grid->ymax = L + delta;
  grid->zmin = -delta;
  grid->zmax = L + delta;

  grid->NI = 151;
  grid->NJ = 151;
  grid->NK = 151;

  grid->dx = (grid->xmax-grid->xmin)/grid->NI;
  grid->dy = (grid->ymax-grid->ymin)/grid->NJ;
  grid->dz = (grid->zmax-grid->zmin)/grid->NK;
  
  grid->invdx = grid->NI/(grid->xmax-grid->xmin);
  grid->invdy = grid->NJ/(grid->ymax-grid->ymin);
  grid->invdz = grid->NK/(grid->zmax-grid->zmin);

  grid->bboxes = (bbox_t*) calloc(grid->NI*grid->NJ*grid->NK, sizeof(bbox_t));
  for(int k=0;k<grid->NK;++k){
    for(int j=0;j<grid->NJ;++j){
      for(int i=0;i<grid->NI;++i){
	int id = i + j*grid->NI + k*grid->NI*grid->NJ;
	grid->bboxes[id].xmin = i*grid->dx + grid->xmin;
	grid->bboxes[id].xmax = (i+1)*grid->dx + grid->xmin;
	grid->bboxes[id].ymin = j*grid->dy + grid->ymin;
	grid->bboxes[id].ymax = (j+1)*grid->dy + grid->ymin;
	grid->bboxes[id].zmin = k*grid->dz + grid->zmin;
	grid->bboxes[id].zmax = (k+1)*grid->dz + grid->zmin;
      }
    }
  }

  // capture all elements into the scene
  scene_t *scene = (scene_t*) calloc(1, sizeof(scene_t));
  scene->Nlights  = Nlights;
  scene->lights   = lights;
  scene->Nshapes   = Nshapes;
  scene->shapes   = shapes;
  scene->Nmaterials = Nmaterials;
  scene->materials  = materials;
  scene->grid = grid;
  
  return scene;
}
