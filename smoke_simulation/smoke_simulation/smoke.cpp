#include "glCanvas.h"

#include <fstream>
#include <iomanip>
#include <algorithm>

#include "smoke.h"
#include "glCanvas.h"

#include <fstream>
#include <iomanip>
#include <algorithm>

#include "smoke.h"
#include "argparser.h"
#include "boundingbox.h"
#include "vectors.h"
#include "matrix.h"
#include "marching_cubes.h"
#include "utils.h"

#define BETA_0 1.7
#define EPSILON 0.0001

// ==============================================================
// ==============================================================
// CONSTRUCTOR
// ==============================================================
// ==============================================================

Smoke::Smoke(ArgParser *_args) {
	args = _args;
	Load();
	marchingCubes = new MarchingCubes(nx+1,ny+1,nz+1,dx,dy,dz);
	SetEmptySurfaceFull();
	initializeVBOs();
	setupVBOs();
}

Smoke::~Smoke() { 
  delete [] oc; 
  delete marchingCubes; 
  cleanupVBOs(); 
}

// ==============================================================

void Smoke::Load() {    

	// open the file
	assert (args->smoke_file != "");
	std::ifstream istr(args->smoke_file.c_str());
	assert (istr != NULL);
	std::string token, token2, token3;

	// load in the grid size
	// Read in size of grid / scene, assert that it's greater than 0
	istr >> token >> nx >> ny >> nz;  assert (token=="grid");
	assert (nx > 0 && ny > 0 && nz > 0);

	// Create our original bounding box to encompass scene
	Vec3f min = Vec3f(0, 0, 0);		// Min corner is origin
	//Vec3f max = Vec3f((nx+2), (ny+2), (nz+2));
	Vec3f max = Vec3f(nx, ny, nz);	// Max corner is x,y,z
	BoundingBox * bb = new BoundingBox(min, max);

	// Initialize our octree data structure
	oc = new OCTree(bb, 0);
	grid = new BoundingBox(min - Vec3f(1,1,1), max + Vec3f(1,1,1));

	// simulation parameters
	istr >> token >> token2;  assert (token=="flow");
	if (token2 == "compressible") compressible = true;
	else { assert (token2 == "incompressible"); compressible = false; }
	istr >> token >> token2;  assert (token=="xy_boundary");
	if (token2 == "free_slip") xy_free_slip = true;
	else { assert  (token2 == "no_slip"); xy_free_slip = false; }
	istr >> token >> token2;  assert (token=="yz_boundary");
	if (token2 == "free_slip") yz_free_slip = true;
	else { assert  (token2 == "no_slip"); yz_free_slip = false; }
	istr >> token >> token2;  assert (token=="zx_boundary");
	if (token2 == "free_slip") zx_free_slip = true;
	else { assert  (token2 == "no_slip"); zx_free_slip = false; }
	istr >> token >> viscosity;  assert (token=="viscosity");
	double gravity;
	istr >> token >> gravity;  assert (token=="gravity");
	args->gravity = Vec3f(0,-9.8,0) * gravity;
  
	// initialize marker particles 
	istr >> token >> token2 >> token3;  assert (token=="initial_particles");
	istr >> token >> density;  assert (token=="density");
	GenerateParticles(token2,token3);

	// initialize velocities
	istr >> token >> token2;  assert (token=="initial_velocity");
	if (token2 == "zero") 
	{
		// default is zero
	} 
	else 
	{
		/*
		assert (token2 == "random");
		int i,j,k;
		double max_dim = my_max(dx,my_max(dy,dz));
		for (i = -1; i <= nx; i++) 
		{
			for (j = -1; j <= ny; j++) 
			{
				for (k = -1; k <= nz; k++) 
				{
					//getBoundingBox(i,j,k)->set_u_plus((2*args->mtrand.rand()-1)*max_dim);
					//getBoundingBox(i,j,k)->set_v_plus((2*args->mtrand.rand()-1)*max_dim);
					//getBoundingBox(i,j,k)->set_w_plus((2*args->mtrand.rand()-1)*max_dim);
				}
			}
		}
		*/
	}
	// read in custom velocities
	while(istr >> token) 
	{
		int i,j,k;
		double velocity;
		assert (token == "u" || token == "v" || token == "w");
		istr >> i >> j >> k >> velocity;
		assert(i >= 0 && i < nx);
		assert(j >= 0 && j < ny);
		assert(k >= 0 && k < nz);
		if		(token == "u") oc->getCell(i,j,k)->set_u_plus(velocity);
		else if	(token == "v") oc->getCell(i,j,k)->set_v_plus(velocity);
		else if	(token == "w") oc->getCell(i,j,k)->set_w_plus(velocity);
		else assert(0);
	}
	SetBoundaryVelocities();
}

// ==============================================================

bool Smoke::inShape(Vec3f &pos, const std::string &shape) {
	// return true if this point is inside the "shape"
	// defined procedurally (using an implicit surface)
	if (shape == "everywhere") 
	{
		return true;
	} 
	else if (shape == "left") 
	{
		// a blob of particles on the lower left (for the dam)
		return (pos.x() < 0.2*nx && pos.y() < 0.5*ny);
	} 
	else if (shape == "drop") 
	{
		// a shallow pool of particles on the bottom
		double h = ny*dy/6.0;
		if (pos.y() < 2*h) return true;
		// and a sphere of particles above
		Vec3f center = Vec3f(nx*dx*0.5, 5*h,nz*dz*0.5);
		double length = (center-pos).Length();
		if (length < 0.8*h) return true;
		return false;
	} 
	else 
	{
		std::cout << "unknown shape: " << shape << std::endl;
		exit(0);
	}
}

// ==============================================================

void Smoke::GenerateParticles(const std::string &shape, const std::string &placement) {
  // create a set of points according to the "placement" token,
  // then check whether they are inside of the "shape"
	if (placement == "uniform") 
	{
		int dens = (int)pow(density,0.334);
		assert (dens*dens*dens == density);
		// the uniform grid spacing
		double spacing = 1/double(dens);
		for (double x = 0.5*spacing*dx; x < nx*dx; x += spacing*dx) {
			for (double y = 0.5*spacing*dy; y < ny*dy; y += spacing*dy) {
				for (double z = 0.5*spacing*dz; z < nz*dz; z += spacing*dz) {
					Vec3f pos = Vec3f(x,y,z);
					if (inShape(pos,shape))
					{
						SmokeParticle *p = new SmokeParticle();
						p->setPosition(pos);
						//std::cout << p->getPosition().x() << ',' << p->getPosition().y() << ',' << p->getPosition().z() << std::endl;
						oc->AddParticle(p);
					}
				}
			}
		}
	} 
	else 
	{
		assert (placement == "random");
		// note: we don't necessarily have the same number of particles in each cell
		for (int n = 0; n < nx*ny*nz*density; n++) 
		{
			Vec3f pos = Vec3f(args->mtrand.rand()*nx, args->mtrand.rand()*ny, args->mtrand.rand()*nz);
			if (inShape(pos,shape)) 
			{      
				SmokeParticle *p = new SmokeParticle();
				p->setPosition(pos);
				//std::cout << p->getPosition().x() << ',' << p->getPosition().y() << ',' << p->getPosition().z() << std::endl;
				oc->AddParticle(p);
			}
		}
	}
}

// ==============================================================
// ==============================================================
// ANIMATION
// ==============================================================
// ==============================================================

void Smoke::Animate() {

  // the animation manager:  this is what gets done each timestep!

  ComputeNewVelocities();
  SetBoundaryVelocities();
  
  // compressible / incompressible flow
  if (compressible == false) {
    for (int iters = 0; iters < 20; iters++) {
      double max_divergence = AdjustForIncompressibility();
      SetBoundaryVelocities();
      if (max_divergence < EPSILON) break;
    }
  }

  UpdatePressures();
  CopyVelocities();

  // advanced the particles through the Smoke
  MoveParticles();
  ReassignParticles();
  SetEmptySurfaceFull();

  setupVBOs();
}

// ==============================================================

void Smoke::ComputeNewVelocities() {
  double dt = args->timestep;
  int i,j,k;

  // using the formulas from Foster & Metaxas
  // NEED TO IMPLEMENT
  /*
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			double new_u_plus = get_u_plus(i,j,k) + dt * 
			((1/dx) * (square(get_u_avg(i,j,k)) - square(get_u_avg(i+1,j,k))) +
			(1/dy) * (get_uv_plus(i,j-1,k) - get_uv_plus(i,j,k)) + 
			(1/dz) * (get_uw_plus(i,j,k-1) - get_uw_plus(i,j,k)) +
			args->gravity.x() +
			(1/dx) * (getPressure(i,j,k)-getPressure(i+1,j,k)) +
			(viscosity/square(dx)) * (get_u_plus(i+1,j  ,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i-1,j  ,k  )) +
			(viscosity/square(dy)) * (get_u_plus(i  ,j+1,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j-1,k  )) +
			(viscosity/square(dz)) * (get_u_plus(i  ,j  ,k+1) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j  ,k-1)) );
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
  /*
  for (i = 0; i < nx-1; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        BoundingBox * bb = oc->getCell(i,j,k);
        double new_u_plus =
          get_u_plus(i,j,k) +            
          dt * ((1/dx) * (square(get_u_avg(i,j,k)) - square(get_u_avg(i+1,j,k))) +
                (1/dy) * (get_uv_plus(i,j-1,k) - get_uv_plus(i,j,k)) + 
                (1/dz) * (get_uw_plus(i,j,k-1) - get_uw_plus(i,j,k)) +
                args->gravity.x() +
                (1/dx) * (getPressure(i,j,k)-getPressure(i+1,j,k)) +
                (viscosity/square(dx)) * (get_u_plus(i+1,j  ,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_u_plus(i  ,j+1,k  ) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_u_plus(i  ,j  ,k+1) - 2*get_u_plus(i,j,k) + get_u_plus(i  ,j  ,k-1)) );
        bb->set_new_u_plus(new_u_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny-1; j++) {
      for (k = 0; k < nz; k++) {	
        BoundingBox * bb = getBoundingBox(i,j,k);
        double new_v_plus =
          get_v_plus(i,j,k) +
          dt * ((1/dx) * (get_uv_plus(i-1,j,k) - get_uv_plus(i,j,k)) +
                (1/dy) * (square(get_v_avg(i,j,k)) - square(get_v_avg(i,j+1,k))) +
                (1/dz) * (get_vw_plus(i,j,k-1) - get_vw_plus(i,j,k)) +
                args->gravity.y() +
                (1/dy) * (getPressure(i,j,k)-getPressure(i,j+1,k)) +
                (viscosity/square(dx)) * (get_v_plus(i+1,j  ,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_v_plus(i  ,j+1,k  ) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_v_plus(i  ,j  ,k+1) - 2*get_v_plus(i,j,k) + get_v_plus(i  ,j  ,k-1)) );
        bb->set_new_v_plus(new_v_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz-1; k++) {
        BoundingBox *bb = getBoundingBox(i,j,k);
        double new_w_plus =
          get_w_plus(i,j,k) +
          dt * ((1/dx) * (get_uw_plus(i-1,j,k) - get_uw_plus(i,j,k)) +
                (1/dy) * (get_vw_plus(i,j-1,k) - get_vw_plus(i,j,k)) +
                (1/dz) * (square(get_w_avg(i,j,k)) - square(get_w_avg(i,j,k+1))) +
                args->gravity.z() +
                (1/dz) * (getPressure(i,j,k)-getPressure(i,j,k+1)) +
                (viscosity/square(dx)) * (get_w_plus(i+1,j  ,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i-1,j  ,k  )) +
                (viscosity/square(dy)) * (get_w_plus(i  ,j+1,k  ) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j-1,k  )) +
                (viscosity/square(dz)) * (get_w_plus(i  ,j  ,k+1) - 2*get_w_plus(i,j,k) + get_w_plus(i  ,j  ,k-1)) );
        bb->set_new_w_plus(new_w_plus);
      }
    }
  }
  */
}


// ==============================================================

void Smoke::SetBoundaryVelocities() {

  // zero out flow perpendicular to the boundaries (no sources or sinks)
	grid->set_u_plus(0);
	grid->set_v_plus(0);
	grid->set_w_plus(0);

  // free slip or no slip boundaries (friction with boundary)
  double xy_sign = (xy_free_slip) ? 1 : -1;
  double yz_sign = (yz_free_slip) ? 1 : -1;
  double zx_sign = (zx_free_slip) ? 1 : -1;
  // NEED TO IMPLEMENT
  /*
  for (int i = 0; i < nx; i++) {
    for (int j = -1; j <= ny; j++) {
      getBoundingBox(i,j,-1)->set_u_plus(xy_sign*getBoundingBox(i,j,0)->get_u_plus());
      getBoundingBox(i,j,nz)->set_u_plus(xy_sign*getBoundingBox(i,j,nz-1)->get_u_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getBoundingBox(i,-1,k)->set_u_plus(zx_sign*getBoundingBox(i,0,k)->get_u_plus());
      getBoundingBox(i,ny,k)->set_u_plus(zx_sign*getBoundingBox(i,ny-1,k)->get_u_plus());
    }
  }
  for (int j = 0; j < ny; j++) {
    for (int i = -1; i <= nx; i++) {
      getBoundingBox(i,j,-1)->set_v_plus(xy_sign*getBoundingBox(i,j,0)->get_v_plus());
      getBoundingBox(i,j,nz)->set_v_plus(xy_sign*getBoundingBox(i,j,nz-1)->get_v_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getBoundingBox(-1,j,k)->set_v_plus(yz_sign*getBoundingBox(0,j,k)->get_v_plus());
      getBoundingBox(nx,j,k)->set_v_plus(yz_sign*getBoundingBox(nx-1,j,k)->get_v_plus());
    }
  }
  for (int k = 0; k < nz; k++) {
    for (int i = -1; i <= nx; i++) {
      getBoundingBox(i,-1,k)->set_w_plus(zx_sign*getBoundingBox(i,0,k)->get_w_plus());
      getBoundingBox(i,ny,k)->set_w_plus(zx_sign*getBoundingBox(i,ny-1,k)->get_w_plus());
    }
    for (int j = -1; j <= ny; j++) {
      getBoundingBox(-1,j,k)->set_w_plus(yz_sign*getBoundingBox(0,j,k)->get_w_plus());
      getBoundingBox(nx,j,k)->set_w_plus(yz_sign*getBoundingBox(nx-1,j,k)->get_w_plus());
    }
  }
  */
}

// ==============================================================

void Smoke::EmptyVelocities(int i, int j, int k) {
  BoundingBox * c = oc->getCell(i,j,k);
  // NEED TO IMPLEMENT
  /*
  if (c->getStatus() != CELL_EMPTY) return;
  BoundingBox *ciplus = getBoundingBox(i+1,j,k);
  BoundingBox *cjplus = getBoundingBox(i,j+1,k);
  BoundingBox *ckplus = getBoundingBox(i,j,k+1);
  if (ciplus->getStatus() == CELL_EMPTY)
    c->set_new_u_plus(0);
  if (cjplus->getStatus() == CELL_EMPTY)
    c->set_new_v_plus(0);
  if (ckplus->getStatus() == CELL_EMPTY)
    c->set_new_w_plus(0);
	*/
}

void Smoke::CopyVelocities() {
	// NEED TO IMPLEMENT
	/*
  double dt = args->timestep;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
	BoundingBox *c = getBoundingBox(i,j,k);
	EmptyVelocities(i,j,k);
	c->copyVelocity();
	if (fabs(c->get_u_plus()) > 0.5*dx/dt ||
	    fabs(c->get_v_plus()) > 0.5*dy/dt ||
	    fabs(c->get_w_plus()) > 0.5*dz/dt) {
	  // velocity has exceeded reasonable threshhold
	  std::cout << "velocity has exceeded reasonable threshhold, stopping animation" << std::endl;
	  args->animate=false;
	}
      }
    }
  }
  */
}

// ==============================================================

double Smoke::AdjustForIncompressibility() {

  // *********************************************************************  
  // ASSIGNMENT:
  //
  // This is not a complete implementation of the Marker and BoundingBox (MAC) method.
  // Additional boundary velocities should be equalized as described in the references
  // depending on whether the boundaries are free-slip or no-slip.
  //
  // Also play around with compressible flow!
  //
  // *********************************************************************    
	double div = 0;
	double maxDiv = 0;
	// NEED TO IMPLEMENT
	/*
	for (int i = 0; i < nx; i++)  
	{
		for (int j = 0; j < ny; j++)
		{
			for (int k = 0; k < nz; k++)
			{
				/*if (getBoundingBox(i, j, k)->getStatus() == CELL_SURFACE)
				{
					if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i-1, j, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_u_plus(i+1, j, k, getBoundingBox(i-1, j, k)->get_new_u_plus());
					}
					if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i+1, j, k)->getStatus() != CELL_EMPTY )
					{
						adjust_new_u_plus(i-1, j, k, getBoundingBox(i, j, k)->get_new_u_plus());
					}
					if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i, j-1, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_v_plus(i, j+1, k, getBoundingBox(i, j-1, k)->get_new_v_plus());
					}
					if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY &&
						getBoundingBox(i, j+1, k)->getStatus() != CELL_EMPTY)
					{
						adjust_new_v_plus(i, j-1, k, getBoundingBox(i, j, k)->get_new_v_plus());
					}

					int freeBoundingBoxCount = 0;
					if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;
					if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY)
						freeBoundingBoxCount++;

					div = - (  (get_new_u_plus(i,j,k) - get_new_u_plus(i-1,j,k)) +
								 (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
							    (get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)));

					//if (div > 0)
					//{
					//	// Push this excess volume through the free faces
					//	if (getBoundingBox(i+1, j, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_u_plus(i, j, k, div/freeBoundingBoxCount);
					//	if (getBoundingBox(i, j+1, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_v_plus(i, j, k, div/freeBoundingBoxCount);
					//	if (getBoundingBox(i-1, j, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_u_plus(i-1, j, k, -div/freeBoundingBoxCount);
					//	if (getBoundingBox(i, j-1, k)->getStatus() == CELL_EMPTY)
					//		adjust_new_v_plus(i, j-1, k, -div/freeBoundingBoxCount);
					//}
					//else if (div < 0)
					//{
						//?
					//}
				}

				if (getBoundingBox(i, j, k)->getStatus() == CELL_FULL || getBoundingBox(i, j, k)->getStatus() == CELL_SURFACE)
				{
					//flow from cell                flow from neighbor cells (example 12-4 = 8)
					div = (get_new_u_plus(i,j,k)  - get_new_u_plus(i-1,j,k));
					div += (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k));
					div +=(get_new_w_plus(i,j,k)  - get_new_w_plus(i,j,k-1));
				    div =-div; //think bc adj is in flow, so flip >>in paper they have negative in equation		
  
					if(abs(div) >abs(maxDiv))
						maxDiv = div;
					
					/* Checking if we boarding all adjecent faces 
					int adjFaces = 6; //(each side cube)
					if (i == 0) //first cell
						adjFaces--;
					if (i == nx-1)  //last cell
						adjFaces--;
					if (j == 0) //first cell
						adjFaces--;
					if (j == ny-1)  // cell
						adjFaces--;
					if (k == 0)  //first cell
						adjFaces--;
					if (k == nz-1)  //last cell
						adjFaces--;
					
					//adjust velocity for face adj of in/out flow
					if (i < nx-1)
						adjust_new_u_plus(i, j, k, div/adjFaces);
					if (j < ny-1)
						adjust_new_v_plus(i, j, k, div/adjFaces);
					if (k < nz-1)
						adjust_new_v_plus(i, j, k, div/adjFaces);
					if (i-1 >= 0)
						adjust_new_u_plus(i-1, j, k, -div/adjFaces);
					if (j-1 >= 0)
						adjust_new_v_plus(i, j-1, k, -div/adjFaces);
					if (k-1 >= 0)
						adjust_new_w_plus(i, j, k-1, -div/adjFaces);
				
				}
			}

		}

	}

  // return the divergence (will be repeated while divergence > threshold)
	*/
	return fabs(maxDiv);
}

// ==============================================================

void Smoke::UpdatePressures() {
	// NEED TO IMPLEMENT
	/*
  for (int i = -1; i <= nx; i++) {
    for (int j = -1; j <= ny; j++) {
      for (int k = -1; k <= nz; k++) {
	BoundingBox *c = getBoundingBox(i,j,k);
	if (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz) {
	  // compute divergence and increment/decrement pressure
	  double pressure = c->getPressure();
	  double divergence = 
	    - ( (1/dx) * (get_new_u_plus(i,j,k) - get_new_u_plus(i-1,j,k)) +
		(1/dy) * (get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
		(1/dz) * (get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)) );
	  double dt = args->timestep;
	  double beta = BETA_0/((2*dt) * (1/square(dx) + 1/square(dy) + 1/square(dz)));
	  double dp = beta*divergence;
	  c->setPressure(pressure + dp);
	} else {
	  // zero out boundary cells (just in case)
	  c->setPressure(0);
	}

	// zero out empty cells (From Foster 2001 paper)
	if (c->getStatus() == CELL_EMPTY) {
	  c->setPressure(0);
	}
	// ========================================

      }
    }
  }
  */
}

// ==============================================================

void Smoke::MoveParticles() {
	// NEED TO IMPLEMENT
	/*
  double dt = args->timestep;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        BoundingBox *cell = getBoundingBox(i,j,k);
	std::vector<SmokeParticle*> &particles = cell->getParticles();
        for (unsigned int iter = 0; iter < particles.size(); iter++) {
          SmokeParticle *p = particles[iter];
          Vec3f pos = p->getPosition();
          Vec3f vel = getInterpolatedVelocity(pos);
          Vec3f pos2 = pos + vel*dt;
          // euler integration
          p->setPosition(pos2);
        }
      }
    }
  }
  */
}

// ==============================================================

void Smoke::ReassignParticles() {
	// NEED TO IMPLEMENT
	/*
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        BoundingBox *cell = getBoundingBox(i,j,k);
	std::vector<SmokeParticle*> &particles = cell->getParticles();
        for (unsigned int iter = 0; iter < particles.size(); iter++) {
          SmokeParticle *p = particles[iter];
          Vec3f pos = p->getPosition();
          int i2 = (int)my_min(double(nx-1),my_max(0.0,floor(pos.x()/dx)));
          int j2 = (int)my_min(double(ny-1),my_max(0.0,floor(pos.y()/dy)));
          int k2 = (int)my_min(double(nz-1),my_max(0.0,floor(pos.z()/dz)));
          // if the particle has crossed one of the cell faces 
          // assign it to the new cell
          if (i != i2 || j != j2 || k != k2) {
            cell->removeParticle(p);
            getBoundingBox(i2,j2,k2)->addParticle(p);
          } 
        }
      }
    }
  }
  */
}

// ==============================================================

void Smoke::SetEmptySurfaceFull() {
	// NEED TO IMPLEMENT
	/*
  int i,j,k;
  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        BoundingBox *cell = getBoundingBox(i,j,k);
        if (cell->numParticles() == 0)
          cell->setStatus(CELL_EMPTY);
        else 
          cell->setStatus(CELL_FULL);
      }
    }
  }

  // pick out the boundary cells
  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        BoundingBox *cell = getBoundingBox(i,j,k);
        if (cell->getStatus() == CELL_FULL &&
            (getBoundingBox(i-1,j,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i+1,j,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j-1,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j+1,k)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j,k-1)->getStatus() == CELL_EMPTY ||
             getBoundingBox(i,j,k+1)->getStatus() == CELL_EMPTY)) {
          cell->setStatus(CELL_SURFACE);
        }
      }
    }
  }
  */
}

// ==============================================================

Vec3f Smoke::getInterpolatedVelocity(const Vec3f &pos) const {


  // *********************************************************************  
  // ASSIGNMENT:
  //
  // I've intentionally reverted to the "dumb" velocity interpolation.
  // Do it right, as described in the papers.
  // ********************************************************************
	
	double u[8],v[8],w[8];
	double au[8],av[8],aw[8];
	Vec3f average;
	double xWeight, yWeight, zWeight;     

	// The percentage position of the particle relative to the origin of the cell it is within (adentro)
	double xDis = (pos.x()/dx) - floor(pos.x()/dx);
	double yDis = (pos.y()/dy) - floor(pos.y()/dy);
	double zDis = (pos.z()/dz) - floor(pos.z()/dz);
	
	int i = int(floor(pos.x()/dx)); if (i < 0) i = 0; if (i >= nx) i = nx-1;
	int j = int(floor(pos.y()/dx)); if (j < 0) j = 0; if (j >= ny) j = ny-1;
	int k = int(floor(pos.z()/dz)); if (k < 0) k = 0; if (k >= nz) k = nz-1;
	
	// Calculate the U velocity (x direction)

	/* Finding cells we need to use for the velocity in X direction. 
	    Evaluates to 0 when the particle is in the upper half of the 
		cell and 1 when the particle is in the lower half of the cell. */	
	i = int(floor(pos.x()/dx)) - 1;//if (i < 0) i = 0;
	j = int(floor(pos.y()/dy) - (1 - floor(2 * yDis)));//if (j < 0) j = 0;
	k = int(floor(pos.z()/dz) - (1 - floor(2 * zDis)));//if (k < 0) k = 0;
	
	u[0] = get_u_plus(i,j,k);
	u[1] = get_u_plus(i + 1,j,k);
	u[2] = get_u_plus(i,j + 1,k);
	u[3] = get_u_plus(i+1,j + 1,k); 
	u[4] = get_u_plus(i,j,k+1);
	u[5] = get_u_plus(i+1,j,k+1);
	u[6] = get_u_plus(i,j+1,k+1);
	u[7] = get_u_plus(i+1,j+1,k+1);
	
	xWeight = xDis;
	yWeight = yDis + (0.5 - floor(2*yDis));
	zWeight = zDis + (0.5 - floor(2*zDis));
	
	au[0] = (1 - xWeight)*(1 - yWeight)*(1 - zWeight);
	au[1] =  xWeight*(1 - yWeight)*(1 - zWeight);
	au[2] = (1 - xWeight)*yWeight*(1 - zWeight);
	au[3] =  xWeight*yWeight*(1 - zWeight);
	au[4] = (1 - xWeight)*(1 - yWeight)*zWeight;
	au[5] = xWeight*(1 - yWeight)*zWeight ;
	au[6] = (1- xWeight)*yWeight*zWeight;
	au[7] = xWeight*yWeight*zWeight;

	average.setx(au[0]*u[0] + au[1]*u[1] + au[2]*u[2] + au[3]*u[3] + au[4]*u[4] + au[5]*u[5] + au[6]*u[6] + au[7]*u[7]);

	// Calculate the V velocity (y direction)

	 /* Finding cells we need to use for the velocity in Y direction. 
	    Evaluates to 0 when the particle is in the upper half of the 
		cell and dx when the particle is in the lower half of the cell. */	
	i = int(floor(pos.x()/dx) - (1 - floor(2 * xDis)));//if (i < 0) i = 0;
	j = int(floor(pos.y()/dy)) - 1;//if (j < 0) j = 0;
	k = int(floor(pos.z()/dz) - (1 - floor(2 * zDis)));//if (k < 0) k = 0;
	
	xWeight = xDis + (0.5 - floor(2*xDis));
	yWeight = yDis;
	zWeight = zDis + (0.5 - floor(2*zDis));

	v[0] = get_v_plus(i,j,k);
	v[1] = get_v_plus(i + 1,j,k);
	v[2] = get_v_plus(i,j + 1,k);
	v[3] = get_v_plus(i+1,j + 1,k); 
	v[4] = get_v_plus(i,j,k+1);
	v[5] = get_v_plus(i+1,j,k+1);
	v[6] = get_v_plus(i,j+1,k+1);
	v[7] = get_v_plus(i+1,j+1,k+1);

	av[0] = (1 - xWeight)*(1 - yWeight)*(1 - zWeight);
	av[1] = xWeight*(1 - yWeight)*(1 - zWeight);
	av[2] = (1 - xWeight)*yWeight*(1 - zWeight);
	av[3] = xWeight*yWeight*(1 - zWeight) ;
	av[4] = (1 - xWeight)*(1 - yWeight)*zWeight;
	av[5] = xWeight*(1 - yWeight)*zWeight;
	av[6] = (1 - xWeight)*yWeight*zWeight;
	av[7] =  xWeight*yWeight*zWeight;

	average.sety(av[0]*v[0] + av[1]*v[1] + av[2]*v[2] + av[3]*v[3] + av[4]*v[4] + av[5]*v[5] + av[6]*v[6] + av[7]*v[7]);

	// Calculate the W velocity (z direction)

	 /* Finding cells we need to use for the velocity in Z direction. 
	    Evaluates to 0 when the particle is in the upper half of the 
		cell and dx when the particle is in the lower half of the cell. */	
	i = int(floor(pos.x()/dx) - (1 - floor(2 * xDis)));//if (i < 0) i = 0;
	j = int(floor(pos.y()/dy) - (1 - floor(2 * yDis)));//if (j < 0) j = 0;
	k = int(floor(pos.z()/dz)) - 1; //if (k < 0) k = 0;
	xWeight = xDis + (0.5 - floor(2*xDis));
	yWeight = yDis + (0.5 - floor(2*yDis));
	zWeight = zDis;
	
	w[0] = get_w_plus(i,j,k);
	w[1] = get_w_plus(i + 1,j,k);
	w[2] = get_w_plus(i,j + 1,k);
	w[3] = get_w_plus(i + 1,j + 1,k);
	w[4] = get_w_plus(i,j,k + 1);
	w[5] = get_w_plus(i + 1,j,k + 1);
	w[6] = get_w_plus(i,j + 1,k + 1);
	w[7] = get_w_plus(i + 1,j + 1,k + 1);

	aw[0] = (1 - xWeight)*(1 - yWeight)*(1 - zWeight);
	aw[1] =  xWeight*(1 - yWeight)*(1 - zWeight);
	aw[2] = (1 - xWeight)*yWeight*(1 - zWeight);
	aw[3] = xWeight*yWeight*(1 - zWeight);
	aw[4] = (1 - xWeight)*(1 - yWeight)*zWeight;
	aw[5] = xWeight*(1 - yWeight)*zWeight;
	aw[6] = (1 - xWeight)*yWeight*zWeight;
	aw[7] = xWeight*yWeight*zWeight;

	average.setz(aw[0]*w[0] + aw[1]*w[1] + aw[2]*w[2] + aw[3]*w[3] + aw[4]*w[4] + aw[5]*w[5] + aw[6]*w[6] + aw[7]*w[7]);


	return average;
}




