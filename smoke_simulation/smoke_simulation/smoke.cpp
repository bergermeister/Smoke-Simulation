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
		double i,j,k;
		double velocity;
		assert (token == "u" || token == "v" || token == "w");
		istr >> i >> j >> k >> velocity;
		assert(i >= 0 && i < nx);
		assert(j >= 0 && j < ny);
		assert(k >= 0 && k < nz);
		if		(token == "u") oc->getCell(Vec3f(i,j,k))->set_u_plus(velocity);
		else if	(token == "v") oc->getCell(Vec3f(i,j,k))->set_v_plus(velocity);
		else if	(token == "w") oc->getCell(Vec3f(i,j,k))->set_w_plus(velocity);
		else assert(0);
		/*
		std::vector<OCTree*> todo;  
		todo.push_back(oc);
		while (!todo.empty()) 
		{
			OCTree *node = todo.back();
			todo.pop_back(); 
			if (node->isLeaf()) {
				std::cout << "Child at " << node->getCenter() << " | u: " << node->getCell()->get_u_plus() << std::endl;
			} 
			else 
			{
				// if this cell is not a leaf, explore both children
				for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
			} 
		}
		*/
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
		Vec3f center = Vec3f(nx*0.5, 5*h,nz*0.5);
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
		for (double x = 0.5*spacing; x < nx; x += spacing) {
			for (double y = 0.5*spacing; y < ny; y += spacing) {
				for (double z = 0.5*spacing; z < nz; z += spacing) {
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
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	// Compute new U
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			BoundingBox * bb111 = node->getCell();															// i, j, k
			BoundingBox * bb011 = oc->getCell(min.x() - 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z())); // i-1,j, k
			BoundingBox * bb211 = oc->getCell(max.x() + 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z()));	// i+1,j, k
			BoundingBox * bb101 = oc->getCell(0.5*(min.x()+max.x()), min.y() - 0.1, 0.5*(min.z()+max.z())); // i,j-1,k
			BoundingBox * bb121 = oc->getCell(0.5*(min.x()+max.x()), max.y() + 0.1, 0.5*(min.z()+max.z()));	// i,j+1,k
			BoundingBox * bb110 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), min.z() - 0.1); // i,j,k-1
			BoundingBox * bb112 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), max.z() + 0.1);	// i,j,k+1
			double new_u_plus = bb111->get_u_plus() + dt * 
				((1/dx) * (square(0.5*(bb111->get_u_plus() + bb011->get_u_plus())) - square(0.5*(bb111->get_u_plus() + bb211->get_u_plus()))) +
				(1/dy) * ( 0.5*(bb101->get_u_plus() + bb111->get_u_plus()) * 0.5*(bb101->get_v_plus() + bb111->get_v_plus())
						 - 0.5*(bb111->get_u_plus() + bb121->get_u_plus()) * 0.5*(bb111->get_v_plus() + bb121->get_v_plus())) + 
				(1/dz) * (0.5*(bb110->get_u_plus() + bb111->get_u_plus()) * 0.5*(bb110->get_w_plus() + bb111->get_w_plus())
						 - 0.5*(bb111->get_u_plus() + bb112->get_u_plus()) * 0.5*(bb111->get_w_plus() + bb112->get_w_plus())) +
				args->gravity.x() +
				(1/dx) * (bb111->getPressure()-bb211->getPressure()) +
				(viscosity/square(dx)) * (bb211->get_u_plus() - 2*bb111->get_u_plus() + bb011->get_u_plus()) +
				(viscosity/square(dy)) * (bb121->get_u_plus() - 2*bb111->get_u_plus() + bb101->get_u_plus()) +
				(viscosity/square(dz)) * (bb112->get_u_plus() - 2*bb111->get_u_plus() + bb110->get_u_plus()) );
			bb111->set_new_u_plus(new_u_plus);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	todo.clear();
	todo.push_back(oc);
	// Compute new V
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			BoundingBox * bb111 = node->getCell();															// i, j, k
			BoundingBox * bb011 = oc->getCell(min.x() - 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z())); // i-1,j, k
			BoundingBox * bb211 = oc->getCell(max.x() + 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z()));	// i+1,j, k
			BoundingBox * bb101 = oc->getCell(0.5*(min.x()+max.x()), min.y() - 0.1, 0.5*(min.z()+max.z())); // i,j-1,k
			BoundingBox * bb121 = oc->getCell(0.5*(min.x()+max.x()), max.y() + 0.1, 0.5*(min.z()+max.z()));	// i,j+1,k
			BoundingBox * bb110 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), min.z() - 0.1); // i,j,k-1
			BoundingBox * bb112 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), max.z() + 0.1);	// i,j,k+1

			double new_v_plus = bb111->get_v_plus() + dt * 
				((1/dx) * (0.5*(bb011->get_u_plus() + bb111->get_u_plus()) * 0.5*(bb011->get_v_plus() + bb111->get_v_plus())
						 - 0.5*(bb111->get_u_plus() + bb211->get_u_plus()) * 0.5*(bb111->get_v_plus() + bb211->get_v_plus())) +
                (1/dy) * (square(0.5*(bb111->get_v_plus() + bb101->get_v_plus())) - square(0.5*(bb111->get_v_plus() + bb121->get_v_plus()))) +
                (1/dz) * (0.5*(bb110->get_v_plus() + bb111->get_v_plus()) * 0.5*(bb110->get_w_plus() + bb111->get_w_plus())
						- 0.5*(bb111->get_v_plus() + bb112->get_v_plus()) * 0.5*(bb111->get_w_plus() + bb112->get_w_plus())) +
                args->gravity.y() +
                (1/dy) * (bb111->getPressure()-bb121->getPressure()) +
                (viscosity/square(dx)) * (bb211->get_v_plus() - 2*bb111->get_v_plus() + bb011->get_v_plus()) +
                (viscosity/square(dy)) * (bb121->get_v_plus() - 2*bb111->get_v_plus() + bb101->get_v_plus()) +
                (viscosity/square(dz)) * (bb112->get_v_plus() - 2*bb111->get_v_plus() + bb110->get_v_plus()) );
			bb111->set_new_v_plus(new_v_plus);
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	todo.clear();
	todo.push_back(oc);
	// Compute new W
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			//get_u_avg = 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)
			//get_uv_plus = 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k));
			Vec3f max = node->getCell()->getMax();
			Vec3f min = node->getCell()->getMin();
			BoundingBox * bb111 = node->getCell();															// i, j, k
			BoundingBox * bb011 = oc->getCell(min.x() - 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z())); // i-1,j, k
			BoundingBox * bb211 = oc->getCell(max.x() + 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z()));	// i+1,j, k
			BoundingBox * bb101 = oc->getCell(0.5*(min.x()+max.x()), min.y() - 0.1, 0.5*(min.z()+max.z())); // i,j-1,k
			BoundingBox * bb121 = oc->getCell(0.5*(min.x()+max.x()), max.y() + 0.1, 0.5*(min.z()+max.z()));	// i,j+1,k
			BoundingBox * bb110 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), min.z() - 0.1); // i,j,k-1
			BoundingBox * bb112 = oc->getCell(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), max.z() + 0.1);	// i,j,k+1

			double new_w_plus = bb111->get_w_plus() + dt * 
				((1/dx) * (0.5*(bb011->get_u_plus() + bb111->get_u_plus()) * 0.5*(bb011->get_w_plus() + bb111->get_w_plus())
						 - 0.5*(bb111->get_u_plus() + bb211->get_u_plus()) * 0.5*(bb111->get_w_plus() + bb211->get_w_plus())) +
                (1/dy) * ( 0.5*(bb101->get_v_plus() + bb111->get_v_plus()) * 0.5*(bb101->get_w_plus() + bb111->get_w_plus())
						 - 0.5*(bb111->get_v_plus() + bb121->get_v_plus()) * 0.5*(bb111->get_w_plus() + bb121->get_w_plus())) + 
                (1/dz) * (square(0.5*(bb111->get_w_plus() + bb110->get_w_plus())) - square(0.5*(bb111->get_w_plus() + bb112->get_w_plus()))) +
                args->gravity.z() +
                (1/dz) * (bb111->getPressure()-bb112->getPressure()) +
                (viscosity/square(dx)) * (bb211->get_w_plus() - 2*bb111->get_w_plus() + bb011->get_w_plus()) +
                (viscosity/square(dy)) * (bb121->get_w_plus() - 2*bb111->get_w_plus() + bb101->get_w_plus()) +
                (viscosity/square(dz)) * (bb112->get_w_plus() - 2*bb111->get_w_plus() + bb110->get_w_plus()) );
			bb111->set_new_w_plus(new_w_plus);
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

void Smoke::EmptyVelocities(BoundingBox *c) {
  
  // NEED TO IMPLEMENT
  if (c->getStatus() != CELL_EMPTY) return;
  Vec3f max = c->getMax();
  BoundingBox *ciplus = oc->getCell(max.x() + 0.01, max.y(), max.z());
  BoundingBox *cjplus = oc->getCell(max.x(), max.y() + 0.01, max.z());
  BoundingBox *ckplus = oc->getCell(max.x(), max.y(), max.z() + 0.01);
  if (ciplus->getStatus() == CELL_EMPTY)
    c->set_new_u_plus(0);
  if (cjplus->getStatus() == CELL_EMPTY)
    c->set_new_v_plus(0);
  if (ckplus->getStatus() == CELL_EMPTY)
    c->set_new_w_plus(0);
}

void Smoke::CopyVelocities() {
	// NEED TO IMPLEMENT
	double dt = args->timestep;
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			BoundingBox *c = node->getCell();
			EmptyVelocities(c);
			c->copyVelocity();
			double dx = c->getMax().x() - c->getMin().x();
			double dy = c->getMax().y() - c->getMin().y();
			double dz = c->getMax().z() - c->getMin().z();
			/*if (fabs(c->get_u_plus()) > 0.5*dx/dt ||
				fabs(c->get_v_plus()) > 0.5*dy/dt ||
				fabs(c->get_w_plus()) > 0.5*dz/dt) {
				// velocity has exceeded reasonable threshhold
				std::cout << "velocity has exceeded reasonable threshhold, stopping animation" << std::endl;
				args->animate=false;
			}*/
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
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
	double dt = args->timestep;
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) {
			std::vector<SmokeParticle*> &particles = node->getParticles();
			for (unsigned int iter = 0; iter < particles.size(); iter++) {
				SmokeParticle *p = particles[iter];
				Vec3f pos = p->getPosition();
				Vec3f vel = getInterpolatedVelocity(pos);
				Vec3f pos2 = pos + vel*dt;
				// euler integration
				p->setPosition(pos2);
				if (!oc->ParticleInCell(p)) p->setPosition(pos);
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
}

// ==============================================================

void Smoke::ReassignParticles() {
	// NEED TO IMPLEMENT
	std::vector<OCTree*> todo;  
	todo.push_back(oc);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (node->isLeaf()) 
		{
			std::vector<SmokeParticle*> particles = node->getParticles();
			for(int i = 0; i < particles.size(); i++)
			{
				SmokeParticle *p = particles[i];
				if (!node->ParticleInCell(p))
				{
					//if(!oc->ParticleInCell(p)) p->setPosition(Vec3f(0.1,0.1,0.1));
					BoundingBox * cell = node->getCell();
					cell->removeParticle(p);
					oc->AddParticle(p);
				}
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		}
	}
	oc->cleanupTree();
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

Vec3f Smoke::getInterpolatedVelocity(const Vec3f &pos) const 
{
	BoundingBox * bb111 = oc->getCell(pos);															// i  ,j  ,k
	Vec3f max = bb111->getMax();
	Vec3f min = bb111->getMin();

	// 9 Cells under
	BoundingBox * bb020 = oc->getCell(Vec3f(min.x() - 0.1, max.y() + 0.1, min.z() - 0.1));					// i-1,j+1,k-1
	BoundingBox * bb120 = oc->getCell(Vec3f(0.5*(max.x()+min.x()), max.y() + 0.1, min.z() - 0.1));			// i  ,j+1,k-1
	BoundingBox * bb220 = oc->getCell(Vec3f(max.x() + 0.1, max.y() + 0.1, min.z() - 0.1));					// i+1,j+1,k-1

	BoundingBox * bb010 = oc->getCell(Vec3f(min.x() - 0.1, 0.5*(max.y()+min.y()), min.z() - 0.1));			// i-1,j,k-1
	BoundingBox * bb110 = oc->getCell(Vec3f(0.5*(max.x()+min.x()), 0.5*(max.y()+min.y()), min.z() - 0.1));	// i  ,j,k-1
	BoundingBox * bb210 = oc->getCell(Vec3f(max.x() + 0.1, 0.5*(max.y()+min.y()), min.z() - 0.1));			// i+1,j,k-1

	BoundingBox * bb000 = oc->getCell(Vec3f(min.x() - 0.1, min.y() - 0.1, min.z() - 0.1));					// i-1,j-1,k-1	
	BoundingBox * bb100 = oc->getCell(Vec3f(0.5*(min.x()+max.x()), min.y() - 0.1, min.z() - 0.1));			// i  ,j-1,k-1
	BoundingBox * bb200 = oc->getCell(Vec3f(max.x() + 0.1, min.y() - 0.1, min.z() - 0.1));					// i+1,j-1,k-1

	// 8 Cells surrounding
	BoundingBox * bb021 = oc->getCell(Vec3f(min.x() - 0.1, max.y() + 0.1, 0.5*(min.z()+max.z())));			// i-1,j+1,k
	BoundingBox * bb121 = oc->getCell(Vec3f(0.5*(min.x()+max.x()), max.y() + 0.1, 0.5*(min.z()+max.z())));	// i  ,j+1,k
	BoundingBox * bb221 = oc->getCell(Vec3f(max.x() + 0.1, max.y() + 0.1, 0.5*(min.z()+max.z())));			// i+1,j+1,k

	BoundingBox * bb011 = oc->getCell(Vec3f(min.x() - 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z()))); // i-1,j  ,k
	//			  bb111																						// i  ,j  ,k
	BoundingBox * bb211 = oc->getCell(Vec3f(max.x() + 0.1, 0.5*(min.y()+max.y()), 0.5*(min.z()+max.z())));	// i+1,j  ,k
	
	BoundingBox * bb001 = oc->getCell(Vec3f(min.x() - 0.1, min.y() - 0.1, 0.5*(min.z()+max.z())));			// i-1,j-1,k
	BoundingBox * bb101 = oc->getCell(Vec3f(0.5*(min.x()+max.x()), min.y() - 0.1, 0.5*(min.z()+max.z()))); // i  ,j-1,k
	BoundingBox * bb201 = oc->getCell(Vec3f(max.x() + 0.1, min.y() - 0.1, 0.5*(min.z()+max.z())));			// i+1,j-1,k
	
	// 9 Cells above
	BoundingBox * bb022 = oc->getCell(Vec3f(min.x() - 0.1, max.y() + 0.1, max.z() + 0.1));					// i-1,j+1,k+1
	BoundingBox * bb122 = oc->getCell(Vec3f(0.5*(max.x() + min.x()), max.y() + 0.1, max.z() + 0.1));		// i  ,j+1,k+1
	BoundingBox * bb222 = oc->getCell(Vec3f(max.x() + 0.1, max.y() + 0.1, max.z() + 0.1));					// i+1,j+1,k+1

	BoundingBox * bb012 = oc->getCell(Vec3f(min.x() - 0.1, 0.5*(max.y()+min.y()), max.z() + 0.1));			// i-1,j  ,k+1
	BoundingBox * bb112 = oc->getCell(Vec3f(0.5*(min.x()+max.x()), 0.5*(min.y()+max.y()), max.z() + 0.1));	// i  ,j  ,k+1
	BoundingBox * bb212 = oc->getCell(Vec3f(max.x() + 0.1, 0.5*(min.y()+max.y()), max.z() + 0.1));			// i+1,j  ,k+1

	BoundingBox * bb002 = oc->getCell(Vec3f(min.x() - 0.1, min.y() - 0.1, max.z() + 0.1));					// i-1,j-1,k+1
	BoundingBox * bb102 = oc->getCell(Vec3f(0.5*(max.x()+min.x()), min.y() - 0.1, max.z() + 0.1));			// i  ,j-1,k+1
	BoundingBox * bb202 = oc->getCell(Vec3f(max.x() + 0.1, min.y() - 0.1, max.z() + 0.1));					// i+1,j-1,k+1
	
	double dx = bb111->getMax().x() - bb111->getMin().x();
	double dy = bb111->getMax().y() - bb111->getMin().y();
	double dz = bb111->getMax().z() - bb111->getMin().z();

	
	double u[8],v[8],w[8];
	double au[8],av[8],aw[8];
	Vec3f average;
	double xWeight, yWeight, zWeight;     
	
	// Calculate the U velocity (x direction)
	if (pos.x() > bb111->getMin().x() + 0.5*dx)			// i+1
	{
		if(pos.y() > bb111->getMin().y() + 0.5*dy)		// j+1
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)	// k+1
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb211->get_u_plus();		// get_u_plus(i + 1,j,k);
				u[2] = bb121->get_u_plus();		// get_u_plus(i,j + 1,k);
				u[3] = bb221->get_u_plus();		// get_u_plus(i+1,j + 1,k); 
				u[4] = bb112->get_u_plus();		// get_u_plus(i,j,k+1);
				u[5] = bb212->get_u_plus();		// get_u_plus(i+1,j,k+1);
				u[6] = bb122->get_u_plus();		// get_u_plus(i,j+1,k+1);
				u[7] = bb222->get_u_plus();		// get_u_plus(i+1,j+1,k+1);
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb211->getMax().x() - bb211->getMin().x(); dy = bb211->getMax().y() - bb211->getMin().y(); dz = bb211->getMax().z() - bb211->getMin().z();
				au[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				dx = bb121->getMax().x() - bb121->getMin().x(); dy = bb121->getMax().y() - bb121->getMin().y(); dz = bb121->getMax().z() - bb121->getMin().z();
				au[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				dx = bb221->getMax().x() - bb221->getMin().x(); dy = bb221->getMax().y() - bb221->getMin().y(); dz = bb221->getMax().z() - bb221->getMin().z();
				au[3] = (1 - ((bb221->getMax().x() - pos.x())/dx)) * (1 - ((bb221->getMax().y()-pos.y())/dy))*(1 - ((bb221->getMax().z() - pos.z())/dz));
				dx = bb112->getMax().x() - bb112->getMin().x(); dy = bb112->getMax().y() - bb112->getMin().y(); dz = bb112->getMax().z() - bb112->getMin().z();
				au[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				dx = bb212->getMax().x() - bb212->getMin().x(); dy = bb212->getMax().y() - bb212->getMin().y(); dz = bb212->getMax().z() - bb212->getMin().z();
				au[5] = (1 - ((bb212->getMax().x() - pos.x())/dx)) * (1 - ((bb212->getMax().y()-pos.y())/dy))*(1 - ((bb212->getMax().z() - pos.z())/dz));
				dx = bb122->getMax().x() - bb122->getMin().x(); dy = bb122->getMax().y() - bb122->getMin().y(); dz = bb122->getMax().z() - bb122->getMin().z();
				au[6] = (1 - ((bb122->getMax().x() - pos.x())/dx)) * (1 - ((bb122->getMax().y()-pos.y())/dy))*(1 - ((bb122->getMax().z() - pos.z())/dz));
				dx = bb222->getMax().x() - bb222->getMin().x(); dy = bb222->getMax().y() - bb222->getMin().y(); dz = bb222->getMax().z() - bb222->getMin().z();
				au[7] = (1 - ((bb222->getMax().x() - pos.x())/dx)) * (1 - ((bb222->getMax().y()-pos.y())/dy))*(1 - ((bb222->getMax().z() - pos.z())/dz));
			}
			else										// k-1
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb211->get_u_plus();		// get_u_plus(i + 1,j,k);
				u[2] = bb121->get_u_plus();		// get_u_plus(i,j + 1,k);
				u[3] = bb221->get_u_plus();		// get_u_plus(i+1,j + 1,k); 
				u[4] = bb110->get_u_plus();		// get_u_plus(i,j,k-1);
				u[5] = bb210->get_u_plus();		// get_u_plus(i+1,j,k-1);
				u[6] = bb120->get_u_plus();		// get_u_plus(i,j+1,k-1);
				u[7] = bb220->get_u_plus();		// get_u_plus(i+1,j+1,k-1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb211->getMax().x() - bb211->getMin().x(); dy = bb211->getMax().y() - bb211->getMin().y(); dz = bb211->getMax().z() - bb211->getMin().z();
				au[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				dx = bb121->getMax().x() - bb121->getMin().x(); dy = bb121->getMax().y() - bb121->getMin().y(); dz = bb121->getMax().z() - bb121->getMin().z();
				au[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				dx = bb221->getMax().x() - bb221->getMin().x(); dy = bb221->getMax().y() - bb221->getMin().y(); dz = bb221->getMax().z() - bb221->getMin().z();
				au[3] = (1 - ((bb221->getMax().x() - pos.x())/dx)) * (1 - ((bb221->getMax().y()-pos.y())/dy))*(1 - ((bb221->getMax().z() - pos.z())/dz));
				dx = bb110->getMax().x() - bb110->getMin().x(); dy = bb212->getMax().y() - bb212->getMin().y(); dz = bb212->getMax().z() - bb212->getMin().z();
				au[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				dx = bb210->getMax().x() - bb210->getMin().x(); dy = bb210->getMax().y() - bb210->getMin().y(); dz = bb210->getMax().z() - bb210->getMin().z();
				au[5] = (1 - ((bb210->getMax().x() - pos.x())/dx)) * (1 - ((bb210->getMax().y()-pos.y())/dy))*(1 - ((bb210->getMax().z() - pos.z())/dz));
				dx = bb120->getMax().x() - bb120->getMin().x(); dy = bb120->getMax().y() - bb120->getMin().y(); dz = bb120->getMax().z() - bb120->getMin().z();
				au[6] = (1 - ((bb120->getMax().x() - pos.x())/dx)) * (1 - ((bb120->getMax().y()-pos.y())/dy))*(1 - ((bb120->getMax().z() - pos.z())/dz));
				dx = bb220->getMax().x() - bb220->getMin().x(); dy = bb220->getMax().y() - bb220->getMin().y(); dz = bb220->getMax().z() - bb220->getMin().z();
				au[7] = (1 - ((bb220->getMax().x() - pos.x())/dx)) * (1 - ((bb220->getMax().y()-pos.y())/dy))*(1 - ((bb220->getMax().z() - pos.z())/dz));
			}
		}
		else											// j-1
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)	// k+1
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb211->get_u_plus();		// get_u_plus(i+1,j,k);
				u[2] = bb101->get_u_plus();		// get_u_plus(i,j-1,k);
				u[3] = bb201->get_u_plus();		// get_u_plus(i+1,j-1,k); 
				u[4] = bb112->get_u_plus();		// get_u_plus(i,j,k+1);
				u[5] = bb212->get_u_plus();		// get_u_plus(i+1,j,k+1);
				u[6] = bb102->get_u_plus();		// get_u_plus(i,j-1,k+1);
				u[7] = bb202->get_u_plus();		// get_u_plus(i+1,j-1,k+1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb211->getMax().x() - bb211->getMin().x(); dy = bb211->getMax().y() - bb211->getMin().y(); dz = bb211->getMax().z() - bb211->getMin().z();
				au[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				dx = bb101->getMax().x() - bb101->getMin().x(); dy = bb101->getMax().y() - bb101->getMin().y(); dz = bb101->getMax().z() - bb101->getMin().z();
				au[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				dx = bb201->getMax().x() - bb201->getMin().x(); dy = bb201->getMax().y() - bb201->getMin().y(); dz = bb201->getMax().z() - bb201->getMin().z();
				au[3] = (1 - ((bb201->getMax().x() - pos.x())/dx)) * (1 - ((bb201->getMax().y()-pos.y())/dy))*(1 - ((bb201->getMax().z() - pos.z())/dz));
				dx = bb112->getMax().x() - bb112->getMin().x(); dy = bb112->getMax().y() - bb112->getMin().y(); dz = bb112->getMax().z() - bb112->getMin().z();
				au[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				dx = bb212->getMax().x() - bb212->getMin().x(); dy = bb212->getMax().y() - bb212->getMin().y(); dz = bb212->getMax().z() - bb212->getMin().z();
				au[5] = (1 - ((bb212->getMax().x() - pos.x())/dx)) * (1 - ((bb212->getMax().y()-pos.y())/dy))*(1 - ((bb212->getMax().z() - pos.z())/dz));
				dx = bb102->getMax().x() - bb102->getMin().x(); dy = bb102->getMax().y() - bb102->getMin().y(); dz = bb102->getMax().z() - bb102->getMin().z();
				au[6] = (1 - ((bb102->getMax().x() - pos.x())/dx)) * (1 - ((bb102->getMax().y()-pos.y())/dy))*(1 - ((bb102->getMax().z() - pos.z())/dz));
				dx = bb202->getMax().x() - bb202->getMin().x(); dy = bb202->getMax().y() - bb202->getMin().y(); dz = bb202->getMax().z() - bb202->getMin().z();
				au[7] = (1 - ((bb202->getMax().x() - pos.x())/dx)) * (1 - ((bb202->getMax().y()-pos.y())/dy))*(1 - ((bb202->getMax().z() - pos.z())/dz));
			}
			else										// k-1
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb211->get_u_plus();		// get_u_plus(i + 1,j,k);
				u[2] = bb101->get_u_plus();		// get_u_plus(i,j-1,k);
				u[3] = bb201->get_u_plus();		// get_u_plus(i+1,j-1,k); 
				u[4] = bb110->get_u_plus();		// get_u_plus(i,j,k-1);
				u[5] = bb210->get_u_plus();		// get_u_plus(i+1,j,k-1);
				u[6] = bb100->get_u_plus();		// get_u_plus(i,j-1,k-1);
				u[7] = bb200->get_u_plus();		// get_u_plus(i+1,j-1,k-1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb211->getMax().x() - bb211->getMin().x(); dy = bb211->getMax().y() - bb211->getMin().y(); dz = bb211->getMax().z() - bb211->getMin().z();
				au[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				dx = bb101->getMax().x() - bb101->getMin().x(); dy = bb101->getMax().y() - bb101->getMin().y(); dz = bb101->getMax().z() - bb101->getMin().z();
				au[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				dx = bb201->getMax().x() - bb201->getMin().x(); dy = bb201->getMax().y() - bb201->getMin().y(); dz = bb201->getMax().z() - bb201->getMin().z();
				au[3] = (1 - ((bb201->getMax().x() - pos.x())/dx)) * (1 - ((bb201->getMax().y()-pos.y())/dy))*(1 - ((bb201->getMax().z() - pos.z())/dz));
				dx = bb110->getMax().x() - bb110->getMin().x(); dy = bb110->getMax().y() - bb110->getMin().y(); dz = bb110->getMax().z() - bb110->getMin().z();
				au[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				dx = bb210->getMax().x() - bb210->getMin().x(); dy = bb210->getMax().y() - bb210->getMin().y(); dz = bb210->getMax().z() - bb210->getMin().z();
				au[5] = (1 - ((bb210->getMax().x() - pos.x())/dx)) * (1 - ((bb210->getMax().y()-pos.y())/dy))*(1 - ((bb210->getMax().z() - pos.z())/dz));
				dx = bb100->getMax().x() - bb100->getMin().x(); dy = bb100->getMax().y() - bb100->getMin().y(); dz = bb100->getMax().z() - bb100->getMin().z();
				au[6] = (1 - ((bb100->getMax().x() - pos.x())/dx)) * (1 - ((bb100->getMax().y()-pos.y())/dy))*(1 - ((bb100->getMax().z() - pos.z())/dz));
				dx = bb200->getMax().x() - bb200->getMin().x(); dy = bb200->getMax().y() - bb200->getMin().y(); dz = bb200->getMax().z() - bb200->getMin().z();
				au[7] = (1 - ((bb200->getMax().x() - pos.x())/dx)) * (1 - ((bb200->getMax().y()-pos.y())/dy))*(1 - ((bb200->getMax().z() - pos.z())/dz));
			}
		}
	}
	else												// i-1
	{
		if(pos.y() > bb111->getMin().y() + 0.5*dy)		// 
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb011->get_u_plus();		// get_u_plus(i-1,j,k);
				u[2] = bb121->get_u_plus();		// get_u_plus(i,j + 1,k);
				u[3] = bb021->get_u_plus();		// get_u_plus(i-1,j + 1,k); 
				u[4] = bb112->get_u_plus();		// get_u_plus(i,j,k+1);
				u[5] = bb012->get_u_plus();		// get_u_plus(i-1,j,k+1);
				u[6] = bb122->get_u_plus();		// get_u_plus(i,j+1,k+1);
				u[7] = bb022->get_u_plus();		// get_u_plus(i-1,j+1,k+1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb011->getMax().x() - bb011->getMin().x(); dy = bb011->getMax().y() - bb011->getMin().y(); dz = bb011->getMax().z() - bb011->getMin().z();
				au[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				dx = bb121->getMax().x() - bb121->getMin().x(); dy = bb121->getMax().y() - bb121->getMin().y(); dz = bb121->getMax().z() - bb121->getMin().z();
				au[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				dx = bb021->getMax().x() - bb021->getMin().x(); dy = bb021->getMax().y() - bb021->getMin().y(); dz = bb021->getMax().z() - bb021->getMin().z();
				au[3] = (1 - ((bb021->getMax().x() - pos.x())/dx)) * (1 - ((bb021->getMax().y()-pos.y())/dy))*(1 - ((bb021->getMax().z() - pos.z())/dz));
				dx = bb112->getMax().x() - bb112->getMin().x(); dy = bb112->getMax().y() - bb112->getMin().y(); dz = bb112->getMax().z() - bb112->getMin().z();
				au[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				dx = bb012->getMax().x() - bb012->getMin().x(); dy = bb012->getMax().y() - bb012->getMin().y(); dz = bb012->getMax().z() - bb012->getMin().z();
				au[5] = (1 - ((bb012->getMax().x() - pos.x())/dx)) * (1 - ((bb012->getMax().y()-pos.y())/dy))*(1 - ((bb012->getMax().z() - pos.z())/dz));
				dx = bb122->getMax().x() - bb122->getMin().x(); dy = bb122->getMax().y() - bb122->getMin().y(); dz = bb122->getMax().z() - bb122->getMin().z();
				au[6] = (1 - ((bb122->getMax().x() - pos.x())/dx)) * (1 - ((bb122->getMax().y()-pos.y())/dy))*(1 - ((bb122->getMax().z() - pos.z())/dz));
				dx = bb022->getMax().x() - bb022->getMin().x(); dy = bb022->getMax().y() - bb022->getMin().y(); dz = bb022->getMax().z() - bb022->getMin().z();
				au[7] = (1 - ((bb022->getMax().x() - pos.x())/dx)) * (1 - ((bb022->getMax().y()-pos.y())/dy))*(1 - ((bb022->getMax().z() - pos.z())/dz));
			}
			else
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb011->get_u_plus();		// get_u_plus(i-1,j,k);
				u[2] = bb121->get_u_plus();		// get_u_plus(i,j + 1,k);
				u[3] = bb021->get_u_plus();		// get_u_plus(i-1,j + 1,k); 
				u[4] = bb110->get_u_plus();		// get_u_plus(i,j,k-1);
				u[5] = bb010->get_u_plus();		// get_u_plus(i-1,j,k-1);
				u[6] = bb120->get_u_plus();		// get_u_plus(i,j+1,k-1);
				u[7] = bb020->get_u_plus();		// get_u_plus(i-1,j+1,k-1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb011->getMax().x() - bb011->getMin().x(); dy = bb011->getMax().y() - bb011->getMin().y(); dz = bb011->getMax().z() - bb011->getMin().z();
				au[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				dx = bb121->getMax().x() - bb121->getMin().x(); dy = bb121->getMax().y() - bb121->getMin().y(); dz = bb121->getMax().z() - bb121->getMin().z();
				au[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				dx = bb021->getMax().x() - bb021->getMin().x(); dy = bb021->getMax().y() - bb021->getMin().y(); dz = bb021->getMax().z() - bb021->getMin().z();
				au[3] = (1 - ((bb021->getMax().x() - pos.x())/dx)) * (1 - ((bb021->getMax().y()-pos.y())/dy))*(1 - ((bb021->getMax().z() - pos.z())/dz));
				dx = bb110->getMax().x() - bb110->getMin().x(); dy = bb110->getMax().y() - bb110->getMin().y(); dz = bb110->getMax().z() - bb110->getMin().z();
				au[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				dx = bb010->getMax().x() - bb010->getMin().x(); dy = bb010->getMax().y() - bb010->getMin().y(); dz = bb010->getMax().z() - bb010->getMin().z();
				au[5] = (1 - ((bb010->getMax().x() - pos.x())/dx)) * (1 - ((bb010->getMax().y()-pos.y())/dy))*(1 - ((bb010->getMax().z() - pos.z())/dz));
				dx = bb120->getMax().x() - bb120->getMin().x(); dy = bb120->getMax().y() - bb120->getMin().y(); dz = bb120->getMax().z() - bb120->getMin().z();
				au[6] = (1 - ((bb120->getMax().x() - pos.x())/dx)) * (1 - ((bb120->getMax().y()-pos.y())/dy))*(1 - ((bb120->getMax().z() - pos.z())/dz));
				dx = bb020->getMax().x() - bb020->getMin().x(); dy = bb020->getMax().y() - bb020->getMin().y(); dz = bb020->getMax().z() - bb020->getMin().z();
				au[7] = (1 - ((bb020->getMax().x() - pos.x())/dx)) * (1 - ((bb020->getMax().y()-pos.y())/dy))*(1 - ((bb020->getMax().z() - pos.z())/dz));
			}
		}
		else
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb011->get_u_plus();		// get_u_plus(i-1,j,k);
				u[2] = bb101->get_u_plus();		// get_u_plus(i,j-1,k);
				u[3] = bb001->get_u_plus();		// get_u_plus(i-1,j-1,k); 
				u[4] = bb112->get_u_plus();		// get_u_plus(i,j,k+1);
				u[5] = bb012->get_u_plus();		// get_u_plus(i-1,j,k+1);
				u[6] = bb102->get_u_plus();		// get_u_plus(i,j-1,k+1);
				u[7] = bb002->get_u_plus();		// get_u_plus(i-1,j-1,k+1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb011->getMax().x() - bb011->getMin().x(); dy = bb011->getMax().y() - bb011->getMin().y(); dz = bb011->getMax().z() - bb011->getMin().z();
				au[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				dx = bb101->getMax().x() - bb101->getMin().x(); dy = bb101->getMax().y() - bb101->getMin().y(); dz = bb101->getMax().z() - bb101->getMin().z();
				au[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				dx = bb001->getMax().x() - bb001->getMin().x(); dy = bb001->getMax().y() - bb001->getMin().y(); dz = bb001->getMax().z() - bb001->getMin().z();
				au[3] = (1 - ((bb001->getMax().x() - pos.x())/dx)) * (1 - ((bb001->getMax().y()-pos.y())/dy))*(1 - ((bb001->getMax().z() - pos.z())/dz));
				dx = bb112->getMax().x() - bb112->getMin().x(); dy = bb112->getMax().y() - bb112->getMin().y(); dz = bb112->getMax().z() - bb112->getMin().z();
				au[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				dx = bb012->getMax().x() - bb012->getMin().x(); dy = bb012->getMax().y() - bb012->getMin().y(); dz = bb012->getMax().z() - bb012->getMin().z();
				au[5] = (1 - ((bb012->getMax().x() - pos.x())/dx)) * (1 - ((bb012->getMax().y()-pos.y())/dy))*(1 - ((bb012->getMax().z() - pos.z())/dz));
				dx = bb102->getMax().x() - bb102->getMin().x(); dy = bb102->getMax().y() - bb102->getMin().y(); dz = bb102->getMax().z() - bb102->getMin().z();
				au[6] = (1 - ((bb102->getMax().x() - pos.x())/dx)) * (1 - ((bb102->getMax().y()-pos.y())/dy))*(1 - ((bb102->getMax().z() - pos.z())/dz));
				dx = bb002->getMax().x() - bb002->getMin().x(); dy = bb002->getMax().y() - bb002->getMin().y(); dz = bb002->getMax().z() - bb002->getMin().z();
				au[7] = (1 - ((bb002->getMax().x() - pos.x())/dx)) * (1 - ((bb002->getMax().y()-pos.y())/dy))*(1 - ((bb002->getMax().z() - pos.z())/dz));
			}
			else
			{
				u[0] = bb111->get_u_plus();		// get_u_plus(i,j,k);
				u[1] = bb011->get_u_plus();		// get_u_plus(i-1,j,k);
				u[2] = bb101->get_u_plus();		// get_u_plus(i,j-1,k);
				u[3] = bb001->get_u_plus();		// get_u_plus(i-1,j-1,k); 
				u[4] = bb110->get_u_plus();		// get_u_plus(i,j,k-1);
				u[5] = bb010->get_u_plus();		// get_u_plus(i-1,j,k-1);
				u[6] = bb100->get_u_plus();		// get_u_plus(i,j-1,k-1);
				u[7] = bb000->get_u_plus();		// get_u_plus(i-1,j-1,k-1);
				dx = bb111->getMax().x() - bb111->getMin().x(); dy = bb111->getMax().y() - bb111->getMin().y(); dz = bb111->getMax().z() - bb111->getMin().z();
				au[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				dx = bb011->getMax().x() - bb011->getMin().x(); dy = bb011->getMax().y() - bb011->getMin().y(); dz = bb011->getMax().z() - bb011->getMin().z();
				au[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				dx = bb101->getMax().x() - bb101->getMin().x(); dy = bb101->getMax().y() - bb101->getMin().y(); dz = bb101->getMax().z() - bb101->getMin().z();
				au[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				dx = bb001->getMax().x() - bb001->getMin().x(); dy = bb001->getMax().y() - bb001->getMin().y(); dz = bb001->getMax().z() - bb001->getMin().z();
				au[3] = (1 - ((bb001->getMax().x() - pos.x())/dx)) * (1 - ((bb001->getMax().y()-pos.y())/dy))*(1 - ((bb001->getMax().z() - pos.z())/dz));
				dx = bb110->getMax().x() - bb110->getMin().x(); dy = bb110->getMax().y() - bb110->getMin().y(); dz = bb110->getMax().z() - bb110->getMin().z();
				au[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				dx = bb010->getMax().x() - bb010->getMin().x(); dy = bb010->getMax().y() - bb010->getMin().y(); dz = bb010->getMax().z() - bb010->getMin().z();
				au[5] = (1 - ((bb010->getMax().x() - pos.x())/dx)) * (1 - ((bb010->getMax().y()-pos.y())/dy))*(1 - ((bb010->getMax().z() - pos.z())/dz));
				dx = bb100->getMax().x() - bb100->getMin().x(); dy = bb100->getMax().y() - bb100->getMin().y(); dz = bb100->getMax().z() - bb100->getMin().z();
				au[6] = (1 - ((bb100->getMax().x() - pos.x())/dx)) * (1 - ((bb100->getMax().y()-pos.y())/dy))*(1 - ((bb100->getMax().z() - pos.z())/dz));
				dx = bb000->getMax().x() - bb000->getMin().x(); dy = bb000->getMax().y() - bb000->getMin().y(); dz = bb000->getMax().z() - bb000->getMin().z();
				au[7] = (1 - ((bb000->getMax().x() - pos.x())/dx)) * (1 - ((bb000->getMax().y()-pos.y())/dy))*(1 - ((bb000->getMax().z() - pos.z())/dz));
			}
		}
	}
	
	average.setx((abs(au[0])*u[0] + abs(au[1])*u[1] + abs(au[2])*u[2] + abs(au[3])*u[3] + abs(au[4])*u[4] + abs(au[5])*u[5] + abs(au[6])*u[6] + abs(au[7])*u[7]));
	/*
	// Calculate the V velocity (y direction)
	if (pos.x() > bb111->getMin().x() + 0.5*dx)			// i+1
	{
		if(pos.y() > bb111->getMin().y() + 0.5*dy)		// j+1
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)	// k+1
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb211->get_v_plus();		// get_v_plus(i + 1,j,k);
				v[2] = bb121->get_v_plus();		// get_v_plus(i,j + 1,k);
				v[3] = bb221->get_v_plus();		// get_v_plus(i+1,j + 1,k); 
				v[4] = bb112->get_v_plus();		// get_v_plus(i,j,k+1);
				v[5] = bb212->get_v_plus();		// get_v_plus(i+1,j,k+1);
				v[6] = bb122->get_v_plus();		// get_v_plus(i,j+1,k+1);
				v[7] = bb222->get_v_plus();		// get_v_plus(i+1,j+1,k+1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb221->getMax().x() - pos.x())/dx)) * (1 - ((bb221->getMax().y()-pos.y())/dy))*(1 - ((bb221->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb212->getMax().x() - pos.x())/dx)) * (1 - ((bb212->getMax().y()-pos.y())/dy))*(1 - ((bb212->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb122->getMax().x() - pos.x())/dx)) * (1 - ((bb122->getMax().y()-pos.y())/dy))*(1 - ((bb122->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb222->getMax().x() - pos.x())/dx)) * (1 - ((bb222->getMax().y()-pos.y())/dy))*(1 - ((bb222->getMax().z() - pos.z())/dz));
			}
			else										// k-1
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb211->get_v_plus();		// get_v_plus(i + 1,j,k);
				v[2] = bb121->get_v_plus();		// get_v_plus(i,j + 1,k);
				v[3] = bb221->get_v_plus();		// get_v_plus(i+1,j + 1,k); 
				v[4] = bb110->get_v_plus();		// get_v_plus(i,j,k-1);
				v[5] = bb210->get_v_plus();		// get_v_plus(i+1,j,k-1);
				v[6] = bb120->get_v_plus();		// get_v_plus(i,j+1,k-1);
				v[7] = bb220->get_v_plus();		// get_v_plus(i+1,j+1,k-1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb221->getMax().x() - pos.x())/dx)) * (1 - ((bb221->getMax().y()-pos.y())/dy))*(1 - ((bb221->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb210->getMax().x() - pos.x())/dx)) * (1 - ((bb210->getMax().y()-pos.y())/dy))*(1 - ((bb210->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb120->getMax().x() - pos.x())/dx)) * (1 - ((bb120->getMax().y()-pos.y())/dy))*(1 - ((bb120->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb220->getMax().x() - pos.x())/dx)) * (1 - ((bb220->getMax().y()-pos.y())/dy))*(1 - ((bb220->getMax().z() - pos.z())/dz));
			}
		}
		else											// j-1
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)	// k+1
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb211->get_v_plus();		// get_v_plus(i+1,j,k);
				v[2] = bb101->get_v_plus();		// get_v_plus(i,j-1,k);
				v[3] = bb201->get_v_plus();		// get_v_plus(i+1,j-1,k); 
				v[4] = bb112->get_v_plus();		// get_v_plus(i,j,k+1);
				v[5] = bb212->get_v_plus();		// get_v_plus(i+1,j,k+1);
				v[6] = bb102->get_v_plus();		// get_v_plus(i,j-1,k+1);
				v[7] = bb202->get_v_plus();		// get_v_plus(i+1,j-1,k+1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb201->getMax().x() - pos.x())/dx)) * (1 - ((bb201->getMax().y()-pos.y())/dy))*(1 - ((bb201->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb212->getMax().x() - pos.x())/dx)) * (1 - ((bb212->getMax().y()-pos.y())/dy))*(1 - ((bb212->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb102->getMax().x() - pos.x())/dx)) * (1 - ((bb102->getMax().y()-pos.y())/dy))*(1 - ((bb102->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb202->getMax().x() - pos.x())/dx)) * (1 - ((bb202->getMax().y()-pos.y())/dy))*(1 - ((bb202->getMax().z() - pos.z())/dz));
			}
			else										// k-1
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb211->get_v_plus();		// get_v_plus(i + 1,j,k);
				v[2] = bb101->get_v_plus();		// get_v_plus(i,j-1,k);
				v[3] = bb201->get_v_plus();		// get_v_plus(i+1,j-1,k); 
				v[4] = bb110->get_v_plus();		// get_v_plus(i,j,k-1);
				v[5] = bb210->get_v_plus();		// get_v_plus(i+1,j,k-1);
				v[6] = bb100->get_v_plus();		// get_v_plus(i,j-1,k-1);
				v[7] = bb200->get_v_plus();		// get_v_plus(i+1,j-1,k-1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb211->getMax().x() - pos.x())/dx)) * (1 - ((bb211->getMax().y()-pos.y())/dy))*(1 - ((bb211->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb201->getMax().x() - pos.x())/dx)) * (1 - ((bb201->getMax().y()-pos.y())/dy))*(1 - ((bb201->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb210->getMax().x() - pos.x())/dx)) * (1 - ((bb210->getMax().y()-pos.y())/dy))*(1 - ((bb210->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb100->getMax().x() - pos.x())/dx)) * (1 - ((bb100->getMax().y()-pos.y())/dy))*(1 - ((bb100->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb200->getMax().x() - pos.x())/dx)) * (1 - ((bb200->getMax().y()-pos.y())/dy))*(1 - ((bb200->getMax().z() - pos.z())/dz));
			}
		}
	}
	else												// i-1
	{
		if(pos.y() > bb111->getMin().y() + 0.5*dy)		// 
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb011->get_v_plus();		// get_v_plus(i-1,j,k);
				v[2] = bb121->get_v_plus();		// get_v_plus(i,j + 1,k);
				v[3] = bb021->get_v_plus();		// get_v_plus(i-1,j + 1,k); 
				v[4] = bb112->get_v_plus();		// get_v_plus(i,j,k+1);
				v[5] = bb012->get_v_plus();		// get_v_plus(i-1,j,k+1);
				v[6] = bb122->get_v_plus();		// get_v_plus(i,j+1,k+1);
				v[7] = bb022->get_v_plus();		// get_v_plus(i-1,j+1,k+1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb021->getMax().x() - pos.x())/dx)) * (1 - ((bb021->getMax().y()-pos.y())/dy))*(1 - ((bb021->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb012->getMax().x() - pos.x())/dx)) * (1 - ((bb012->getMax().y()-pos.y())/dy))*(1 - ((bb012->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb122->getMax().x() - pos.x())/dx)) * (1 - ((bb122->getMax().y()-pos.y())/dy))*(1 - ((bb122->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb022->getMax().x() - pos.x())/dx)) * (1 - ((bb022->getMax().y()-pos.y())/dy))*(1 - ((bb022->getMax().z() - pos.z())/dz));
			}
			else
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb011->get_v_plus();		// get_v_plus(i-1,j,k);
				v[2] = bb121->get_v_plus();		// get_v_plus(i,j + 1,k);
				v[3] = bb021->get_v_plus();		// get_v_plus(i-1,j + 1,k); 
				v[4] = bb110->get_v_plus();		// get_v_plus(i,j,k-1);
				v[5] = bb010->get_v_plus();		// get_v_plus(i-1,j,k-1);
				v[6] = bb120->get_v_plus();		// get_v_plus(i,j+1,k-1);
				v[7] = bb020->get_v_plus();		// get_v_plus(i-1,j+1,k-1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb121->getMax().x() - pos.x())/dx)) * (1 - ((bb121->getMax().y()-pos.y())/dy))*(1 - ((bb121->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb021->getMax().x() - pos.x())/dx)) * (1 - ((bb021->getMax().y()-pos.y())/dy))*(1 - ((bb021->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb010->getMax().x() - pos.x())/dx)) * (1 - ((bb010->getMax().y()-pos.y())/dy))*(1 - ((bb010->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb120->getMax().x() - pos.x())/dx)) * (1 - ((bb120->getMax().y()-pos.y())/dy))*(1 - ((bb120->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb020->getMax().x() - pos.x())/dx)) * (1 - ((bb020->getMax().y()-pos.y())/dy))*(1 - ((bb020->getMax().z() - pos.z())/dz));
			}
		}
		else
		{
			if(pos.z() > bb111->getMin().z() + 0.5*dz)
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb011->get_v_plus();		// get_v_plus(i-1,j,k);
				v[2] = bb101->get_v_plus();		// get_v_plus(i,j-1,k);
				v[3] = bb001->get_v_plus();		// get_v_plus(i-1,j-1,k); 
				v[4] = bb112->get_v_plus();		// get_v_plus(i,j,k+1);
				v[5] = bb012->get_v_plus();		// get_v_plus(i-1,j,k+1);
				v[6] = bb102->get_v_plus();		// get_v_plus(i,j-1,k+1);
				v[7] = bb002->get_v_plus();		// get_v_plus(i-1,j-1,k+1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb001->getMax().x() - pos.x())/dx)) * (1 - ((bb001->getMax().y()-pos.y())/dy))*(1 - ((bb001->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb112->getMax().x() - pos.x())/dx)) * (1 - ((bb112->getMax().y()-pos.y())/dy))*(1 - ((bb112->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb012->getMax().x() - pos.x())/dx)) * (1 - ((bb012->getMax().y()-pos.y())/dy))*(1 - ((bb012->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb102->getMax().x() - pos.x())/dx)) * (1 - ((bb102->getMax().y()-pos.y())/dy))*(1 - ((bb102->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb002->getMax().x() - pos.x())/dx)) * (1 - ((bb002->getMax().y()-pos.y())/dy))*(1 - ((bb002->getMax().z() - pos.z())/dz));
			}
			else
			{
				v[0] = bb111->get_v_plus();		// get_v_plus(i,j,k);
				v[1] = bb011->get_v_plus();		// get_v_plus(i-1,j,k);
				v[2] = bb101->get_v_plus();		// get_v_plus(i,j-1,k);
				v[3] = bb001->get_v_plus();		// get_v_plus(i-1,j-1,k); 
				v[4] = bb110->get_v_plus();		// get_v_plus(i,j,k-1);
				v[5] = bb010->get_v_plus();		// get_v_plus(i-1,j,k-1);
				v[6] = bb100->get_v_plus();		// get_v_plus(i,j-1,k-1);
				v[7] = bb000->get_v_plus();		// get_v_plus(i-1,j-1,k-1);
				av[0] = (1 - ((bb111->getMax().x() - pos.x())/dx)) * (1 - ((bb111->getMax().y()-pos.y())/dy))*(1 - ((bb111->getMax().z() - pos.z())/dz));
				av[1] = (1 - ((bb011->getMax().x() - pos.x())/dx)) * (1 - ((bb011->getMax().y()-pos.y())/dy))*(1 - ((bb011->getMax().z() - pos.z())/dz));
				av[2] = (1 - ((bb101->getMax().x() - pos.x())/dx)) * (1 - ((bb101->getMax().y()-pos.y())/dy))*(1 - ((bb101->getMax().z() - pos.z())/dz));
				av[3] = (1 - ((bb001->getMax().x() - pos.x())/dx)) * (1 - ((bb001->getMax().y()-pos.y())/dy))*(1 - ((bb001->getMax().z() - pos.z())/dz));
				av[4] = (1 - ((bb110->getMax().x() - pos.x())/dx)) * (1 - ((bb110->getMax().y()-pos.y())/dy))*(1 - ((bb110->getMax().z() - pos.z())/dz));
				av[5] = (1 - ((bb010->getMax().x() - pos.x())/dx)) * (1 - ((bb010->getMax().y()-pos.y())/dy))*(1 - ((bb010->getMax().z() - pos.z())/dz));
				av[6] = (1 - ((bb100->getMax().x() - pos.x())/dx)) * (1 - ((bb100->getMax().y()-pos.y())/dy))*(1 - ((bb100->getMax().z() - pos.z())/dz));
				av[7] = (1 - ((bb000->getMax().x() - pos.x())/dx)) * (1 - ((bb000->getMax().y()-pos.y())/dy))*(1 - ((bb000->getMax().z() - pos.z())/dz));
			}
		}
	}
	
	average.sety(av[0]*v[0] + av[1]*v[1] + av[2]*v[2] + av[3]*v[3] + av[4]*v[4] + av[5]*v[5] + av[6]*v[6] + av[7]*v[7]);
	*/
	average.sety(0);
	average.setz(0);

	return average;
}




