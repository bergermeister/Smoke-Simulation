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
  delete [] cells; 
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

  // load in the grid size & dimensions
  istr >> token >> nx >> ny >> nz;  assert (token=="grid");
  assert (nx > 0 && ny > 0 && nz > 0);
  istr >> token >> dx >> dy >> dz; assert (token=="cell_dimensions");
  cells = new Cell[(nx+2)*(ny+2)*(nz+2)];

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
  if (token2 == "zero") {
    // default is zero
  } else {
    assert (token2 == "random");
    int i,j,k;
    double max_dim = my_max(dx,my_max(dy,dz));
    for (i = -1; i <= nx; i++) {
      for (j = -1; j <= ny; j++) {
        for (k = -1; k <= nz; k++) {
          getCell(i,j,k)->set_u_plus((2*args->mtrand.rand()-1)*max_dim);
	  getCell(i,j,k)->set_v_plus((2*args->mtrand.rand()-1)*max_dim);
	  getCell(i,j,k)->set_w_plus((2*args->mtrand.rand()-1)*max_dim);
        }
      }
    }
  }
  // read in custom velocities
  while(istr >> token) {
    int i,j,k;
    double velocity;
    assert (token == "u" || token == "v" || token == "w");
    istr >> i >> j >> k >> velocity;
    assert(i >= 0 && i < nx);
    assert(j >= 0 && j < ny);
    assert(k >= 0 && k < nz);
    if      (token == "u") getCell(i,j,k)->set_u_plus(velocity);
    else if (token == "v") getCell(i,j,k)->set_v_plus(velocity);
    else if (token == "w") getCell(i,j,k)->set_w_plus(velocity);
    else assert(0);
  }
  SetBoundaryVelocities();
}

// ==============================================================

bool Smoke::inShape(Vec3f &pos, const std::string &shape) {
  // return true if this point is inside the "shape"
  // defined procedurally (using an implicit surface)
  if (shape == "everywhere") {
    return true;
  } else if (shape == "left") {
    // a blob of particles on the lower left (for the dam)
    return (pos.x() < 0.2*nx*dx && pos.y() < 0.5*ny*dy);
  } else if (shape == "drop") {
    // a shallow pool of particles on the bottom
    double h = ny*dy/6.0;
    if (pos.y() < 2*h) return true;
    // and a sphere of particles above
    Vec3f center = Vec3f(nx*dx*0.5, 5*h,nz*dz*0.5);
    double length = (center-pos).Length();
    if (length < 0.8*h) return true;
    return false;
  } else {
    std::cout << "unknown shape: " << shape << std::endl;
    exit(0);
  }
}

// ==============================================================

void Smoke::GenerateParticles(const std::string &shape, const std::string &placement) {
  // create a set of points according to the "placement" token,
  // then check whether they are inside of the "shape"
  if (placement == "uniform") {
    int dens = (int)pow(density,0.334);
    assert (dens*dens*dens == density);
    // the uniform grid spacing
    double spacing = 1/double(dens);
    for (double x = 0.5*spacing*dx; x < nx*dx; x += spacing*dx) {
      for (double y = 0.5*spacing*dy; y < ny*dy; y += spacing*dy) {
        for (double z = 0.5*spacing*dz; z < nz*dz; z += spacing*dz) {
          Vec3f pos = Vec3f(x,y,z);
          if (inShape(pos,shape)) {
            Cell *cell = getCell(int(x/dx),int(y/dy),int(z/dz));
            SmokeParticle *p = new SmokeParticle();
            p->setPosition(pos);
            cell->addParticle(p);
          }
        }
      }
    }
  } else {
    assert (placement == "random");
    // note: we don't necessarily have the same number of particles in each cell
    for (int n = 0; n < nx*ny*nz*density; n++) {
      Vec3f pos = Vec3f(args->mtrand.rand()*nx*dx,
                        args->mtrand.rand()*ny*dy,
                        args->mtrand.rand()*nz*dz);
      if (inShape(pos,shape)) {      
        Cell *cell = getCell(int(pos.x()/dx),int(pos.y()/dy),int(pos.z()/dz));
        SmokeParticle *p = new SmokeParticle();
        p->setPosition(pos);
        cell->addParticle(p);
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

  // advanced the particles through the fluid
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

  for (i = 0; i < nx-1; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        Cell *cell = getCell(i,j,k);
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
        cell->set_new_u_plus(new_u_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny-1; j++) {
      for (k = 0; k < nz; k++) {	
        Cell *cell = getCell(i,j,k);
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
        cell->set_new_v_plus(new_v_plus);
      }
    }
  }

  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz-1; k++) {
        Cell *cell = getCell(i,j,k);
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
        cell->set_new_w_plus(new_w_plus);
      }
    }
  }
}


// ==============================================================

void Smoke::SetBoundaryVelocities() {

  // zero out flow perpendicular to the boundaries (no sources or sinks)
  for (int j = -1; j <= ny; j++) {
    for (int k = -1; k <= nz; k++) {
      getCell(-1  ,j,k)->set_u_plus(0);
      getCell(nx-1,j,k)->set_u_plus(0);
      getCell(nx  ,j,k)->set_u_plus(0);
    }
  }
  for (int i = -1; i <= nx; i++) {
    for (int k = -1; k <= nz; k++) {
      getCell(i,-1  ,k)->set_v_plus(0);
      getCell(i,ny-1,k)->set_v_plus(0);
      getCell(i,ny  ,k)->set_v_plus(0);
    }
  }
  for (int i = -1; i <= nx; i++) {
    for (int j = -1; j <= ny; j++) {
      getCell(i,j,-1  )->set_w_plus(0);
      getCell(i,j,nz-1)->set_w_plus(0);
      getCell(i,j,nz  )->set_w_plus(0);
    }
  }

  // free slip or no slip boundaries (friction with boundary)
  double xy_sign = (xy_free_slip) ? 1 : -1;
  double yz_sign = (yz_free_slip) ? 1 : -1;
  double zx_sign = (zx_free_slip) ? 1 : -1;
  for (int i = 0; i < nx; i++) {
    for (int j = -1; j <= ny; j++) {
      getCell(i,j,-1)->set_u_plus(xy_sign*getCell(i,j,0)->get_u_plus());
      getCell(i,j,nz)->set_u_plus(xy_sign*getCell(i,j,nz-1)->get_u_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getCell(i,-1,k)->set_u_plus(zx_sign*getCell(i,0,k)->get_u_plus());
      getCell(i,ny,k)->set_u_plus(zx_sign*getCell(i,ny-1,k)->get_u_plus());
    }
  }
  for (int j = 0; j < ny; j++) {
    for (int i = -1; i <= nx; i++) {
      getCell(i,j,-1)->set_v_plus(xy_sign*getCell(i,j,0)->get_v_plus());
      getCell(i,j,nz)->set_v_plus(xy_sign*getCell(i,j,nz-1)->get_v_plus());
    }
    for (int k = -1; k <= nz; k++) {
      getCell(-1,j,k)->set_v_plus(yz_sign*getCell(0,j,k)->get_v_plus());
      getCell(nx,j,k)->set_v_plus(yz_sign*getCell(nx-1,j,k)->get_v_plus());
    }
  }
  for (int k = 0; k < nz; k++) {
    for (int i = -1; i <= nx; i++) {
      getCell(i,-1,k)->set_w_plus(zx_sign*getCell(i,0,k)->get_w_plus());
      getCell(i,ny,k)->set_w_plus(zx_sign*getCell(i,ny-1,k)->get_w_plus());
    }
    for (int j = -1; j <= ny; j++) {
      getCell(-1,j,k)->set_w_plus(yz_sign*getCell(0,j,k)->get_w_plus());
      getCell(nx,j,k)->set_w_plus(yz_sign*getCell(nx-1,j,k)->get_w_plus());
    }
  }
}

// ==============================================================

void Smoke::EmptyVelocities(int i, int j, int k) {
  Cell *c = getCell(i,j,k);
  if (c->getStatus() != CELL_EMPTY) return;
  Cell *ciplus = getCell(i+1,j,k);
  Cell *cjplus = getCell(i,j+1,k);
  Cell *ckplus = getCell(i,j,k+1);
  if (ciplus->getStatus() == CELL_EMPTY)
    c->set_new_u_plus(0);
  if (cjplus->getStatus() == CELL_EMPTY)
    c->set_new_v_plus(0);
  if (ckplus->getStatus() == CELL_EMPTY)
    c->set_new_w_plus(0);
}

void Smoke::CopyVelocities() {
  double dt = args->timestep;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
	Cell *c = getCell(i,j,k);
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
}

// ==============================================================

double Smoke::AdjustForIncompressibility() {


  // *********************************************************************  
  // ASSIGNMENT:
  //
  // This is not a complete implementation of the Marker and Cell (MAC) method.
  // Additional boundary velocities should be equalized as described in the references
  // depending on whether the boundaries are free-slip or no-slip.
  //
  // Also play around with compressible flow!
  //
  // *********************************************************************    
	double max_div = 0;
	
	for(int i = 0; i <= nx; i++){
		for(int j = 0; j <= ny; j++){
			for(int k = 0; k <= nz; k++){
				double div = 0;
				if(i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz){
					int count = 0;
					div = -((1/dx)*(get_new_u_plus(i,j,k) - get_new_u_plus(i-1, j, k)) +
							(1/dy)*(get_new_v_plus(i,j,k) - get_new_v_plus(i,j-1,k)) +
							(1/dz)*(get_new_w_plus(i,j,k) - get_new_w_plus(i,j,k-1)));

					if(i > 0)
						count++;
					if(j > 0)
						count++;
					if(k > 0)
						count++;
					if(i<nx-1)
						count++;
					if(j<ny-1)
						count++;
					if(k<nz-1)
						count++;
					
					if(i < nx-1 && getCell(i-1,j,k)->getStatus() != CELL_EMPTY) adjust_new_u_plus(i,j,k, div/count);
					if(j < ny-1 && getCell(i,j-1,k)->getStatus() != CELL_EMPTY) adjust_new_v_plus(i,j,k, div/count);
					if(k < nz-1 && getCell(i,j,k-1)->getStatus() != CELL_EMPTY) adjust_new_w_plus(i,j,k, div/count);
					if(i > 0 && getCell(i+1,j,k)->getStatus() != CELL_EMPTY) adjust_new_u_plus(i-1,j,k, -div/count);
					if(j > 0 && getCell(i,j+1,k)->getStatus() != CELL_EMPTY) adjust_new_v_plus(i,j-1,k, -div/count);
					if(k > 0 && getCell(i,j,k+1)->getStatus() != CELL_EMPTY) adjust_new_w_plus(i,j,k-1, -div/count);
				}
				
				if(max_div < abs(div)) max_div = abs(div);
				
			}
		}
	}
	//std::cout << max_div << std::endl;
  // return the divergence (will be repeated while divergence > threshold)
  return max_div;
}

// ==============================================================

void Smoke::UpdatePressures() {
  for (int i = -1; i <= nx; i++) {
    for (int j = -1; j <= ny; j++) {
      for (int k = -1; k <= nz; k++) {
		Cell *c = getCell(i,j,k);
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
		  /*
		  std::cout << "dp::" << dp << '\t' << "divergence::" << divergence << std::endl;
		  std::cout<< "Pressure::Cell(" << i << "," << j << "," << k << ") u = " << get_new_u_plus(i,j,k) << std::endl;
		  std::cout<< "Pressure::Cell(" << i << "," << j << "," << k << ") v = " << get_new_v_plus(i,j,k) << std::endl;
		  std::cout<< "Pressure::Cell(" << i << "," << j << "," << k << ") w = " << get_new_w_plus(i,j,k) << std::endl;
		  */
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
}

// ==============================================================

void Smoke::MoveParticles() {
  double dt = args->timestep;
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        Cell *cell = getCell(i,j,k);
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
}

// ==============================================================

void Smoke::ReassignParticles() {
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        Cell *cell = getCell(i,j,k);
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
            getCell(i2,j2,k2)->addParticle(p);
          } 
        }
      }
    }
  }
}

// ==============================================================

void Smoke::SetEmptySurfaceFull() {
  int i,j,k;
  for (i = 0; i < nx; i++) {
    for (j = 0; j < ny; j++) {
      for (k = 0; k < nz; k++) {
        Cell *cell = getCell(i,j,k);
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
        Cell *cell = getCell(i,j,k);
        if (cell->getStatus() == CELL_FULL &&
            (getCell(i-1,j,k)->getStatus() == CELL_EMPTY ||
             getCell(i+1,j,k)->getStatus() == CELL_EMPTY ||
             getCell(i,j-1,k)->getStatus() == CELL_EMPTY ||
             getCell(i,j+1,k)->getStatus() == CELL_EMPTY ||
             getCell(i,j,k-1)->getStatus() == CELL_EMPTY ||
             getCell(i,j,k+1)->getStatus() == CELL_EMPTY)) {
          cell->setStatus(CELL_SURFACE);
        }
      }
    }
  }
}

// ==============================================================

Vec3f Smoke::getInterpolatedVelocity(const Vec3f &pos) const {


  // *********************************************************************  
  // ASSIGNMENT:
  //
  // I've intentionally reverted to the "dumb" velocity interpolation.
  // Do it right, as described in the papers.
  //
	double u[8], v[8], w[8];
	double A[8], B[8], C[8];
	
	int i = int(floor(pos.x()/dx)); if (i < 0) i = 0; if (i >= nx) i = nx-1;
	int j = int(floor(pos.y()/dy)); if (j < 0) j = 0; if (j >= ny) j = ny-1;
	int k = int(floor(pos.z()/dz)); if (k < 0) k = 0; if (k >= nz) k = nz-1;
	int i2, j2, k2;

	if(pos.x() < i + 0.5*dx) i2 = i - 1;
	else i2 = i + 1;
	if(pos.y() < j + 0.5*dy) j2 = j - 1;
	else j2 = j + 1;
	if(pos.z() < k + 0.5*dz) k2 = k - 1;
	else k2 = k + 1;

	u[0] = get_u_plus(i, j2, k);
	u[1] = get_u_plus(i-1, j2, k);
	u[2] = get_u_plus(i-1, j, k);
	u[3] = get_u_plus(i, j, k);
	/*
	u[4] = get_u_plus(i, j2, k2);
	u[5] = get_u_plus(i-dx, j2, k2);
	u[6] = get_u_plus(i-dx, j, k2);
	u[7] = get_u_plus(i, j, k2);
	*/
	A[0] = (dx - (i + dx - pos.x())) * (dy - abs(j2 + 0.5*dy - pos.y()));
	A[1] = (dx - (pos.x() - i)) * (dy - abs(j2 + 0.5*dy - pos.y()));
	A[2] = (dx - (pos.x() - i)) * (dy - abs(j + 0.5*dy - pos.y()));
	A[3] = (dx - (i + dx - pos.x())) * (dy - abs(j + 0.5*dy - pos.y()));
	
	//std::cout<<A[0] << '+' << A[1] << '+' << A[2] << '+' << A[3] << '=' << A[0] + A[1] + A[2] + A[3] << std::endl;
	double x = (A[0]*u[0] + A[1]*u[1] + A[2]*u[2] + A[3]*u[3]);

	v[0] = get_v_plus(i2, j, k);
	v[1] = get_v_plus(i, j, k);
	v[2] = get_v_plus(i, j-1, k);
	v[3] = get_v_plus(i2, j-1, k);
	/*
	v[4] = get_v_plus(i2, j, k2);
	v[5] = get_v_plus(i, j, k2);
	v[6] = get_v_plus(i, j-dy, k2);
	v[7] = get_v_plus(i2, j-dy, k2);
	*/
	B[0] = (dx - abs(i2 + 0.5*dx - pos.x())) * (dy - (j + dy - pos.y()));
	B[1] = (dx - abs(i + 0.5*dx - pos.x())) * (dy - (j + dy - pos.y()));
	B[2] = (dx - abs(i + 0.5*dx - pos.x())) * (dy - (pos.y() - j));
	B[3] = (dx - abs(i2 + 0.5*dx - pos.x())) * (dy - (pos.y() - j));
	
	double y = (B[0]*v[0] + B[1]*v[1] + B[2]*v[2] + B[3]*v[3]);

	/*
	if(pos.x() < i + 0.5*dx) i2 = i - dx;
	else i2 = i + dx;
	if(pos.y() < j + 0.5*dy) j2 = j - dy;
	else j2 = j + dy;
	if(pos.z() < k + 0.5*dz) k2 = k - dz;
	else k2 = k + dz;

	u[0] = get_u_plus(i, j2, k);
	u[1] = get_u_plus(i-dx, j2, k);
	u[2] = get_u_plus(i-dx, j, k);
	u[3] = get_u_plus(i, j, k);
	
	//u[4] = get_u_plus(i, j2, k2);
	//u[5] = get_u_plus(i-dx, j2, k2);
	//u[6] = get_u_plus(i-dx, j, k2);
	//u[7] = get_u_plus(i, j, k2);
	
	A[0] = (1 - (i + dx - pos.x())) * (1 - abs(j2 + 0.5*dy - pos.y()));
	A[1] = (1 - (pos.x() - i)) * (1 - abs(j2 + 0.5*dy - pos.y()));
	A[2] = (1 - (pos.x() - i)) * (1 - abs(j + 0.5*dy - pos.y()));
	A[3] = (1 - (i + dx - pos.x())) * (1 - abs(j + 0.5*dy - pos.y()));

	double x = A[0]*u[0] + A[1]*u[1] + A[2]*u[2] + A[3]*u[3];

	v[0] = get_v_plus(i2, j, k);
	v[1] = get_v_plus(i, j, k);
	v[2] = get_v_plus(i, j-dy, k);
	v[3] = get_v_plus(i2, j-dy, k);
	
	//v[4] = get_v_plus(i2, j, k2);
	//v[5] = get_v_plus(i, j, k2);
	//v[6] = get_v_plus(i, j-dy, k2);
	//v[7] = get_v_plus(i2, j-dy, k2);
	
	B[0] = (1 - abs(i2 + 0.5*dx - pos.x())) * (1 - (j + dy - pos.y()));
	B[1] = (1 - abs(i + 0.5*dx - pos.x())) * (1 - (j + dy - pos.y()));
	B[2] = (1 - abs(i + 0.5*dx - pos.x())) * (1 - (pos.y() - j));
	B[3] = (1 - abs(i2 + 0.5*dx - pos.x())) * (1 - (pos.y() - j));

	double y = B[0]*v[0] + B[1]*v[1] + B[2]*v[2] + B[3]*v[3];
	*/
  return Vec3f(x,y,k);
	//return Vec3f(i,j,k);
  //
  // *********************************************************************  

}