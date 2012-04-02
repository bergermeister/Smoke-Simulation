#include "glCanvas.h"

#include <fstream>
#include <algorithm>

#include "smoke.h"
#include "argparser.h"
#include "boundingbox.h"
#include "vectors.h"
#include "matrix.h"
#include "marching_cubes.h"
#include "utils.h"

// ==============================================================
// ==============================================================

void Smoke::initializeVBOs() {
  glGenBuffers(1, &smoke_particles_VBO);
  glGenBuffers(1, &smoke_velocity_vis_VBO);
  glGenBuffers(1, &smoke_face_velocity_vis_VBO);
  glGenBuffers(1, &smoke_pressure_vis_VBO);
  glGenBuffers(1, &smoke_cell_type_vis_VBO);
  marchingCubes->initializeVBOs();
}


void Smoke::setupVBOs() {
  HandleGLError("in setup smoke VBOs");

  smoke_particles.clear();
  smoke_velocity_vis.clear();  
  smoke_face_velocity_vis.clear();
  smoke_pressure_vis.clear();
  smoke_cell_type_vis.clear();

  // =====================================================================================
  // setup the particles
  // =====================================================================================
  for (int x = 0; x < nx; x++) {
    for (int y = 0; y < ny; y++) {
      for (int z = 0; z < nz; z++) {
	Cell *cell = getCell(x,y,z);
	std::vector<FluidParticle*> &particles = cell->getParticles();
	for (unsigned int iter = 0; iter < particles.size(); iter++) {
	  FluidParticle *p = particles[iter];
	  Vec3f v = p->getPosition();
	  smoke_particles.push_back(VBOPos(v));
	}
      }
    }
  }

  // =====================================================================================
  // visualize the velocity
  // =====================================================================================
  if (args->dense_velocity == 0) {
    // one velocity vector per cell, at the centroid
    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < ny; j++) {
	for (int k = 0; k < nz; k++) {
	  Vec3f cell_center((i+0.5)*dx,(j+0.5)*dy,(k+0.5)*dz);
	  Vec3f direction(get_u_avg(i,j,k),get_v_avg(i,j,k),get_w_avg(i,j,k));
	  Vec3f pt2 = cell_center+100*args->timestep*direction;
	  smoke_velocity_vis.push_back(VBOPosColor(cell_center,Vec3f(1,0,0)));
	  smoke_velocity_vis.push_back(VBOPosColor(pt2,Vec3f(1,1,1)));
	}
      }
    }
  } else if (args->dense_velocity == 1) {
    double z = nz*dz / 2.0;
    for (double x = 0; x <= (nx+0.01)*dx; x+=0.25*dx) {
      for (double y = 0; y <= (ny+0.01)*dy; y+=0.25*dy) {
	Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
	Vec3f pt1(x,y,z);
	Vec3f pt2(x+vel.x(),y+vel.y(),z+vel.z());
	smoke_velocity_vis.push_back(VBOPosColor(pt1,Vec3f(1,0,0)));
	smoke_velocity_vis.push_back(VBOPosColor(pt2,Vec3f(1,1,1)));
      } 
    }
  } else if (args->dense_velocity == 2) {
    double y = ny*dy / 2.0;
    for (double x = 0; x <= (nx+0.01)*dx; x+=0.25*dx) {
      for (double z = 0; z <= (nz+0.01)*dz; z+=0.25*dz) {
	Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
	Vec3f pt1(x,y,z);
	Vec3f pt2(x+vel.x(),y+vel.y(),z+vel.z());
	smoke_velocity_vis.push_back(VBOPosColor(pt1,Vec3f(1,0,0)));
	smoke_velocity_vis.push_back(VBOPosColor(pt2,Vec3f(1,1,1)));
      }
    } 
  } else if (args->dense_velocity == 3) {
    double x = nx*dx / 2.0;
    for (double y = 0; y <= (ny+0.01)*dy; y+=0.25*dy) {
      for (double z = 0; z <= (nz+0.01)*dz; z+=0.25*dz) {
	Vec3f vel = getInterpolatedVelocity(Vec3f(x,y,z));
	Vec3f pt1(x,y,z);
	Vec3f pt2(x+vel.x(),y+vel.y(),z+vel.z());
	smoke_velocity_vis.push_back(VBOPosColor(pt1,Vec3f(1,0,0)));
	smoke_velocity_vis.push_back(VBOPosColor(pt2,Vec3f(1,1,1)));
      }
    } 
  }

  // =====================================================================================
  // visualize the face velocity
  // render stubby triangles to visualize the u, v, and w velocities between cell faces
  // =====================================================================================
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
	double u = get_u_plus(i,j,k);
	double v = get_v_plus(i,j,k);
	double w = get_w_plus(i,j,k);
	double x = i*dx;
	double y = j*dy;
	double z = k*dz;
	double dt = args->timestep;
	if (u < -10*dt) {
	  Vec3f pts[5] = { Vec3f(x+dx+u,y+0.5*dy,z+0.5*dz),
			   Vec3f(x+dx,y+0.55*dy,z+0.55*dz),
			   Vec3f(x+dx,y+0.55*dy,z+0.45*dz),
			   Vec3f(x+dx,y+0.45*dy,z+0.45*dz),
			   Vec3f(x+dx,y+0.45*dy,z+0.55*dz) };
	  setupConeVBO(pts,Vec3f(1,0,0),smoke_face_velocity_vis);	  
	} else if (u > 10*dt) {
	  Vec3f pts[5] = { Vec3f(x+dx+u,y+0.5*dy,z+0.5*dz),
			   Vec3f(x+dx,y+0.45*dy,z+0.45*dz),
			   Vec3f(x+dx,y+0.55*dy,z+0.45*dz),
			   Vec3f(x+dx,y+0.55*dy,z+0.55*dz),
			   Vec3f(x+dx,y+0.45*dy,z+0.55*dz) };
	  setupConeVBO(pts,Vec3f(1,0,0),smoke_face_velocity_vis);	  
	}
	if (v < -10*dt) {
	  Vec3f pts[5] = { Vec3f(x+0.5*dx,y+dy+v,z+0.5*dz),
			   Vec3f(x+0.45*dx,y+dy,z+0.45*dz),
			   Vec3f(x+0.55*dx,y+dy,z+0.45*dz),
			   Vec3f(x+0.55*dx,y+dy,z+0.55*dz),
			   Vec3f(x+0.45*dx,y+dy,z+0.55*dz) };
	  setupConeVBO(pts,Vec3f(0,1,0),smoke_face_velocity_vis);	  
	} else if (v > 10*dt) {
	  Vec3f pts[5] = { Vec3f(x+0.5*dx,y+dy+v,z+0.5*dz),
			   Vec3f(x+0.55*dx,y+dy,z+0.55*dz),
			   Vec3f(x+0.55*dx,y+dy,z+0.45*dz),
			   Vec3f(x+0.45*dx,y+dy,z+0.45*dz),
			   Vec3f(x+0.45*dx,y+dy,z+0.55*dz) };
	  setupConeVBO(pts,Vec3f(0,1,0),smoke_face_velocity_vis);	  
	}
	if (w < -10*dt) {
	  Vec3f pts[5] = { Vec3f(x+0.5*dx,y+0.5*dy,z+dz+w),
			   Vec3f(x+0.55*dx,y+0.55*dy,z+dz),
			   Vec3f(x+0.55*dx,y+0.45*dy,z+dz),
			   Vec3f(x+0.45*dx,y+0.45*dy,z+dz),
			   Vec3f(x+0.45*dx,y+0.55*dy,z+dz) };
	  setupConeVBO(pts,Vec3f(0,0,1),smoke_face_velocity_vis);	  
	} else if (w > 10*dt) {
	  Vec3f pts[5] = { Vec3f(x+0.5*dx,y+0.5*dy,z+dz+w),
			   Vec3f(x+0.45*dx,y+0.45*dy,z+dz),
			   Vec3f(x+0.55*dx,y+0.45*dy,z+dz),
			   Vec3f(x+0.55*dx,y+0.55*dy,z+dz),
			   Vec3f(x+0.45*dx,y+0.55*dy,z+dz) };
	  setupConeVBO(pts,Vec3f(0,0,1),smoke_face_velocity_vis);	  
	}
      }
    }
  }

  // =====================================================================================
  // visualize the cell pressure
  // =====================================================================================
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
	Vec3f pts[8] = { Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.1)*dz),
			 Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.9)*dz),
			 Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.1)*dz),
			 Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.9)*dz),
			 Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.1)*dz),
			 Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.9)*dz),
			 Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.1)*dz),
			 Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.9)*dz) };
          double p = getCell(i,j,k)->getPressure();
          p *= 0.1;
          if (p > 1) p = 1;
          if (p < -1) p = -1;
          assert(p >= -1 && p <= 1);
          Vec3f color;
	  if (p < 0) {
            color = Vec3f(1+p,1+p,1);
          } else {
            color = Vec3f(1,1-p,1-p);
          }
	  setupCubeVBO(pts,color,smoke_pressure_vis);
      }
    }
  }

  // =====================================================================================
  // render the MAC cells (FULL, SURFACE, or EMPTY)
  // =====================================================================================
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
	Vec3f pts[8] = { Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.1)*dz),
			 Vec3f((i+0.1)*dx,(j+0.1)*dy,(k+0.9)*dz),
			 Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.1)*dz),
			 Vec3f((i+0.1)*dx,(j+0.9)*dy,(k+0.9)*dz),
			 Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.1)*dz),
			 Vec3f((i+0.9)*dx,(j+0.1)*dy,(k+0.9)*dz),
			 Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.1)*dz),
			 Vec3f((i+0.9)*dx,(j+0.9)*dy,(k+0.9)*dz) };
	Cell *cell = getCell(i,j,k);
	Vec3f color;
	if (cell->getStatus() == CELL_FULL) {
	  color = Vec3f(1,0,0);
	} else if (cell->getStatus() == CELL_SURFACE) {
	  color=Vec3f(0,0,1);
	} else {
	  continue;
	}
	setupCubeVBO(pts,color,smoke_cell_type_vis);
      }
    }
  }

  // cleanup old buffer data (if any)
  cleanupVBOs();

  // copy the data to each VBO
  glBindBuffer(GL_ARRAY_BUFFER,smoke_particles_VBO); 
  glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPos)*smoke_particles.size(),&smoke_particles[0],GL_STATIC_DRAW); 
  if (smoke_velocity_vis.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,smoke_velocity_vis_VBO); 
    glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosColor)*smoke_velocity_vis.size(),&smoke_velocity_vis[0],GL_STATIC_DRAW); 
  }
  if (smoke_face_velocity_vis.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER,smoke_face_velocity_vis_VBO); 
    glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosNormalColor)*smoke_face_velocity_vis.size(),&smoke_face_velocity_vis[0],GL_STATIC_DRAW); 
  }
  glBindBuffer(GL_ARRAY_BUFFER,smoke_pressure_vis_VBO); 
  glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosNormalColor)*smoke_pressure_vis.size(),&smoke_pressure_vis[0],GL_STATIC_DRAW); 
  glBindBuffer(GL_ARRAY_BUFFER,smoke_cell_type_vis_VBO); 
  glBufferData(GL_ARRAY_BUFFER,sizeof(VBOPosNormalColor)*smoke_cell_type_vis.size(),&smoke_cell_type_vis[0],GL_STATIC_DRAW); 

  HandleGLError("leaving setup smoke");

  // =====================================================================================
  // setup a marching cubes representation of the surface
  // =====================================================================================
  for (int i = 0; i <= nx; i++) {
    for (int j = 0; j <= ny; j++) {
      for (int k = 0; k <= nz; k++) {
	marchingCubes->set(i,j,k,interpolateIsovalue(Vec3f((i-0.5),(j-0.5),(k-0.5))));
      } 
    }
  }
  marchingCubes->setupVBOs();
}






void Smoke::drawVBOs() {

  // =====================================================================================
  // render the particles
  // =====================================================================================
  if (args->particles) {
    glColor3f(0,0,0);
    glPointSize(3);
    glBindBuffer(GL_ARRAY_BUFFER, smoke_particles_VBO);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,sizeof(VBOPos), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VBOPos), 0);
    glDrawArrays(GL_POINTS, 0, smoke_particles.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableVertexAttribArray(0);
  }

  // =====================================================================================
  // visualize the average cell velocity
  // =====================================================================================
  if (args->velocity && smoke_velocity_vis.size() > 0) {
    glLineWidth(3); 
    glBindBuffer(GL_ARRAY_BUFFER, smoke_velocity_vis_VBO);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,sizeof(VBOPosColor), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VBOPosColor), 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, sizeof(VBOPosColor),BUFFER_OFFSET(12));
    glDrawArrays(GL_LINES, 0, smoke_velocity_vis.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableVertexAttribArray(0);
  }

  // =====================================================================================
  // visualize the face velocity
  // =====================================================================================
  if (args->face_velocity && smoke_face_velocity_vis.size() > 0) {
    glEnable(GL_LIGHTING);
    glBindBuffer(GL_ARRAY_BUFFER, smoke_face_velocity_vis_VBO);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,sizeof(VBOPosNormalColor), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VBOPosNormalColor), 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(24));
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(12));
    glDrawArrays(GL_TRIANGLES, 0, smoke_face_velocity_vis.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableVertexAttribArray(0);
    glDisable(GL_LIGHTING);
  }

  // =====================================================================================
  // visualize the cell pressure
  // =====================================================================================
  if (args->pressure) {
    glEnable(GL_LIGHTING);
    glBindBuffer(GL_ARRAY_BUFFER, smoke_pressure_vis_VBO);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,sizeof(VBOPosNormalColor), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,sizeof(VBOPosNormalColor), 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(24));
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(12));
    glDrawArrays(GL_QUADS, 0, smoke_pressure_vis.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableVertexAttribArray(0);
    glDisable(GL_LIGHTING);
  }

  // =====================================================================================
  // render the MAC cells (FULL, SURFACE, or EMPTY)
  // =====================================================================================
  if (args->cubes) {
    glEnable(GL_LIGHTING);
    glBindBuffer(GL_ARRAY_BUFFER, smoke_cell_type_vis_VBO);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,sizeof(VBOPosNormalColor), 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VBOPosNormalColor), 0);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(24));
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof(VBOPosNormalColor),BUFFER_OFFSET(12));
    glDrawArrays(GL_QUADS, 0, smoke_cell_type_vis.size());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableVertexAttribArray(0);
    glDisable(GL_LIGHTING);
  }

  // =====================================================================================
  // render a marching cubes representation of the surface
  //    note: make sure you set the Smoke::getIsovalue() to what you want
  // =====================================================================================
  if (args->surface) {
    marchingCubes->drawVBOs();
  } 
}

void Smoke::cleanupVBOs() { 
  glDeleteBuffers(1, &smoke_particles_VBO);
  glDeleteBuffers(1, &smoke_velocity_vis_VBO);  
  glDeleteBuffers(1, &smoke_face_velocity_vis_VBO);  
  glDeleteBuffers(1, &smoke_pressure_vis_VBO);
  glDeleteBuffers(1, &smoke_cell_type_vis_VBO);
}

// ==============================================================

double Smoke::getIsovalue(int i, int j, int k) const {
  i = my_max(0,(my_min(i,nx-1)));
  j = my_max(0,(my_min(j,ny-1)));
  k = my_max(0,(my_min(k,nz-1)));
  Cell *c = getCell(i,j,k);
  if (c->getStatus() == CELL_EMPTY) return 0;
  // note: this is technically not a correct thing to do
  //       the number of particles is not an indication of it's "fullness"
  if (c->getStatus() == CELL_SURFACE) return 0.5 + c->numParticles()/double(density);
  if (c->getStatus() == CELL_FULL) return 2;
  assert(0);
  return 0;
}

// ==============================================================

double Smoke::interpolateIsovalue(const Vec3f &v) const {

  double x = v.x();
  double y = v.y();
  double z = v.z();

  // get the values at the corners
  double a = getIsovalue(int(floor(x)),int(floor(y)),int(floor(z)));
  double b = getIsovalue(int(floor(x)),int(floor(y)),int( ceil(z)));
  double c = getIsovalue(int(floor(x)),int( ceil(y)),int(floor(z)));
  double d = getIsovalue(int(floor(x)),int( ceil(y)),int( ceil(z)));
  double e = getIsovalue(int( ceil(x)),int(floor(y)),int(floor(z)));
  double f = getIsovalue(int( ceil(x)),int(floor(y)),int( ceil(z)));
  double g = getIsovalue(int( ceil(x)),int( ceil(y)),int(floor(z)));
  double h = getIsovalue(int( ceil(x)),int( ceil(y)),int( ceil(z)));

  double x_frac = x - (floor(x));
  double y_frac = y - (floor(y));
  double z_frac = z - (floor(z));

  assert (x_frac >= 0 && x_frac <= 1);
  assert (y_frac >= 0 && y_frac <= 1);
  assert (z_frac >= 0 && z_frac <= 1);
  
  double answer = triInterpolate(x_frac,y_frac,z_frac,a,b,c,d,e,f,g,h);
  return answer;
}


// ==============================================================

void setupCubeVBO(const Vec3f pts[8], const Vec3f &color, std::vector<VBOPosNormalColor> &faces) {
  
  faces.push_back(VBOPosNormalColor(pts[0],Vec3f(-1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[1],Vec3f(-1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[3],Vec3f(-1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[2],Vec3f(-1,0,0),color));
  
  faces.push_back(VBOPosNormalColor(pts[4],Vec3f(1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[6],Vec3f(1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[7],Vec3f(1,0,0),color));
  faces.push_back(VBOPosNormalColor(pts[5],Vec3f(1,0,0),color));
  
  faces.push_back(VBOPosNormalColor(pts[0],Vec3f(0,0,-1),color));
  faces.push_back(VBOPosNormalColor(pts[2],Vec3f(0,0,-1),color));
  faces.push_back(VBOPosNormalColor(pts[6],Vec3f(0,0,-1),color));
  faces.push_back(VBOPosNormalColor(pts[4],Vec3f(0,0,-1),color));
  
  faces.push_back(VBOPosNormalColor(pts[1],Vec3f(0,0,1),color));
  faces.push_back(VBOPosNormalColor(pts[5],Vec3f(0,0,1),color));
  faces.push_back(VBOPosNormalColor(pts[7],Vec3f(0,0,1),color));
  faces.push_back(VBOPosNormalColor(pts[3],Vec3f(0,0,1),color));
  
  faces.push_back(VBOPosNormalColor(pts[0],Vec3f(0,-1,0),color));
  faces.push_back(VBOPosNormalColor(pts[4],Vec3f(0,-1,0),color));
  faces.push_back(VBOPosNormalColor(pts[5],Vec3f(0,-1,0),color));
  faces.push_back(VBOPosNormalColor(pts[1],Vec3f(0,-1,0),color));
	  
  faces.push_back(VBOPosNormalColor(pts[2],Vec3f(0,1,0),color));
  faces.push_back(VBOPosNormalColor(pts[3],Vec3f(0,1,0),color));
  faces.push_back(VBOPosNormalColor(pts[7],Vec3f(0,1,0),color));
  faces.push_back(VBOPosNormalColor(pts[6],Vec3f(0,1,0),color));
}


void setupConeVBO(const Vec3f pts[5], const Vec3f &color, std::vector<VBOPosNormalColor> &faces) {

  Vec3f normal = computeNormal(pts[0],pts[1],pts[2]);
  faces.push_back(VBOPosNormalColor(pts[0],normal,Vec3f(1,1,1)));
  faces.push_back(VBOPosNormalColor(pts[1],normal,color));
  faces.push_back(VBOPosNormalColor(pts[2],normal,color));

  normal = computeNormal(pts[0],pts[2],pts[3]);
  faces.push_back(VBOPosNormalColor(pts[0],normal,Vec3f(1,1,1)));
  faces.push_back(VBOPosNormalColor(pts[2],normal,color));
  faces.push_back(VBOPosNormalColor(pts[3],normal,color));

  normal = computeNormal(pts[0],pts[3],pts[4]);
  faces.push_back(VBOPosNormalColor(pts[0],normal,Vec3f(1,1,1)));
  faces.push_back(VBOPosNormalColor(pts[3],normal,color));
  faces.push_back(VBOPosNormalColor(pts[4],normal,color));

  normal = computeNormal(pts[0],pts[4],pts[1]);
  faces.push_back(VBOPosNormalColor(pts[0],normal,Vec3f(1,1,1)));
  faces.push_back(VBOPosNormalColor(pts[4],normal,color));
  faces.push_back(VBOPosNormalColor(pts[1],normal,color));

  normal = computeNormal(pts[1],pts[3],pts[2]);
  faces.push_back(VBOPosNormalColor(pts[1],normal,color));
  faces.push_back(VBOPosNormalColor(pts[3],normal,color));
  faces.push_back(VBOPosNormalColor(pts[2],normal,color));
  faces.push_back(VBOPosNormalColor(pts[1],normal,color));
  faces.push_back(VBOPosNormalColor(pts[4],normal,color));
  faces.push_back(VBOPosNormalColor(pts[3],normal,color));
}

// ==============================================================
