#ifndef _SMOKE_H_
#define _SMOKE_H_

#include <cassert>
#include <vector>
#include "argparser.h"
#include "boundingbox.h"
#include "vectors.h"
#include "vbo_structs.h"
#include "octree.h"
#include "ray.h"

class ArgParser;
class MarchingCubes;
//woooooooooooooooooooooooooooooooooooooooo
// ========================================================================
// ========================================================================
// data structure to store a segment

class Segment {

public:
  // CONSTRUCTOR
  Segment(const Ray &ray, double tstart, double tstop) {
    // first clamp the segment to "reasonable" values 
    // to make sure it is drawn correctly in OpenGL
    if (tstart < -1000) tstart = -1000;
    if (tstop  >  1000) tstop  =  1000;
    a = ray.pointAtParameter(tstart);
    b = ray.pointAtParameter(tstop); }
  const Vec3f& getStart() const { return a; }
  const Vec3f& getEnd() const { return b; }
private:
  // REPRESENTATION
  Vec3f a;
  Vec3f b;
};


// ========================================================================
// ========================================================================

class Smoke {

public:
	OCTree *oc;
  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  Smoke(ArgParser *_args);
  ~Smoke();
  void Load();
   Vec3f background_color;
  void initializeVBOs(); 
  void setupVBOs(); 
  void drawVBOs();
  void setupVBOsR(); 
  void cleanupVBOs();

  // ===============================
  // ANIMATION & RENDERING FUNCTIONS
  void Animate();
  BoundingBox getBoundingBox() const {
    return BoundingBox(Vec3f(0,0,0),Vec3f(nx,ny,nz)); }

   //=================================================================================
  //Rendering
   // most of the time the RayTree is NOT activated, so the segments are not updated
  static void Activate() { Clear(); activated = 1; }
  static void Deactivate() { activated = 0; }

  // when activated, these function calls store the segments of the tree
  static void AddMainSegment(const Ray &ray, double tstart, double tstop) {
    if (!activated) return;
    main_segments.push_back(Segment(ray,tstart,tstop));
  }
  static void AddShadowSegment(const Ray &ray, double tstart, double tstop) {
    if (!activated) return;
    shadow_segments.push_back(Segment(ray,tstart,tstop));
  }
  static void AddReflectedSegment(const Ray &ray, double tstart, double tstop) {
    if (!activated) return;
    reflected_segments.push_back(Segment(ray,tstart,tstop));
  }
  static void AddTransmittedSegment(const Ray &ray, double tstart, double tstop) {
    if (!activated) return;
    transmitted_segments.push_back(Segment(ray,tstart,tstop));
  }

    BoundingBox * grid;
    std::vector<VBOPos> smoke_particlesHit;
	GLuint smoke_particles_Hit_VBO;
private:

  // ==============
  // CELL ACCESSORS
  int Index(int i, int j, int k) const {
    assert (i >= -1 && i <= nx);
    assert (j >= -1 && j <= ny);
    assert (k >= -1 && k <= nz);
    return (i+1)*(ny+2)*(nz+2) + (j+1)*(nz+2) + (k+1);
  }
  BoundingBox *getCell(int i, int j, int k) const { return oc->getCell(i, j, k); }

  // =================
  // ANIMATION HELPERS
  void ComputeNewVelocities();
  void SetBoundaryVelocities();
  void EmptyVelocities(int i, int j, int k);
  void CopyVelocities();
  double AdjustForIncompressibility();
  void UpdatePressures();
  void MoveParticles();
  void ReassignParticles();
  void SetEmptySurfaceFull();

  // =====================
  // NAVIER-STOKES HELPERS
  Vec3f getInterpolatedVelocity(const Vec3f &pos) const;
  double getPressure(int i, int j, int k) const { return getCell(i,j,k)->getPressure(); }
  // velocity accessors
  double get_u_plus(int i, int j, int k) const { return getCell(i,j,k)->get_u_plus(); }
  double get_v_plus(int i, int j, int k) const { return getCell(i,j,k)->get_v_plus(); }  
  double get_w_plus(int i, int j, int k) const { return getCell(i,j,k)->get_w_plus(); }  
  double get_new_u_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_u_plus(); }  
  double get_new_v_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_v_plus(); }  
  double get_new_w_plus(int i, int j, int k) const { return getCell(i,j,k)->get_new_w_plus(); }  
  double get_u_avg(int i, int j, int k) const { return 0.5*(get_u_plus(i-1,j,k)+get_u_plus(i,j,k)); }
  double get_v_avg(int i, int j, int k) const { return 0.5*(get_v_plus(i,j-1,k)+get_v_plus(i,j,k)); }
  double get_w_avg(int i, int j, int k) const { return 0.5*(get_w_plus(i,j,k-1)+get_w_plus(i,j,k)); }
  double get_uv_plus(int i, int j, int k) const { 
    return 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j+1,k)) * 0.5*(get_v_plus(i,j,k) + get_v_plus(i+1,j,k)); }
  double get_uw_plus(int i, int j, int k) const { 
    return 0.5*(get_u_plus(i,j,k) + get_u_plus(i,j,k+1)) * 0.5*(get_w_plus(i,j,k) + get_w_plus(i+1,j,k)); }
  double get_vw_plus(int i, int j, int k) const { 
    return 0.5*(get_v_plus(i,j,k) + get_v_plus(i,j,k+1)) * 0.5*(get_w_plus(i,j,k) + get_w_plus(i,j+1,k)); }
  // velocity modifiers
  void set_new_u_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_u_plus(f); }
  void set_new_v_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_v_plus(f); }
  void set_new_w_plus(int i, int j, int k, double f) { getCell(i,j,k)->set_new_w_plus(f); }
  void adjust_new_u_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_u_plus(f); }
  void adjust_new_v_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_v_plus(f); }
  void adjust_new_w_plus(int i, int j, int k, double f) { getCell(i,j,k)->adjust_new_w_plus(f); }

  // ========================================
  // RENDERING SURFACE (using Marching Cubes)
  double interpolateIsovalue(const Vec3f &c) const;
  double getIsovalue(int i, int j, int k) const;

  // ============
  // LOAD HELPERS
  bool inShape(Vec3f &pos, const std::string &shape);
  void GenerateParticles(const std::string &shape, const std::string &placement);

private:

  // HELPER FUNCTIONS:
  double IncompressibleFullCell(int i, int j, int k);
  void CompressibleSurfaceCell(int i, int j, int k);

  // don't use this constructor
  Smoke() { assert(0); }
  
  // ==============
  // REPRESENTATION
  ArgParser *args;

  // Smoke parameters
  int nx,ny,nz;     // number of grid cells in each dimension
  double dx,dy,dz;  // dimensions of each grid cell
  

  //Cell *cells;      // NOTE: padded with extra cells on each side

  // simulation parameters
  bool xy_free_slip;
  bool yz_free_slip;
  bool zx_free_slip;
  bool compressible;
  double viscosity;
  double density; // average # of particles initialized in each "Full" cell

  MarchingCubes *marchingCubes;  // to display an isosurface 

  // VBOs
  GLuint smoke_particles_VBO;
  GLuint smoke_velocity_vis_VBO;
  GLuint smoke_face_velocity_vis_VBO;
  GLuint smoke_pressure_vis_VBO;
  GLuint smoke_cell_type_vis_VBO;
  std::vector<VBOPos> smoke_particles;
  std::vector<VBOPosColor> smoke_velocity_vis;
  std::vector<VBOPosNormalColor> smoke_face_velocity_vis;
  std::vector<VBOPosNormalColor> smoke_pressure_vis;
  std::vector<VBOPosNormalColor> smoke_cell_type_vis;



  //===============================================================================
  //Rendering

  // the quads from the .obj file that have non-zero emission value
   // HELPER FUNCTIONS
  static void Clear() {
    main_segments.clear();
    shadow_segments.clear();
    reflected_segments.clear();
    transmitted_segments.clear();
  }
  
  // REPRESENTATION
  static int activated;
  static std::vector<Segment> main_segments;
  static std::vector<Segment> shadow_segments;
  static std::vector<Segment> reflected_segments;
  static std::vector<Segment> transmitted_segments;

  // VBO
  static GLuint smoke_verts_VBO;
  static GLuint smoke_edge_indices_VBO;
  static std::vector<VBOPosColor4> smoke_verts; 
  static std::vector<VBOIndexedEdge> smoke_edge_indices;

  

};


void setupCubeVBO(const Vec3f pts[8], const Vec3f &color, std::vector<VBOPosNormalColor> &faces);
void setupConeVBO(const Vec3f pts[5], const Vec3f &color, std::vector<VBOPosNormalColor> &faces);

// ========================================================================

#endif