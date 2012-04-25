#ifndef _OCTREE_H_
#define _OCTREE_H_

#include <cstdlib>
#include <vector>
#include "vectors.h"
#include "boundingbox.h"
#include "cell.h"

// ==================================================================
// A hierarchical spatial data structure to store particles.  This data
// struture allows for fast nearby neighbor queries for use in smoke
// particle simulation.

class OCTree {
 public:

  // ========================
  // CONSTRUCTOR & DESTRUCTOR
  OCTree(BoundingBox * &_bbox, int _depth) {
    bbox = _bbox;
    depth = _depth;
    for(int i = 0; i < 8; i++){
		child[i] = NULL;
	}
	split_center = (bbox->getMin() + bbox->getMax())*0.5;
  }
  OCTree() { assert(0); }
  ~OCTree();

  // =========
  // ACCESSORS
  // boundingbox
  const Vec3f& getMin() const { return bbox->getMin(); }
  const Vec3f& getMax() const { return bbox->getMax(); }
  const Vec3f& getCenter() const { return split_center; }
  bool overlaps(const BoundingBox &bb) const;
  // hierarchy
  int getDepth() const { return depth; }
  bool isLeaf() const { 
	  for(int i = 0; i < 8; i++)
		if (child[i]!=NULL) return false;
    return true; }
  bool isParentLeaf() const {
	  for(int i = 0; i < 8; i++) if (!child[i]->isLeaf()) return false;
	  return true;
  }
  OCTree* getChild(int i) const { assert (!isLeaf()); assert (child[i] != NULL); return child[i]; }
  
  // Smoke Particles
  BoundingBox *getCell() { return bbox; };
  BoundingBox *getCell(Vec3f v);
  BoundingBox *getCell(double x, double y, double z);
  //std::vector<SmokeParticle*>& getParticles() const { return bbox.getParticles(); }
  std::vector<SmokeParticle*> getParticles() { return bbox->getParticles(); }
  void CollectParticlesInBox(const BoundingBox &bb, std::vector<SmokeParticle*> &particles) ;

  // =========
  // MODIFIERS
  void AddParticle(const SmokeParticle* p);
  void RemoveParticle(const SmokeParticle* p);
  bool ParticleInCell(const SmokeParticle* p);
  void mergeChildren();
  void cleanupTree();

  void initializeVBOs() {bbox->initializeVBOs();}
  void setupVBOs() {bbox->setupVBOs();}
  void drawVBOs() {bbox->drawVBOs();}
 private:

  // HELPER FUNCTION
  void SplitCell();

  // REPRESENTATION
  BoundingBox *bbox;
  OCTree* child[8];
  Vec3f split_center;
  double split_value;
  int depth;
};

#endif
