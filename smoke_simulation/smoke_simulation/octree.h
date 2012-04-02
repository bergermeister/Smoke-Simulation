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
  OCTree(const BoundingBox &_bbox, int _depth=0) {
    bbox = _bbox;
    depth = _depth;
    for(int i = 0; i < 8; i++){
		child[i] = NULL;
	}     
  }
  ~OCTree();

  // =========
  // ACCESSORS
  // boundingbox
  const Vec3f& getMin() const { return bbox.getMin(); }
  const Vec3f& getMax() const { return bbox.getMax(); }
  bool overlaps(const BoundingBox &bb) const;
  // hierarchy
  int getDepth() const { return depth; }
  bool isLeaf() const { 
	  for(int i = 0; i < 8; i++)
		if (child[i]!=NULL) return false;
    return true; }
  const OCTree* getChild(int i) const { assert (!isLeaf()); assert (child[i] != NULL); return child[i]; }
  
  // Smoke Particles
  const std::vector<SmokeParticle*>& getParticles() const { return particles; }
  void CollectParticlesInBox(const BoundingBox &bb, std::vector<SmokeParticle*> &particles) const;

  // =========
  // MODIFIERS
  void AddParticle(const SmokeParticle* &p);
  bool ParticleInCell(const SmokeParticle* &p);

 private:

  // HELPER FUNCTION
  void SplitCell();

  // REPRESENTATION
  BoundingBox bbox;
  OCTree* child[8];
  Vec3f split_center;
  double split_value;
  std::vector<SmokeParticle*> particles;
  int depth;
};

#endif
