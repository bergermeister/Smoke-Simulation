#include "octree.h"

// ==================================================================
// DESTRUCTOR
// ==================================================================
OCTree::~OCTree() {
  if (!isLeaf()) {
    for(int i = 0; i < 8; i++) delete child[i];
  } else {
    // delete all the particles (this is done automatically since they
    // are stored directly in an STL vector, not using pointers)
  }
}


// ==================================================================
// HELPER FUNCTIONS

bool OCTree::ParticleInCell(const SmokeParticle * &p) {
  const Vec3f& min = bbox.getMin();
  const Vec3f& max = bbox.getMax();
  const Vec3f &position = p.getPosition();
  if (position.x() > min.x() - EPSILON &&
      position.y() > min.y() - EPSILON &&
      position.z() > min.z() - EPSILON &&
      position.x() < max.x() + EPSILON &&
      position.y() < max.y() + EPSILON &&
      position.z() < max.z() + EPSILON)
    return true;
  return false;
}

bool KDTree::overlaps(const BoundingBox &bb) const {
  const Vec3f& bb_min = bb.getMin();
  const Vec3f& bb_max = bb.getMax();
  const Vec3f& tmp_min = bbox.getMin();
  const Vec3f& tmp_max = bbox.getMax();
  if (bb_min.x() > tmp_max.x()) return false;
  if (tmp_min.x() > bb_max.x()) return false;
  if (bb_min.y() > tmp_max.y()) return false;
  if (tmp_min.y() > bb_max.y()) return false;
  if (bb_min.z() > tmp_max.z()) return false;
  if (tmp_min.z() > bb_max.z()) return false;
  return true;
}


// ==================================================================
void KDTree::AddPhoton(const Photon &p) {
  const Vec3f &position = p.getPosition();
  assert (PhotonInCell(p));
  if (isLeaf()) {
    // this cell is a leaf node
    photons.push_back(p);
    if (photons.size() > MAX_PHOTONS_BEFORE_SPLIT && depth < MAX_DEPTH) {
      SplitCell();
    }
  } else {
    // this cell is not a leaf node
    // decide which subnode to recurse into
    if (split_axis == 0) {
      if (position.x() < split_value)
	child1->AddPhoton(p);
      else
	child2->AddPhoton(p);
    } else if (split_axis == 1) {
      if (position.y() < split_value)
	child1->AddPhoton(p);
      else
	child2->AddPhoton(p);
    } else {
      assert (split_axis == 2);
      if (position.z() < split_value)
	child1->AddPhoton(p);
      else
	child2->AddPhoton(p);
    }
  }
}


// ==================================================================
void KDTree::CollectPhotonsInBox(const BoundingBox &bb, std::vector<Photon> &photons) const {
  // explicitly store the queue of cells that must be checked (rather
  // than write a recursive function)
  std::vector<const KDTree*> todo;  
  todo.push_back(this);
  while (!todo.empty()) {
    const KDTree *node = todo.back();
    todo.pop_back(); 
    if (!node->overlaps(bb)) continue;
    if (node->isLeaf()) {
      // if this cell overlaps & is a leaf, add all of the photons into the master list
      // NOTE: these photons may not be inside of the query bounding box
      const std::vector<Photon> &photons2 = node->getPhotons();
      int num_photons = photons2.size();
      for (int i = 0; i < num_photons; i++) {
	photons.push_back(photons2[i]);
      }
    } else {
      // if this cell is not a leaf, explore both children
      todo.push_back(node->getChild1());
      todo.push_back(node->getChild2());
    } 
  }
}


// ==================================================================
void OCTree::SplitCell() {
  const Vec3f& min = bbox.getMin();
  const Vec3f& max = bbox.getMax();
  const Vec3f c = split_center;
  double dx = (max.x()-min.x() )/2;
  double dy = (max.y()-min.y() )/2;
  double dz = (max.z()-min.z() )/2;
  // split this cell around the center
  // Bottom 4 quadrants
  child[0] = new OCTree(BoundingBox(min, c), depth+1);

  Vec3f min2 = Vec3f(min.x()+dx, min.y(), min.z());
  Vec3f max2 = Vec3f(c.x()+dx, c.y(), c.z());
  child[1] = new OCTree(BoundingBox(min2, max2), depth+1);

  min2 = Vec3f(min.x()+dx, min.y()+dy, min.z());
  max2 = Vec3f(c.x()+dx, c.y()+dy, c.z());
  child[2] = new OCTree(BoundingBox(min2, max2), depth+1);

  min2 = Vec3f(min.x(), min.y()+dy, min.z());
  max2 = Vec3f(c.x(), c.y()+dy, c.z());
  child[3] = new OCTree(BoundingBox(min2, max2), depth+1);

  // Top 4 quadrants
  min2 = Vec3f(min.x(), min.y(), min.z() + dz);
  max2 = Vec3f(c.x(), c.y(), c.z() + dz);
  child[4] = new OCTree(BoundingBox(min2, max2), depth+1);

  Vec3f min2 = Vec3f(min.x()+dx, min.y(), min.z()+dz);
  Vec3f max2 = Vec3f(c.x()+dx, c.y(), c.z()+dz);
  child[5] = new OCTree(BoundingBox(min2, max2), depth+1);

  min2 = Vec3f(min.x()+dx, min.y()+dy, min.z()+dz);
  max2 = Vec3f(c.x()+dx, c.y()+dy, c.z()+dz);
  child[6] = new OCTree(BoundingBox(min2, max2), depth+1);

  min2 = Vec3f(min.x(), min.y()+dy, min.z()+dz);
  max2 = Vec3f(c.x(), c.y()+dy, c.z()+dz);
  child[7] = new OCTree(BoundingBox(min2, max2), depth+1);

  int num_particles = particles.size();
  std::vector<SmokeParticle *> tmp = particles;
  particles.clear();
  // add all the photons to one of those children
  for (int i = 0; i < num_particles; i++) {
    const SmokeParticle * &p = tmp[i];
    this->AddParticle(p);
  }
}

// ==================================================================
