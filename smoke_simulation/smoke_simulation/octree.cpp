#include "octree.h"

#define EPSILON 0.0001
#define MAX_DEPTH 10
#define MAX_PARTICLES_BEFORE_SPLIT 50

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
  const Vec3f& min = bbox->getMin();
  const Vec3f& max = bbox->getMax();
  const Vec3f &position = p->getPosition();
  if (position.x() > min.x() - EPSILON &&
      position.y() > min.y() - EPSILON &&
      position.z() > min.z() - EPSILON &&
      position.x() < max.x() + EPSILON &&
      position.y() < max.y() + EPSILON &&
      position.z() < max.z() + EPSILON)
    return true;
  return false;
}

bool OCTree::overlaps(const BoundingBox &bb) const {
  const Vec3f& bb_min = bb.getMin();
  const Vec3f& bb_max = bb.getMax();
  const Vec3f& tmp_min = bbox->getMin();
  const Vec3f& tmp_max = bbox->getMax();
  if (bb_min.x() > tmp_max.x()) return false;
  if (tmp_min.x() > bb_max.x()) return false;
  if (bb_min.y() > tmp_max.y()) return false;
  if (tmp_min.y() > bb_max.y()) return false;
  if (bb_min.z() > tmp_max.z()) return false;
  if (tmp_min.z() > bb_max.z()) return false;
  return true;
}


// ==================================================================
void OCTree::AddParticle(const SmokeParticle * p) 
{
	const Vec3f &position = p->getPosition();
	assert (ParticleInCell(p));
	if (isLeaf()) 
	{
		// this cell is a leaf node
		bbox->addParticle((SmokeParticle *)p);
		if (bbox->numParticles() > MAX_PARTICLES_BEFORE_SPLIT && depth < MAX_DEPTH)
		{
			SplitCell();
		}
	} 
	else 
	{
		// this cell is not a leaf node
		// decide which subnode to recurse into
		if (position.x() < split_center.x())									// If the position is to the left..
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[0]->AddParticle(p);			// If the position is behind
				else child[4]->AddParticle(p);											// Else it is in front
			}
			else																	// Else it is above
			{
				if(position.z() < split_center.z()) child[3]->AddParticle(p);			// If the position is behind
				else child[7] ->AddParticle(p);											// Else it is in front
			}
		} 
		else																	// Else it is to the right.
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[1]->AddParticle(p);			// If the position is behind
				else child[5]->AddParticle(p);											// Else it is in front
			}
			else																	// Else it is above.
			{
				if(position.z() < split_center.z()) child[2]->AddParticle(p);			// If the position is behind
				else child[6] ->AddParticle(p);											// Else it is in front.
			}
		} 
	}
}

BoundingBox * OCTree::getCell(double x, double y, double z)
{
	if(isLeaf()) return bbox;
	else
	{
		double dis = (Vec3f(x, y, z) - child[0]->getCenter()).Length();
		double temp;
		int j = 0;
		for(int i = 1; i < 8; i++)
		{
			temp = (Vec3f(x, y, z) - child[i]->getCenter()).Length();
			if(dis > temp) 
			{
				dis = temp;
				j = i;
			}
		}
		child[j]->getCell(x,y,z);
	}
}
// ==================================================================
void OCTree::CollectParticlesInBox(const BoundingBox &bb, std::vector<SmokeParticle*> &particles) 
{
	// explicitly store the queue of cells that must be checked (rather
	// than write a recursive function)
	std::vector<OCTree*> todo;  
	todo.push_back(this);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back(); 
		if (!node->overlaps(bb)) continue;
		if (node->isLeaf()) {
			// if this cell overlaps & is a leaf, add all of the photons into the master list
			// NOTE: these photons may not be inside of the query bounding box
			const std::vector<SmokeParticle *> particles2 = node->getParticles();
			int num_particles = particles2.size();
			for (int i = 0; i < num_particles; i++) 
			{
				particles.push_back(particles2[i]);
			}
		} 
		else 
		{
			// if this cell is not a leaf, explore both children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
		} 
	}
}


// ==================================================================
void OCTree::SplitCell() {
	const Vec3f& min = bbox->getMin();
	const Vec3f& max = bbox->getMax();
	const Vec3f c = split_center;
	double dx = (max.x()-min.x() )/2;
	double dy = (max.y()-min.y() )/2;
	double dz = (max.z()-min.z() )/2;
	// split this cell around the center
	// Bottom 4 quadrants
	BoundingBox * b = new BoundingBox(min, c);
	child[0] = new OCTree(b, depth+1);

	Vec3f min2 = Vec3f(min.x()+dx, min.y(), min.z());
	Vec3f max2 = Vec3f(c.x()+dx, c.y(), c.z());
	b = new BoundingBox(min2, max2);
	child[1] = new OCTree(b, depth+1);

	min2 = Vec3f(min.x()+dx, min.y()+dy, min.z());
	max2 = Vec3f(c.x()+dx, c.y()+dy, c.z());
	b = new BoundingBox(min2, max2);
	child[2] = new OCTree(b, depth+1);

	min2 = Vec3f(min.x(), min.y()+dy, min.z());
	max2 = Vec3f(c.x(), c.y()+dy, c.z());
	b = new BoundingBox(min2, max2);
	child[3] = new OCTree(b, depth+1);

	// Top 4 quadrants
	min2 = Vec3f(min.x(), min.y(), min.z() + dz);
	max2 = Vec3f(c.x(), c.y(), c.z() + dz);
	b = new BoundingBox(min2, max2);
	child[4] = new OCTree(b, depth+1);

	min2 = Vec3f(min.x()+dx, min.y(), min.z()+dz);
	max2 = Vec3f(c.x()+dx, c.y(), c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[5] = new OCTree(b, depth+1);

	min2 = Vec3f(min.x()+dx, min.y()+dy, min.z()+dz);
	max2 = Vec3f(c.x()+dx, c.y()+dy, c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[6] = new OCTree(b, depth+1);

	min2 = Vec3f(min.x(), min.y()+dy, min.z()+dz);
	max2 = Vec3f(c.x(), c.y()+dy, c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[7] = new OCTree(b, depth+1);

	int num_particles = bbox->numParticles();
	std::vector<SmokeParticle*> tmp = bbox->getParticles();
	bbox->getParticles().clear();
	// add all the particless to one of those children
	for (int i = 0; i < num_particles; i++) 
	{
		const SmokeParticle * p = tmp[i];
		this->AddParticle(p);
	}
}

// ==================================================================
