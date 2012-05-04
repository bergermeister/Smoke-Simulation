#include "octree.h"

#define EPSILON 0.00005
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

bool OCTree::ParticleInCell(const SmokeParticle * p) {
  const Vec3f& min = this->bbox->getMin();
  const Vec3f& max = this->bbox->getMax();
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
void OCTree::AddParticle(SmokeParticle * p) 
{
	const Vec3f &position = p->getPosition();
	if(!ParticleInCell(p)){
		if(p->getPosition().x() > bbox->getMax().x()) p->setPosition(Vec3f(bbox->getMax().x()-EPSILON, p->getPosition().y(), p->getPosition().z()));
		if(p->getPosition().y() > bbox->getMax().y()) p->setPosition(Vec3f(p->getPosition().x(), bbox->getMax().y()-EPSILON, p->getPosition().z()));
		if(p->getPosition().z() > bbox->getMax().z()) p->setPosition(Vec3f(p->getPosition().x(), p->getPosition().y(), bbox->getMax().z()-EPSILON));
	}
	//assert (ParticleInCell(p));
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
		if(child[0]->ParticleInCell(p)) child[0]->AddParticle(p);
		else if(child[1]->ParticleInCell(p)) child[1]->AddParticle(p);
		else if(child[2]->ParticleInCell(p)) child[2]->AddParticle(p);
		else if(child[3]->ParticleInCell(p)) child[3]->AddParticle(p);
		else if(child[4]->ParticleInCell(p)) child[4]->AddParticle(p);
		else if(child[5]->ParticleInCell(p)) child[5]->AddParticle(p);
		else if(child[6]->ParticleInCell(p)) child[6]->AddParticle(p);
		else if(child[7]->ParticleInCell(p)) child[7]->AddParticle(p);
		/*
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
				else child[7]->AddParticle(p);											// Else it is in front
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
				else child[6]->AddParticle(p);											// Else it is in front.
			}
		} 
		*/
	}
}

void OCTree::RemoveParticle(const SmokeParticle* p)
{
	assert(p != NULL);
	for (std::vector<SmokeParticle*>::iterator i = bbox->getParticles().begin(); i != bbox->getParticles().end(); i++) {
      if (*i == p) {
        bbox->getParticles().erase(i);
        return;
      }
    }
    assert (0); 
}

void OCTree::cleanupTree()
{
	std::vector<OCTree*> todo;  
	todo.push_back(this);
	while (!todo.empty()) 
	{
		OCTree *node = todo.back();
		todo.pop_back();
		std::vector<SmokeParticle*> particles;
		if(!node->isLeaf()){
			for(int i = 0; i < 8; i++)
			{
				std::vector<SmokeParticle*> childParticles = node->child[i]->getParticles();
				for(int j = 0; j < childParticles.size(); j++) particles.push_back(childParticles[j]);
			}
			if (node->isParentLeaf() && particles.size() <= MAX_PARTICLES_BEFORE_SPLIT) 
			{
				node->mergeChildren(particles);
			} 
			else //if (!node->isLeaf())
			{
			// if this cell is not a leaf, explore all children
			for(int i = 0; i < 8; i++) todo.push_back(node->getChild(i));
			}
		}
		
	}

}

void OCTree::mergeChildren(std::vector<SmokeParticle *> & particles)
{
	Vec3f new_velocities = Vec3f(bbox->get_u_plus(),bbox->get_v_plus(),bbox->get_w_plus());
	double new_pressure = 0.0;
	for(int i = 0; i < 8; i++) 
	{
		new_velocities += Vec3f(child[i]->getCell()->get_u_plus(), child[i]->getCell()->get_v_plus(), child[i]->getCell()->get_w_plus());
		new_pressure = child[i]->getCell()->getPressure();
		delete child[i]; 
		child[i] = NULL;
	}
	for(int i = 0; i < particles.size(); i++) this->AddParticle(particles[i]);
	bbox->set_u_plus(new_velocities.x());
	bbox->set_v_plus(new_velocities.y());
	bbox->set_w_plus(new_velocities.z());
	bbox->setPressure(new_pressure);
}

BoundingBox * OCTree::getCell(Vec3f v)
{
	const Vec3f &position = v;
	if (isLeaf()) 
	{
		// this cell is a leaf node
		return bbox;
	} 
	else 
	{
		// this cell is not a leaf node
		// decide which subnode to recurse into
		if (position.x() < split_center.x())									// If the position is to the left..
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[0]->getCell(v);			// If the position is behind
				else child[4]->getCell(v);											// Else it is in front
			}
			else																	// Else it is above
			{
				if(position.z() < split_center.z()) child[3]->getCell(v);			// If the position is behind
				else child[7] ->getCell(v);											// Else it is in front
			}
		} 
		else																	// Else it is to the right.
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[1]->getCell(v);			// If the position is behind
				else child[5]->getCell(v);											// Else it is in front
			}
			else																	// Else it is above.
			{
				if(position.z() < split_center.z()) child[2]->getCell(v);			// If the position is behind
				else child[6]->getCell(v);											// Else it is in front.
			}
		} 
	}
}

BoundingBox * OCTree::getCell(double x, double y, double z)
{
	const Vec3f &position = Vec3f(x,y,z);
	if (isLeaf()) 
	{
		// this cell is a leaf node
		return bbox;
	} 
	else 
	{
		// this cell is not a leaf node
		// decide which subnode to recurse into
		if (position.x() < split_center.x())									// If the position is to the left..
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[0]->getCell(x,y,z);			// If the position is behind
				else child[4]->getCell(x,y,z);											// Else it is in front
			}
			else																	// Else it is above
			{
				if(position.z() < split_center.z()) child[3]->getCell(x,y,z);			// If the position is behind
				else child[7] ->getCell(x,y,z);											// Else it is in front
			}
		} 
		else																	// Else it is to the right.
		{
			if(position.y() < split_center.y())										// If the position is below..
			{
				if(position.z() < split_center.z()) child[1]->getCell(x,y,z);			// If the position is behind
				else child[5]->getCell(x,y,z);											// Else it is in front
			}
			else																	// Else it is above.
			{
				if(position.z() < split_center.z()) child[2]->getCell(x,y,z);			// If the position is behind
				else child[6]->getCell(x,y,z);											// Else it is in front.
			}
		} 
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
	double dx = (max.x()-min.x() )*0.5;
	double dy = (max.y()-min.y() )*0.5;
	double dz = (max.z()-min.z() )*0.5;
	// split this cell around the center
	// Bottom 4 quadrants
	BoundingBox * b = new BoundingBox(min, c);
	child[0] = new OCTree(b, depth+1);
	child[0]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[0]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[0]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[0]->getCell()->setPressure(bbox->getPressure()/8);

	Vec3f min2 = Vec3f(min.x()+dx, min.y(), min.z());
	Vec3f max2 = Vec3f(c.x()+dx, c.y(), c.z());
	b = new BoundingBox(min2, max2);
	child[1] = new OCTree(b, depth+1);
	child[1]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[1]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[1]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[1]->getCell()->setPressure(bbox->getPressure()/8);

	min2 = Vec3f(min.x()+dx, min.y()+dy, min.z());
	max2 = Vec3f(c.x()+dx, c.y()+dy, c.z());
	b = new BoundingBox(min2, max2);
	child[2] = new OCTree(b, depth+1);
	child[2]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[2]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[2]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[2]->getCell()->setPressure(bbox->getPressure()/8);

	min2 = Vec3f(min.x(), min.y()+dy, min.z());
	max2 = Vec3f(c.x(), c.y()+dy, c.z());
	b = new BoundingBox(min2, max2);
	child[3] = new OCTree(b, depth+1);
	child[3]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[3]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[3]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[3]->getCell()->setPressure(bbox->getPressure()/8);

	// Top 4 quadrants
	min2 = Vec3f(min.x(), min.y(), min.z() + dz);
	max2 = Vec3f(c.x(), c.y(), c.z() + dz);
	b = new BoundingBox(min2, max2);
	child[4] = new OCTree(b, depth+1);
	child[4]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[4]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[4]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[4]->getCell()->setPressure(bbox->getPressure()/8);

	min2 = Vec3f(min.x()+dx, min.y(), min.z()+dz);
	max2 = Vec3f(c.x()+dx, c.y(), c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[5] = new OCTree(b, depth+1);
	child[5]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[5]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[5]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[5]->getCell()->setPressure(bbox->getPressure()/8);

	min2 = Vec3f(min.x()+dx, min.y()+dy, min.z()+dz);
	max2 = Vec3f(c.x()+dx, c.y()+dy, c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[6] = new OCTree(b, depth+1);
	child[6]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[6]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[6]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[6]->getCell()->setPressure(bbox->getPressure()/8);

	min2 = Vec3f(min.x(), min.y()+dy, min.z()+dz);
	max2 = Vec3f(c.x(), c.y()+dy, c.z()+dz);
	b = new BoundingBox(min2, max2);
	child[7] = new OCTree(b, depth+1);
	child[7]->getCell()->set_u_plus(bbox->get_u_plus()/8);
	child[7]->getCell()->set_v_plus(bbox->get_v_plus()/8);
	child[7]->getCell()->set_w_plus(bbox->get_w_plus()/8);
	child[7]->getCell()->setPressure(bbox->getPressure()/8);

	int num_particles = bbox->numParticles();
	std::vector<SmokeParticle*> tmp = bbox->getParticles();
	bbox->getParticles().clear();
	// add all the particless to one of those children
	for (int i = 0; i < num_particles; i++) 
	{
		SmokeParticle * p = tmp[i];
		this->AddParticle(p);
	}
}

// ==================================================================


void OCTree::calculateTransmittanceOfBB(Vec3f xlight,float c,Vec3f lightColor)
{
	float T = 0;
	Vec3f center;
	if(isLeaf())
	{
		bbox->getCenter(center);
		float distToLightCentroid = (xlight - center).Length();
		Vec3f lightIntensity = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
		
		T = exp(-c*abs((center-xlight).Length()) );        //transmitted light source
		bbox->setLi(T*lightIntensity);

		return;
	}
	else
	{
		bbox->getCenter(center);
		float distToLightCentroid = (xlight - center).Length();
		Vec3f lightIntensity = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
		
		T = exp(-c*abs((center-xlight).Length()) );        //transmitted light source
		bbox->setLi(T*lightIntensity);
		for(int i = 0; i < 8; i++)
		{
			child[i]->getCell()->getCenter(center);
			float distToLightCentroid = (xlight - center).Length();
			Vec3f lightIntensity = lightColor / (M_PI*distToLightCentroid*distToLightCentroid);
		
			T = exp(-c*abs((center-xlight).Length()) );        //transmitted light source
			child[i]->getCell()->setLi(T*lightIntensity);
			
			child[i]->calculateTransmittanceOfBB(xlight,c,lightColor);

		}
		return;
	}
}
