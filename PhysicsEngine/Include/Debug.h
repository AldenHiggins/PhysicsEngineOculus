#ifndef DEBUG_H
#define DEBUG_H

#include "BoundingVolumes.h"

namespace PhysicsEngine
{
	void drawBoundingVolumes(BVHNode<BoundingSphere> *boundingNodes);
}

#endif // DEBUG_H