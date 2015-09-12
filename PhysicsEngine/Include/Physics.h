#ifndef PHYSICS_H
#define PHYSICS_H

#include "MathDataTypes.h"
#include "Timing.h"
#include "Collisions.h"
#include "Particle.h"
#include <vector>
#include "Win32_GLAppUtil.h"

namespace PhysicsEngine
{
	class PhysicsMain
	{
	public:
		// Update the physics engine for this frame
		void updatePhysics();

		void initializePhysics(OVR::Scene *scene);

		// Add a new cube to the scene
		void addCube(OVR::Scene *scene, OVR::Vector3f startPosition, OVR::Vector3f startHalfsize);

	private:
		// Store the materials generated in the example code for later use
		OVR::ShaderFill * grid_material[4];

		// Integrate all of the rigid bodies
		void integrateRigidBodies(real duration);
		// Detect collisions
		void detectCollisions(std::vector<Collision> *collisionList);
		// Resolve the found collisions
		void resolveCollisions(std::vector<Collision> *collisionList, real duration);

		// Contains all of the particles in the scene
		std::vector<Particle *> particles;
		// Contains all the rectangular objects in the scene
		std::vector<RectangleObject *> rectangleObjects;
		// Contains all of the spherical objects in the scene
		std::vector<SphereObject *> sphereObjects;
	};


}



#endif // PHYSICS_H