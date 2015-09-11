#ifndef PHYSICS_ENGINE_CONTROLS_H
#define PHYSICS_ENGINE_CONTROLS_H

#include "Particle.h"
#include "RigidBody.h"
#include "RenderableObjects.h"
#include <vector>

namespace PhysicsEngine
{
	class Controls
	{
	public:
		// Check for keystrokes and take the appropriate action based on them
		static void keyCheck
		(
			unsigned char key,
			std::vector<Particle *> *particles,
			std::vector<RectangleObject *> *rigidBodies,
			std::vector<SphereObject *> *spheres,
			float theta,
			float phi
		);
	private:
		// Add force to the first cube
		static void addForceToCube(std::vector<RectangleObject *> *rectangularBodies);
		// Add a cube rigid body to the scene
		static void addRigidCubeWhereYouLook(std::vector<RectangleObject *> *rectangularBodies, float theta, float phi);
		// Add a sphere to the scene where you look
		static void addSphereWhereYouLook
		(
			std::vector<SphereObject *> *sphereBodies,
			float theta,
			float phi,
			float radius
		);
		// Add a sphere to the scene with the given properties
		static void addSphere
		(
			std::vector<SphereObject *> *sphereBodies,
			Vector3 position,
			Vector3 velocity,
			real mass,
			real radius
		);
		// Add a rigid cube with the inputted parameters
		static void addRigidCube
		(
			std::vector<RectangleObject *> *rectangularBodies,
			Vector3 position,
			Vector3 velocity,
			real mass,
			Vector3 halfSize
		);
		static void rectangleKeyCheck
		(
			unsigned char key,
			std::vector<RectangleObject *> *rectangularBodies,
			float theta,
			float phi
		);
		static void sphereKeyCheck
		(
			unsigned char key,
			std::vector<SphereObject *> *sphereBodies,
			float theta,
			float phi
		);
		static void particleKeyCheck(unsigned char key, std::vector<Particle *> *particles);

		static Vector3 rotatePositionAlongYAxis(real depth, real height, real theta);
	};
	
}

#endif // PHYSICS_ENGINE_CONTROLS_H