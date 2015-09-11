#ifndef PHYSICS_H
#define PHYSICS_H


namespace PhysicsEngine
{
	class PhysicsMain
	{
	public:


	private:
		// Draw the background of the scene
		void drawBackground();
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