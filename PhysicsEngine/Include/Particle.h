#ifndef PARTICLE_H
#define PARTICLE_H

#include "MathDataTypes.h"

// Position
// Size
// Velocity
// Color
// Shape?

namespace PhysicsEngine
{
	class Particle
	{
	protected:
		Vector3 position;
		Vector3 velocity;
		Vector3 acceleration;
		real size;
		Vector3 color;
		real lifeTime;
		bool isDead = false;
		real timeAliveSoFar;
	public:
		Particle()
		{
			timeAliveSoFar = 0;
		}
		// Integrate the particle forward in time by the timestep
		virtual void integrate(real timeStep);
		// Display this particle
		virtual void display();
		// Function called when this particle dies
		virtual void onDeath();
		// Getters and setters for particle data
		Vector3 getPosition();
		void setPosition(Vector3 newPosition);
		Vector3 getVelocity();
		void setVelocity(Vector3 newVelocity);
		Vector3 getAcceleration();
		void setAcceleration(Vector3 newAcceleration);
		real getSize();
		void setSize(real newSize);
		Vector3 getColor();
		void setColor(Vector3 newColor);
		real getLifeTime();
		void setLifeTime(real newLifeTime);
		bool getIsDead();

		
	};

	class CircleParticle : public Particle
	{
	public:
		// Display this particle
		virtual void display();
	};

	class FireworkParticle : public CircleParticle
	{
	private:
		Particle *deathParticles[5];
	public:
		// Display this particle
		virtual void display();
		// Call this when the particle dies
		virtual void onDeath();
		// Firework integrate that integrates all of its children
		virtual void integrate(real timeStep);
	};

	struct CreateParticle
	{
		// Generate a new firework
		static Particle *createFireWorkParticle(real speed, real size, Vector3 position, Vector3 color);
		// Generate a new particle
		static Particle* CreateParticle::createParticle(real speed, real size, Vector3 color);
		// Generate a new circular particle
		static Particle* createCircularParticle(real speed, real size, Vector3 color);
	private:
		// These are private to stop instances being created: use get().
		CreateParticle() {}
		CreateParticle(const CreateParticle &) {}
		CreateParticle& operator=(const CreateParticle &);
	};

	




}


#endif //PARTICLE_H