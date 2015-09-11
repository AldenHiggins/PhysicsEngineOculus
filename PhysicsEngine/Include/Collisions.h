#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "RigidBody.h"
#include "RenderableObjects.h"
#include <vector>

namespace PhysicsEngine
{
	class Collision
	{
	public:
		// The first object taking part in this collision
		RigidBody *firstObject;
		// The second object in the collision
		RigidBody *secondObject;
		// The point where the collision occured
		Vector3 contactPoint;
		// The normal pointing outwards in the direction of the collision
		Vector3 contactNormal;
		// The amount the two objects are interpenetrating
		real penetration;
		// Reduces the amount of velocity resulting from a collision
		real friction;
		// Holds the required change in velocity for this contact to be resolved.
		real desiredDeltaVelocity;
		// Holds the world space position of the contact point relative to centre of each body.This is set when the calculateInternals function is run.
		Vector3 relativeContactPosition[2];
		// An orthonormal transform matrix that converts co-ordinates in the contact's frame of reference to world coordinates
		Matrix3 contactToWorld;
		// Holds the closing velocity at the point of contact. This is set when the calculateInternals function is run.
		Vector3 contactVelocity;

		
		// Calculate the internals of this collision
		void calculateInternals(real duration);
		Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);
		void calculateDesiredDeltaVelocity(real duration);
		// Wakes up sleeping objects involved in a collision
		void matchAwakeState();
		// Performs an inertia-weighted impulse based resolution of this contact alone.
		void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
		// Performs an inertia weighted penetration resolution of this contact alone.
		void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);
	private:
		// Generate the contact basis for this collision
		inline void calculateContactBasis();
		inline Vector3 calculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor);
		inline Vector3 calculateFrictionImpulse(Matrix3 * inverseInertiaTensor);
		// Set the number of iterations for every resolution stage
		void setIterations(unsigned velocityIterations,	unsigned positionIterations);
	};
}

#endif // COLLISIONS_H