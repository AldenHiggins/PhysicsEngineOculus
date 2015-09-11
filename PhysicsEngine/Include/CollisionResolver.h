#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include <vector>
#include "Collisions.h"

namespace PhysicsEngine
{
	class CollisionResolver
	{
	public:
		/**
		* Creates a new contact resolver with the given number of iterations
		* for each kind of resolution, and optional epsilon values.
		*/
		CollisionResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);

		/**
		* Stores the number of velocity iterations used in the
		* last call to resolve contacts.
		*/
		unsigned velocityIterationsUsed;

		/**
		* Stores the number of position iterations used in the
		* last call to resolve contacts.
		*/
		unsigned positionIterationsUsed;

		// Add the required velocities and change the positions of the rigid bodies involved in the contacts in contactList
		void resolveContacts(std::vector<Collision> *collisionList, real duration);
	protected:
		/**
		* Holds the number of iterations to perform when resolving
		* velocity.
		*/
		unsigned velocityIterations;

		/**
		* Holds the number of iterations to perform when resolving
		* position.
		*/
		unsigned positionIterations;

		// Velocities lower than this value are considered to be zero to prevent vibrating and instability
		real velocityEpsilon;

		// Positional interpenetration lower than this value is considered not to be penetration to avoid vibrating and instability
		real positionEpsilon;

	private:
		/**
		* Keeps track of whether the internal settings are valid.
		*/
		bool validSettings;

		void prepareContacts(std::vector<Collision> *collisionList, real duration);
		void adjustVelocities(std::vector<Collision> *collisionList, real duration);
		void adjustPositions(std::vector<Collision> *collisionList, real duration);

		/**
		* Sets the number of iterations for each resolution stage.
		*/
		void setIterations(unsigned velocityIterations, unsigned positionIterations);

		void setEpsilon(real velocityEpsilon, real positionEpsilon);

		/**
		* Returns true if the resolver has valid settings and is ready to go.
		*/
		bool isValid()
		{
			return (velocityIterations > 0) &&
				(positionIterations > 0) &&
				(positionEpsilon >= 0.0f) &&
				(positionEpsilon >= 0.0f);
		}
	};

}


#endif // COLLISION_RESOLVER_H