#include "CollisionResolver.h"
#include "Collisions.h"

using namespace PhysicsEngine;

// Add the required velocities and change the positions of the rigid bodies involved in the contacts in contactList
void CollisionResolver::resolveContacts(std::vector<Collision> *collisionList, real duration)
{
	// Early exit out if the objects are barely penetrating etc... to prevent vibrations
	if (!isValid())
	{
		return;
	}

	// Prepare the contacts for processing
	prepareContacts(collisionList, duration);

	// Resolve the interpenetration problems with the contacts.
	adjustPositions(collisionList, duration);

	// Resolve the velocity problems with the contacts.
	adjustVelocities(collisionList, duration);
}

void CollisionResolver::prepareContacts(std::vector<Collision> *collisionList, real duration)
{
	// Generate contact velocity and axis information.
	for (unsigned int collisionIndex = 0; collisionIndex < collisionList->size(); collisionIndex++)
	{
		// Calculate the internal contact data (inertia, basis, etc).
		(*collisionList)[collisionIndex].calculateInternals(duration);
	}
}

void CollisionResolver::adjustVelocities(std::vector<Collision> *collisionList, real duration)
{
	Vector3 velocityChange[2], rotationChange[2];
	Vector3 deltaVel;

	// iteratively handle impacts in order of severity.
	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		// Find contact with maximum magnitude of probable velocity change.
		real max = velocityEpsilon;
		unsigned index = collisionList->size();
		for (unsigned i = 0; i < collisionList->size(); i++)
		{
			if ((*collisionList)[i].desiredDeltaVelocity > max)
			{
				max = (*collisionList)[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == collisionList->size()) break;

		// Match the awake state at the contact
		(*collisionList)[index].matchAwakeState();

		// Do the resolution on the contact that came out top.
		(*collisionList)[index].applyVelocityChange(velocityChange, rotationChange);

		// With the change in velocity of the two bodies, the update of
		// contact velocities means that some of the relative closing
		// velocities need recomputing.
		for (unsigned i = 0; i < collisionList->size(); i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++)
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					RigidBody *firstRigidBody = NULL;
					if (b == 0)
					{
						firstRigidBody = (*collisionList)[i].firstObject;
					}
					else
					{
						firstRigidBody = (*collisionList)[i].secondObject;
					}

					if (firstRigidBody == NULL)
					{
						continue;
					}

					RigidBody *secondRigidBody = NULL;
					if (d == 0)
					{
						secondRigidBody = (*collisionList)[index].firstObject;
					}
					else
					{
						secondRigidBody = (*collisionList)[index].secondObject;
					}


					if (firstRigidBody == secondRigidBody)
					{
						deltaVel = velocityChange[d] +
							rotationChange[d].vectorProduct(
							(*collisionList)[i].relativeContactPosition[b]);

						// The sign of the change is negative if we're dealing
						// with the second body in a contact.
						(*collisionList)[i].contactVelocity +=
							(*collisionList)[i].contactToWorld.transformTranspose(deltaVel)
							* (b ? -1 : 1);
						(*collisionList)[i].calculateDesiredDeltaVelocity(duration);
					}
				}
			}
		}
		velocityIterationsUsed++;
	}
}

void CollisionResolver::adjustPositions(std::vector<Collision> *collisionList, real duration)
{
	unsigned i, index;
	Vector3 linearChange[2], angularChange[2];
	real max;
	Vector3 deltaPosition;

	// iteratively resolve interpenetrations in order of severity.
	positionIterationsUsed = 0;
	while (positionIterationsUsed < positionIterations)
	{
		// Find biggest penetration
		max = positionEpsilon;
		index = collisionList->size();
		for (i = 0; i < collisionList->size(); i++)
		{
			if ((*collisionList)[i].penetration > max)
			{
				max = (*collisionList)[i].penetration;
				index = i;
			}
		}
		if (index == collisionList->size()) break;

		// Match the awake state at the contact
		(*collisionList)[index].matchAwakeState();

		// Resolve the penetration.
		(*collisionList)[index].applyPositionChange(linearChange, angularChange, max);

		// Again this action may have changed the penetration of other
		// bodies, so we update contacts.
		for (i = 0; i < collisionList->size(); i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++)
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					RigidBody *firstBody = NULL;
					if (b == 0)
					{
						firstBody = (*collisionList)[i].firstObject;
					}
					else
					{
						firstBody = (*collisionList)[i].secondObject;
					}

					if (firstBody == NULL)
					{
						continue;
					}

					RigidBody *secondBody = NULL;
					if (d == 0)
					{
						secondBody = (*collisionList)[index].firstObject;
					}
					else
					{
						secondBody = (*collisionList)[index].secondObject;
					}

					if (firstBody == secondBody)
					{
						deltaPosition = linearChange[d] +
							angularChange[d].vectorProduct(
							(*collisionList)[i].relativeContactPosition[b]);

						// The sign of the change is positive if we're
						// dealing with the second body in a contact
						// and negative otherwise (because we're
						// subtracting the resolution)..
						(*collisionList)[i].penetration +=
							deltaPosition.scalarProduct((*collisionList)[i].contactNormal)
							* (b ? 1 : -1);
					}
				}
			}
		}
		positionIterationsUsed++;
	}
}

CollisionResolver::CollisionResolver(unsigned velocityIterations,
	unsigned positionIterations,
	real velocityEpsilon,
	real positionEpsilon)
{
	setIterations(velocityIterations, positionIterations);
	setEpsilon(velocityEpsilon, positionEpsilon);
}

void CollisionResolver::setIterations(unsigned velocityIterationsInput,	unsigned positionIterationsInput)
{
	velocityIterations = velocityIterationsInput;
	positionIterations = positionIterationsInput;
}

void CollisionResolver::setEpsilon(real velocityEpsilonInput, real positionEpsilonInput)
{
	velocityEpsilon = velocityEpsilonInput;
	positionEpsilon = positionEpsilonInput;
}