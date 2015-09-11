#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <vector>
#include "Collisions.h"

namespace PhysicsEngine
{
	class CollisionDetection
	{
	public:
		// Determine if there is a collision between two cubic rigid bodies
		static unsigned int cubeCubeCollisionDetect
		(
			std::vector<Collision> *collisionList,
			RectangleObject *first,
			RectangleObject *other
		);
		// Find collisions between a cube and a plane
		static unsigned boxAndHalfSpaceCollisionDetect
		(
			RectangleObject *box,
			const Vector3 planeDirection,
			real planeOffset,
			std::vector<Collision> *collisionList
		);

		// Determine if there is a collision between a sphere and a plane
		static unsigned sphereAndHalfSpaceCollisionDetect
		(
			SphereObject *sphere,
			const Vector3 planeDirection,
			real planeOffset,
			std::vector<Collision> *collisionList
		);

		// Determine if two spheres have collided
		static unsigned sphereSphereCollisionDetect
		(
			SphereObject *first,
			SphereObject *other,
			std::vector<Collision> *collisionList
		);

		// Determine if a sphere has collided with a cube
		static unsigned sphereCubeCollisionDetect
		(
			SphereObject *sphere,
			RectangleObject *cube,
			std::vector<Collision> *collisionList
		);
		
	private:
		// Perform an interesection test between the given box and plane
		static bool boxAndHalfSpaceIntersect
		(
			const RectangleObject *box,
			Vector3 planeDirection,
			real planeOffset
		);
		// Return the length of this box along the given axis
		static inline real transformToAxis
		(
			const RectangleObject *box,
			const Vector3 &axis
		);

		// Find the amount of penetration between the two objects on the given axis
		static inline real penetrationOnAxis
		(
			const RectangleObject *one,
			const RectangleObject *two,
			const Vector3 &axis,
			const Vector3 &toCentre
		);

		static inline bool CollisionDetection::tryAxis
		(
			const RectangleObject *one,
			const RectangleObject *two,
			Vector3 axis,
			const Vector3& toCentre,
			unsigned index,

			// These values may be updated
			real& smallestPenetration,
			unsigned &smallestCase
		);

		static void fillPointFaceBoxBox
		(
			RectangleObject *one,
			RectangleObject *two,
			const Vector3 &toCentre,
			std::vector<Collision> *data,
			unsigned best,
			real pen
		);

		static inline Vector3 contactPointCalculate
		(
			const Vector3 &pOne,
			const Vector3 &dOne,
			real oneSize,
			const Vector3 &pTwo,
			const Vector3 &dTwo,
			real twoSize,

			// If this is true, and the contact point is outside
			// the edge (in the case of an edge-face contact) then
			// we use one's midpoint, otherwise we use two's.
			bool useOne
		);
	};
}


#endif // COLLISION_DETECTION_H