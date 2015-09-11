#ifndef COLLISION_PRIMITIVES_H
#define COLLISION_PRIMITIVES_H

#include "MathDataTypes.h"
#include "RigidBody.h"

namespace PhysicsEngine
{
	class CollisionPrimitive
	{
	public:
		RigidBody *body;
		//Matrix4 transformOffset;
	};

	class CollisionBox : public CollisionPrimitive
	{
	public:
		Vector3 halfSize;
	};

	class CollisionSphere : public CollisionPrimitive
	{
	public:
		real radius;
	};
}

#endif // COLLISION_PRIMITIVES_H