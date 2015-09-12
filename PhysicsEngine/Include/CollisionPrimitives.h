#ifndef COLLISION_PRIMITIVES_H
#define COLLISION_PRIMITIVES_H

#include "MathDataTypes.h"
#include "RigidBody.h"
#include "Win32_GLAppUtil.h"

namespace PhysicsEngine
{
	class CollisionPrimitive
	{
	public:
		RigidBody *body;
		OVR::Model *model;
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