#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "MathDataTypes.h"

namespace PhysicsEngine
{
	class RigidBody
	{
	protected:
		// Basic object properties
		// Hold the inverse mass because it is more useful in calculations and approaches infinity rather than zero
		real inverseMass;
		Quaternion orientation;
		Vector3 position;
		// Linear movement values
		Vector3 linearVelocity;
		Vector3 acceleration;
		Vector3 lastFrameAcceleration;
		// Angular movemement values
		Vector3 angularVelocity;
		// Store the inverse of the inertia tensor because that's the only form used in our calculations
		// and the tensor is a constant
		Matrix3 inverseInertiaTensor;
		// Store the inertia tensor in world coordinates to be used in conjunction with torque (also in world coords)
		Matrix3 inverseInertiaTensorWorld;

		// Holds the amount of motion of the body
		real motion;

		// Damping applied to linear motion of the rigid body to remove excess energy added by the integrator
		real linearDamping;

		// Damping applied to angular motion of the rigid body to remove excess energy added by the integrator
		real angularDamping;

		// Transformation matrix for this object
		Matrix4 transformationMatrix;

		// All the accumulated forces on this object's center of mass this frame
		Vector3 forceAccum;

		// All the accumulated torque on this object this frame
		Vector3 torqueAccum;

		// Denotes whether the rigid body is awake or moving/colliding/etc.. to prevent it from being updated unnecessarily
		bool isAwake;
	public:
		RigidBody()
		{
			motion = sleepEpsilon * 2.0f;
		}

		// Integrate this object based on the time elapsed this frame
		void integrate(real timeStep);

		// Add a force to this object's center of mass
		void addForce(const Vector3 &force);

		// Add a force at a position on this object in world space
		void addForceAtPoint(const Vector3 &force, const Vector3 &point);

		// Add a force at a position on this object in object space
		void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);

		// Add the inputted velocity to this rigid body
		void addVelocity(const Vector3 &deltaVelocity);

		// Add the inputted rotation to this rigid body's rotational velocity
		void addRotation(const Vector3 &deltaRotation);

		// Clear all forces/torques active on the object
		void clearAccumulators();

		// Calculates all of the derived data that results from the current state of the rigidbody, this happens
		// automatically during integration
		void calculateDerivedData();

		// Set the inertia tensor of this rigid body
		void setInertiaTensor(const Matrix3 &inertiaTensor);

		// Get the inverse inertia tensor of this object in world coords
		void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;

		// Set the mass of this object
		void setMass(real objectMass);

		// Get the mass of this object
		real getMass();

		// Get the inverse mass of this object
		real getInverseMass();

		// Set the position of this object
		void setPosition(Vector3 &positionInput);

		// Get the position of this object
		Vector3 getPosition();

		// Set the orientation of this rigid body
		void setOrientation(const Quaternion &orientationInput);

		// Get the orientation of this rigid body
		Quaternion getOrientation();

		// Set the velocity of the rigid body
		void setVelocity(const Vector3 &velocityInput);

		// Get the velocity of the rigid body
		Vector3 getVelocity() const;

		// Set the acceleration of the rigid body
		void setAcceleration(const Vector3 &accelerationInput);

		// Get the acceleration of the rigid body
		Vector3 getAcceleration() const;

		// Get the last frame's acceleration of this rigid body
		Vector3 getLastFrameAcceleration() const;

		// Get the angular velocity of this rigid body
		Vector3 getRotation() const;

		// Set the damping values for this rigid body
		void setDamping(const real linearDampingInput, const real angularDampingInput);

		// Get whether or not this body is awake
		bool getIsAwake() const;

		// Set this rigid body to be awake or asleep
		void setIsAwake(bool isAwakeInput);

		// Get this body's transformation matrix in a form that opengl can use
		void getGLTransform(float matrix[16]) const;

		// Transform a point from object space into world space
		Vector3 getPointInWorldSpace(const Vector3 &point) const;

		// Get this rigid bodies transformation matrix
		Matrix4 getTransformMatrix() const;
	};
}


#endif // RIGID_BODY_H