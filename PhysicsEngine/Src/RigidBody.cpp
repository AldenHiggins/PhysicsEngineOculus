#include <iostream>
//#include "gl/glut.h"
#include "RigidBody.h"

using namespace PhysicsEngine;
// Recalculate the given transformation matrix based on the given position and orientation (taken from Cyclone)
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix, const Vector3 &position, const Quaternion &orientation);
// Change the inertia tensor of the rigid body to be in world instead of object space (because our torques are going to be in world space)
static inline void _transformInertiaTensor(Matrix3 &iitWorld, const Quaternion &q, const Matrix3 &iitBody, const Matrix4 &rotmat);

// Integrate this object based on the time elapsed this frame
void RigidBody::integrate(real timeStep)
{
	// If this rigid body isn't awake skip the integration
	if (!isAwake)
	{
		return;
	}

	// Calculate linear acceleration from force inputs.
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

	// Calculate angular acceleration from torque inputs.
	Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

	// Adjust velocities
	// Update linear velocity from both acceleration and impulse.
	linearVelocity.addScaledVector(lastFrameAcceleration, timeStep);

	// Update angular velocity from both acceleration and impulse.
	angularVelocity.addScaledVector(angularAcceleration, timeStep);

	// Impose drag.
	linearVelocity *= real_pow(linearDamping, timeStep);
	angularVelocity *= real_pow(angularDamping, timeStep);

	// Adjust positions
	// Update linear position.
	position.addScaledVector(linearVelocity, timeStep);

	// Update angular position.
	orientation.addScaledVector(angularVelocity, timeStep);

	// Normalise the orientation, and update the matrices with the new
	// position and orientation
	calculateDerivedData();

	// Clear all forces and torques active on the body because we've already added them to the body's velocity
	clearAccumulators();

	// Update the kinetic energy store, and possibly put the body to
	// sleep.
	real currentMotion = linearVelocity.scalarProduct(linearVelocity) +	angularVelocity.scalarProduct(angularVelocity);
	real bias = real_pow(0.5, timeStep);
	motion = bias*motion + (1 - bias)*currentMotion;

	if (motion < sleepEpsilon)
	{
		setIsAwake(false);
	}
	else if (motion > 10 * sleepEpsilon)
	{
		motion = 10 * sleepEpsilon;
	}
}

// Add a force to this object's center of mass
void RigidBody::addForce(const Vector3 &force)
{
	forceAccum += force;
}

// Add a force at a position on this object in world space
void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += pt % force;
}

// Add a force at a position on this object in object space
void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
{
	// Convert to coordinates relative to center of mass.
	Vector3 pt = getPointInWorldSpace(point);
	addForceAtPoint(force, pt);
}

// Add the inputted velocity to this rigid body
void RigidBody::addVelocity(const Vector3 &deltaVelocity)
{
	linearVelocity += deltaVelocity;
}

// Add the inputted rotation to this rigid body's rotational velocity
void RigidBody::addRotation(const Vector3 &deltaRotation)
{
	angularVelocity += deltaRotation;
}

// Clear all forces/torques active on the object
void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum.clear();
}

// Set the inertia tensor of this rigid body
void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor)
{
	inverseInertiaTensor.setInverse(inertiaTensor);
}

// Get the inverse inertia tensor of this object in world coords
void RigidBody::getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = inverseInertiaTensorWorld;
}

// Set the mass of this object
void RigidBody::setMass(real objectMass)
{
	inverseMass = 1 / objectMass;
}

// Get the mass of this object
real RigidBody::getMass()
{
	return 1 / inverseMass;
}

// Get the inverse mass of this object
real RigidBody::getInverseMass()
{
	return inverseMass;
}

// Set the position of this object
void RigidBody::setPosition(Vector3 &positionInput)
{
	position = positionInput;
}

// Get the position of this object
Vector3 RigidBody::getPosition()
{
	return position;
}

// Set the orientation of this rigid body
void RigidBody::setOrientation(const Quaternion &orientationInput)
{
	orientation = orientationInput;
	orientation.normalise();
}

// Set the damping values for this rigid body
void RigidBody::setDamping(const real linearDampingInput, const real angularDampingInput)
{
	linearDamping = linearDampingInput;
	angularDamping = angularDampingInput;
}

// Get the orientation of this rigid body
Quaternion RigidBody::getOrientation()
{
	return orientation;
}

// Set the velocity of the rigid body
void RigidBody::setVelocity(const Vector3 &velocityInput)
{
	linearVelocity = velocityInput;
}

// Get the velocity of the rigid body
Vector3 RigidBody::getVelocity() const
{
	return linearVelocity;
}

// Set the acceleration of the rigid body
void RigidBody::setAcceleration(const Vector3 &accelerationInput)
{
	acceleration = accelerationInput;
}

// Get the acceleration of the rigid body
Vector3 RigidBody::getAcceleration() const
{
	return acceleration;
}

// Get the last frame's acceleration of this rigid body
Vector3 RigidBody::getLastFrameAcceleration() const
{
	return lastFrameAcceleration;
}

// Get the angular velocity of this rigid body
Vector3 RigidBody::getRotation() const
{
	return angularVelocity;
}

// Get whether or not this body is awake
bool RigidBody::getIsAwake() const
{
	return isAwake;
}

// Set this rigid body to be awake or asleep
void RigidBody::setIsAwake(bool isAwakeInput)
{
	if (isAwakeInput)
	{
		isAwake = true;

		// Add a bit of motion to avoid it falling asleep immediately.
		motion = sleepEpsilon*2.0f;
	}
	else 
	{
		isAwake = false;
		linearVelocity.clear();
		angularVelocity.clear();
	}
}

// Get this rigid bodies transformation matrix
Matrix4 RigidBody::getTransformMatrix() const
{
	return transformationMatrix;
}

// Transform a point from object space into world space
Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const
{
	return transformationMatrix.transform(point);
}

void RigidBody::getGLTransform(float matrix[16]) const
{
	matrix[0] = (float)transformationMatrix.data[0];
	matrix[1] = (float)transformationMatrix.data[4];
	matrix[2] = (float)transformationMatrix.data[8];
	matrix[3] = 0;

	matrix[4] = (float)transformationMatrix.data[1];
	matrix[5] = (float)transformationMatrix.data[5];
	matrix[6] = (float)transformationMatrix.data[9];
	matrix[7] = 0;

	matrix[8] = (float)transformationMatrix.data[2];
	matrix[9] = (float)transformationMatrix.data[6];
	matrix[10] = (float)transformationMatrix.data[10];
	matrix[11] = 0;

	matrix[12] = (float)transformationMatrix.data[3];
	matrix[13] = (float)transformationMatrix.data[7];
	matrix[14] = (float)transformationMatrix.data[11];
	matrix[15] = 1;
}

// Calculates all of the derived data that results from the current state of the rigidbody, this happens
// automatically during integration
void RigidBody::calculateDerivedData()
{
	orientation.normalise();

	// Calculate the transformation matrix of this body
	_calculateTransformMatrix(transformationMatrix, position, orientation);

	// Transform the inverse inertia tensor into world space
	_transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformationMatrix);
}

// Change the inertia tensor of the rigid body to be in world instead of object space (because our torques are going to be in world space)
static inline void _transformInertiaTensor(Matrix3 &iitWorld, const Quaternion &q, const Matrix3 &iitBody, const Matrix4 &rotmat)
{
	real t4 = rotmat.data[0] * iitBody.data[0] +
		rotmat.data[1] * iitBody.data[3] +
		rotmat.data[2] * iitBody.data[6];
	real t9 = rotmat.data[0] * iitBody.data[1] +
		rotmat.data[1] * iitBody.data[4] +
		rotmat.data[2] * iitBody.data[7];
	real t14 = rotmat.data[0] * iitBody.data[2] +
		rotmat.data[1] * iitBody.data[5] +
		rotmat.data[2] * iitBody.data[8];
	real t28 = rotmat.data[4] * iitBody.data[0] +
		rotmat.data[5] * iitBody.data[3] +
		rotmat.data[6] * iitBody.data[6];
	real t33 = rotmat.data[4] * iitBody.data[1] +
		rotmat.data[5] * iitBody.data[4] +
		rotmat.data[6] * iitBody.data[7];
	real t38 = rotmat.data[4] * iitBody.data[2] +
		rotmat.data[5] * iitBody.data[5] +
		rotmat.data[6] * iitBody.data[8];
	real t52 = rotmat.data[8] * iitBody.data[0] +
		rotmat.data[9] * iitBody.data[3] +
		rotmat.data[10] * iitBody.data[6];
	real t57 = rotmat.data[8] * iitBody.data[1] +
		rotmat.data[9] * iitBody.data[4] +
		rotmat.data[10] * iitBody.data[7];
	real t62 = rotmat.data[8] * iitBody.data[2] +
		rotmat.data[9] * iitBody.data[5] +
		rotmat.data[10] * iitBody.data[8];

	iitWorld.data[0] = t4*rotmat.data[0] +
		t9*rotmat.data[1] +
		t14*rotmat.data[2];
	iitWorld.data[1] = t4*rotmat.data[4] +
		t9*rotmat.data[5] +
		t14*rotmat.data[6];
	iitWorld.data[2] = t4*rotmat.data[8] +
		t9*rotmat.data[9] +
		t14*rotmat.data[10];
	iitWorld.data[3] = t28*rotmat.data[0] +
		t33*rotmat.data[1] +
		t38*rotmat.data[2];
	iitWorld.data[4] = t28*rotmat.data[4] +
		t33*rotmat.data[5] +
		t38*rotmat.data[6];
	iitWorld.data[5] = t28*rotmat.data[8] +
		t33*rotmat.data[9] +
		t38*rotmat.data[10];
	iitWorld.data[6] = t52*rotmat.data[0] +
		t57*rotmat.data[1] +
		t62*rotmat.data[2];
	iitWorld.data[7] = t52*rotmat.data[4] +
		t57*rotmat.data[5] +
		t62*rotmat.data[6];
	iitWorld.data[8] = t52*rotmat.data[8] +
		t57*rotmat.data[9] +
		t62*rotmat.data[10];
}

// Recalculate the given transformation matrix based on the given position and orientation (taken from Cyclone)
inline void _calculateTransformMatrix(Matrix4 &transformMatrix, const Vector3 &position, const Quaternion &orientation)
{
	transformMatrix.data[0] = 1 - 2 * orientation.j*orientation.j -
		2 * orientation.k*orientation.k;
	transformMatrix.data[1] = 2 * orientation.i*orientation.j -
		2 * orientation.r*orientation.k;
	transformMatrix.data[2] = 2 * orientation.i*orientation.k +
		2 * orientation.r*orientation.j;
	transformMatrix.data[3] = position.x;

	transformMatrix.data[4] = 2 * orientation.i*orientation.j +
		2 * orientation.r*orientation.k;
	transformMatrix.data[5] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.k*orientation.k;
	transformMatrix.data[6] = 2 * orientation.j*orientation.k -
		2 * orientation.r*orientation.i;
	transformMatrix.data[7] = position.y;

	transformMatrix.data[8] = 2 * orientation.i*orientation.k -
		2 * orientation.r*orientation.j;
	transformMatrix.data[9] = 2 * orientation.j*orientation.k +
		2 * orientation.r*orientation.i;
	transformMatrix.data[10] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.j*orientation.j;
	transformMatrix.data[11] = position.z;
}