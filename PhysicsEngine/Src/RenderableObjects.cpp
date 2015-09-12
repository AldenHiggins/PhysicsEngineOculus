#include "RenderableObjects.h"
//#include <gl/glut.h>

using namespace PhysicsEngine;

// Display this rectangle
void RectangleObject::display()
{
	//// Get the OpenGL transformation
	//GLfloat mat[16];
	//this->body->getGLTransform(mat);
	//glColor3f(.5f, .1f, .9f);

	//glPushMatrix();
	//glMultMatrixf(mat);
	//glScalef(halfSize[0] * 2, halfSize[1] * 2, halfSize[2] * 2);
	//glutSolidCube(1.0f);
	//glPopMatrix();
}

// Sync up this rectangle's model with the rigid body
void RectangleObject::syncModelWithRigidBody()
{
	Vector3 position = body->getPosition();
	model->Pos = OVR::Vector3f((float)position[0], (float)position[1], (float)position[2]);
	Quaternion rotation = body->getOrientation();
	model->Rot.x = (float)rotation.i;
	model->Rot.y = (float)rotation.j;
	model->Rot.z = (float)rotation.k;
	model->Rot.w = (float)rotation.r;
}

// Display this sphere
void SphereObject::display()
{
	// Get the OpenGL transformation
	//GLfloat mat[16];
	//this->body->getGLTransform(mat);
	//glColor3f(.7f, .5f, .1f);

	//glPushMatrix();
	//glMultMatrixf(mat);
	//glutSolidSphere(radius, SPHERE_SLICES, SPHERE_STACKS);
	//glPopMatrix();
}