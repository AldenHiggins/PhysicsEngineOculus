#include "Debug.h"
#include "BoundingVolumes.h"
//#include <gl/glut.h>
#include <iostream>

using namespace PhysicsEngine;

void drawSphere(BoundingSphere sphere)
{
	////std::cout << "Bounding sphere position: " << sphere.centre[0] << " " << sphere.centre[1] << " " << sphere.centre[2] << " Radius: " << sphere.radius << std::endl;
	//glColor4f(1.0f, 0.0f, 0.0f, 0.2f);
	//glPushMatrix();
	//Vector3 position = sphere.centre;
	//glTranslatef(position[0], position[1], position[2]);
	//glutSolidSphere(sphere.radius, 20.0, 20.0);
	//glPopMatrix();
}

void PhysicsEngine::drawBoundingVolumes(BVHNode<BoundingSphere> *boundingNodes)
{
	drawSphere(boundingNodes->volume);

	if (boundingNodes->children[0] != NULL)
	{
		drawBoundingVolumes(boundingNodes->children[0]);
	}

	if (boundingNodes->children[1] != NULL)
	{
		drawBoundingVolumes(boundingNodes->children[1]);
	}
}