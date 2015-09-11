//#include <GL/glut.h>
#include "Physics.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "MathDataTypes.h"
#include "ApplicationSettings.h"
#include "Particle.h"
#include "Timing.h"
#include "RigidBody.h"
#include "Controls.h"
#include "BoundingVolumes.h"
#include "Debug.h"
#include "Collisions.h"
#include "CollisionResolver.h"
#include "CollisionDetection.h"
#include "RenderableObjects.h"

using namespace PhysicsEngine;

//// Draw the background of the scene
//void drawBackground();
//// Integrate all of the rigid bodies
//void integrateRigidBodies(real duration);
//// Detect collisions
//void detectCollisions(std::vector<Collision> *collisionList);
//// Resolve the found collisions
//void resolveCollisions(std::vector<Collision> *collisionList, real duration);

// Contains all of the particles in the scene
std::vector<Particle *> particles;
// Contains all the rectangular objects in the scene
std::vector<RectangleObject *> rectangleObjects;
// Contains all of the spherical objects in the scene
std::vector<SphereObject *> sphereObjects;

// Camera control variables
float theta;
float phi;
int lastX;
int lastY;

/**
* Creates a window in which to display the scene.
*/
void createWindow(const char* title)
{
	//glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	//glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
	//glutInitWindowPosition(0, 0);
	//glutCreateWindow(title);
}

/**
* Called each frame to update the 3D scene. Delegates to
* the application.
*/
void update()
{
	TimingData::get().update();
	//glutPostRedisplay();
}

/**
* Called each frame to display the 3D scene. Delegates to
* the application.
*/
void display()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//// Light up the scene
	//GLfloat light_position[] = { -4.0, 50.0, -10.0, 0.0 };
	//glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	//glLoadIdentity();
	//// Look out towards the Z direction (eye, center, up)
	//gluLookAt(0.0, 4.0, 0.0, 0.0, 4.0, 6.0, 0.0, 1.0, 0.0);
	//// Rotate the camera based on mouse movements
	//glRotatef(-phi, 1, 0, 0);
	//glRotatef(theta, 0, 1, 0);

	// Draw the background
	drawBackground();

	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
	if (duration <= 0.0f) return;
	else if (duration > 0.05f) duration = 0.05f;

	// Integrate all of the rigid bodies
	integrateRigidBodies(duration);

	// Check for and resolve collisions
	std::vector<Collision> collisionList;
	detectCollisions(&collisionList);
	resolveCollisions(&collisionList, duration);
	
	// Draw all of the particles
	for (unsigned int particleIndex = 0; particleIndex < particles.size(); particleIndex++)
	{
		particles[particleIndex]->display();
	}
	
	// Draw all cubes
	for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < rectangleObjects.size(); rigidBodyIndex++)
	{
		rectangleObjects[rigidBodyIndex]->display();
	}

	// Draw all spheres
	for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < sphereObjects.size(); rigidBodyIndex++)
	{
		sphereObjects[rigidBodyIndex]->display();
	}

	//glFlush();
	//glutSwapBuffers();
}

// Resolve the found collisions
void resolveCollisions(std::vector<Collision> *collisionList, real duration)
{
	if (collisionList->size() > 0)
	{
		CollisionResolver resolver(collisionList->size() * 4, collisionList->size() * 4);
		resolver.resolveContacts(collisionList, duration);
	}
}

// Detect collisions
void detectCollisions(std::vector<Collision> *collisionList)
{
	// Detect cube collisions
	for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < rectangleObjects.size(); rigidBodyIndex++)
	{
		// Check for collisions against the ground
		CollisionDetection::boxAndHalfSpaceCollisionDetect(rectangleObjects[rigidBodyIndex], Vector3(0, 1, 0), 0, collisionList);
		// Check for collisions against the walls
		CollisionDetection::boxAndHalfSpaceCollisionDetect(rectangleObjects[rigidBodyIndex], Vector3(-1, 0, 0), -20, collisionList);
		CollisionDetection::boxAndHalfSpaceCollisionDetect(rectangleObjects[rigidBodyIndex], Vector3(1, 0, 0), -20, collisionList);
		CollisionDetection::boxAndHalfSpaceCollisionDetect(rectangleObjects[rigidBodyIndex], Vector3(0, 0, -1), -20, collisionList);
		CollisionDetection::boxAndHalfSpaceCollisionDetect(rectangleObjects[rigidBodyIndex], Vector3(0, 0, 1), -20, collisionList);

		// Search for box/box collisions
		for (unsigned int otherRigidBodyIndex = rigidBodyIndex + 1; otherRigidBodyIndex < rectangleObjects.size(); otherRigidBodyIndex++)
		{
			CollisionDetection::cubeCubeCollisionDetect(collisionList, rectangleObjects[rigidBodyIndex], rectangleObjects[otherRigidBodyIndex]);
		}
	}

	// Detect sphere collisions
	for (unsigned int sphereIndex = 0; sphereIndex < sphereObjects.size(); sphereIndex++)
	{
		// Check for collisions against ground
		CollisionDetection::sphereAndHalfSpaceCollisionDetect(sphereObjects[sphereIndex], Vector3(0, 1, 0), 0, collisionList);
		// Check for collisions against the walls
		CollisionDetection::sphereAndHalfSpaceCollisionDetect(sphereObjects[sphereIndex], Vector3(-1, 0, 0), -20, collisionList);
		CollisionDetection::sphereAndHalfSpaceCollisionDetect(sphereObjects[sphereIndex], Vector3(1, 0, 0), -20, collisionList);
		CollisionDetection::sphereAndHalfSpaceCollisionDetect(sphereObjects[sphereIndex], Vector3(0, 0, -1), -20, collisionList);
		CollisionDetection::sphereAndHalfSpaceCollisionDetect(sphereObjects[sphereIndex], Vector3(0, 0, 1), -20, collisionList);

		// Check for collisions against other spheres
		for (unsigned int otherSphereIndex = sphereIndex + 1; otherSphereIndex < sphereObjects.size(); otherSphereIndex++)
		{
			CollisionDetection::sphereSphereCollisionDetect(sphereObjects[sphereIndex], sphereObjects[otherSphereIndex], collisionList);
		}

		// Check for collisions against cubes
		for (unsigned int cubeIndex = 0; cubeIndex < rectangleObjects.size(); cubeIndex++)
		{
			CollisionDetection::sphereCubeCollisionDetect(sphereObjects[sphereIndex], rectangleObjects[cubeIndex], collisionList);
		}
	}
}

// Integrate all of the rigid bodies in the scene
void integrateRigidBodies(real duration)
{
	// Integrate all of the particles
	for (unsigned int particleIndex = 0; particleIndex < particles.size(); particleIndex++)
	{
		particles[particleIndex]->integrate(duration);
	}

	// Integrate all of the cubes
	if (rectangleObjects.size() > 0)
	{
		// Build up a list of bounding spheres for broad phase collision detection
		//BVHNode<BoundingSphere> *parent = NULL;
		//BoundingSphere sphere(rigidBodies[0]->getPosition(), 1.0f);
		//BVHNode<BoundingSphere> newNode(parent, sphere);
		//newNode.body = rigidBodies[0];

		// Integrate all of the rigid bodies and add them to the bounding sphere heirarchy
		for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < rectangleObjects.size(); rigidBodyIndex++)
		{
			RigidBody *body = rectangleObjects[rigidBodyIndex]->body;
			body->integrate(duration);
			//BoundingSphere boundSphere(body->getPosition(), 1.0f);
			//if (rigidBodyIndex != 0)
			//{
			//	newNode.insert(body, boundSphere);
			//}
		}

		//if (rigidBodies.size() > 1)
		//{
		// Draw all of the bounding volumes
		//std::cout << "Drawing bounding volumes!!" << std::endl;
		//drawBoundingVolumes(&newNode);
		// Once all of the rigid bodies have been included check for collisions
		//PotentialContact contacts[MAX_CONTACTS_PER_FRAME];
		//int contactsFound = (*(newNode.children[0])).getPotentialContacts(contacts, MAX_CONTACTS_PER_FRAME);

		//PotentialContact contactsTwo[MAX_CONTACTS_PER_FRAME];
		//int secondContactsFound = (*(newNode.children[1])).getPotentialContacts(contactsTwo, MAX_CONTACTS_PER_FRAME);
		//}
	}

	// Integrate all of the spheres
	if (sphereObjects.size() > 0)
	{
		for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < sphereObjects.size(); rigidBodyIndex++)
		{
			sphereObjects[rigidBodyIndex]->body->integrate(duration);
		}
	}
}

// Draw the background of the scene
void drawBackground()
{
	//// Draw the ground
	//glBegin(GL_QUADS);
	//glColor3f(1.0f, 1.0f, 1.0f);
	//glNormal3f(0, 1, 0);
	//glVertex3f(20, 0, 20);
	//glVertex3f(20, 0, -20);
	//glVertex3f(-20, 0, -20);
	//glVertex3f(-20, 0, 20);
	//// Draw the walls
	//// Front wall
	//glColor3f(0.9f, 0.9f, 0.9f);
	//glNormal3f(0, 0, -1);
	//glVertex3f(20, 0, 20);
	//glVertex3f(20, 20, 20);
	//glVertex3f(-20, 20, 20);
	//glVertex3f(-20, 0, 20);
	//// Back Wall
	//glColor3f(0.8f, 0.8f, 0.8f);
	//glNormal3f(0, 0, 1);
	//glVertex3f(20, 0, -20);
	//glVertex3f(20, 20, -20);
	//glVertex3f(-20, 20, -20);
	//glVertex3f(-20, 0, -20);
	//// Left Wall
	//glColor3f(0.7f, 0.7f, 0.7f);
	//glNormal3f(-1, 0, 0);
	//glVertex3f(20, 0, -20);
	//glVertex3f(20, 20, -20);
	//glVertex3f(20, 20, 20);
	//glVertex3f(20, 0, 20);
	//// Right Wall
	//glColor3f(0.6f, 0.6f, 0.6f);
	//glNormal3f(1, 0, 0);
	//glVertex3f(-20, 0, 20);
	//glVertex3f(-20, 20, 20);
	//glVertex3f(-20, 20, -20);
	//glVertex3f(-20, 0, -20);
	//// Draw the lines of the axes
	//glColor3f(0.0f, 0.0f, 0.0f);
	//// x axis
	//glVertex3f(20, .01, AXES_WIDTH/2);
	//glVertex3f(20, .01, -1 * AXES_WIDTH/2);
	//glVertex3f(-20, .01, -1 * AXES_WIDTH/2);
	//glVertex3f(-20, .01, AXES_WIDTH/2);
	//// z axis
	//glVertex3f(AXES_WIDTH/2, .01, 20);
	//glVertex3f(-1 * AXES_WIDTH/2, .01, 20);
	//glVertex3f(-1 * AXES_WIDTH/2, .01, -20);
	//glVertex3f(AXES_WIDTH/2, .01, -20);
	//glEnd();
}

/**
* Called when a mouse button is pressed. Delegates to the
* application.
*/
void mouse(int button, int state, int x, int y)
{
	lastX = x;
	lastY = y;
}

/**
* Called when the display window changes size.
*/
void reshape(int width, int height)
{
}

/**
* Called when a key is pressed.
*/
void keyboard(unsigned char key, int x, int y)
{
	Controls::keyCheck(key, &particles, &rectangleObjects, &sphereObjects, theta, phi);
}


/**
* Called when the mouse is dragged.
*/
void motion(int x, int y)
{
	// Update the camera
	theta += (x - lastX)*0.25f;
	phi += (y - lastY)*0.25f;

	// Keep it in bounds
	if (phi < -20.0f) phi = -20.0f;
	else if (phi > 80.0f) phi = 80.0f;

	// Remember the position
	lastX = x;
	lastY = y;
}

void initializeGraphics()
{
	//// Create a light
	//GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat mat_shininess[] = { 50.0 };
	//GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	//glClearColor(0.0, 0.0, 0.0, 0.0);
	//glShadeModel(GL_SMOOTH);

	//glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	//glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glEnable(GL_DEPTH_TEST);
	//
	//// Enable color
	//glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	//glEnable(GL_COLOR_MATERIAL);

	//// Now set the view
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluPerspective(60.0, (double)SCREEN_WIDTH / (double)SCREEN_HEIGHT, 1.0, 500.0);
	//glMatrixMode(GL_MODELVIEW);
}

void initializeScene()
{
	// Initialize the timers
	TimingData::init();
}

/**
* The main entry point. We pass arguments onto GLUT.
*/
int main(int argc, char** argv)
{
	//// Initialize everything needed for the current scene
	//initializeScene();
	//// Set up GLUT and the timers
	//glutInit(&argc, argv);

	//// Create the application and its window
	//createWindow("PhysicsEngine");

	//// Set up the appropriate handler functions
	//glutReshapeFunc(reshape);
	//glutKeyboardFunc(keyboard);
	//glutDisplayFunc(display);
	//glutIdleFunc(update);
	//glutMouseFunc(mouse);
	//glutMotionFunc(motion);

	//// Initialize the graphics
	//initializeGraphics();

	//// Run the application
	//glutMainLoop();
}