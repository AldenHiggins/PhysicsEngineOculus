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

void PhysicsMain::addCube(OVR::Scene *scene, OVR::Vector3f startPosition, OVR::Vector3f startHalfsize)
{
	// Test generate the scene
	static const GLchar* VertexShaderSrc =
		"#version 150\n"
		"uniform mat4 matWVP;\n"
		"in      vec4 Position;\n"
		"in      vec4 Color;\n"
		"in      vec2 TexCoord;\n"
		"out     vec2 oTexCoord;\n"
		"out     vec4 oColor;\n"
		"void main()\n"
		"{\n"
		"   gl_Position = (matWVP * Position);\n"
		"   oTexCoord   = TexCoord;\n"
		"   oColor.rgb  = pow(Color.rgb, vec3(2.2));\n"   // convert from sRGB to linear
		"   oColor.a    = Color.a;\n"
		"}\n";

	static const char* FragmentShaderSrc =
		"#version 150\n"
		"uniform sampler2D Texture0;\n"
		"in      vec4      oColor;\n"
		"in      vec2      oTexCoord;\n"
		"out     vec4      FragColor;\n"
		"void main()\n"
		"{\n"
		"   FragColor = oColor * texture2D(Texture0, oTexCoord);\n"
		"}\n";

	GLuint    vshader = scene->CreateShader(GL_VERTEX_SHADER, VertexShaderSrc);
	GLuint    fshader = scene->CreateShader(GL_FRAGMENT_SHADER, FragmentShaderSrc);

	// Make textures
	OVR::ShaderFill * grid_material[4];
	for (int k = 0; k < 4; ++k)
	{
		static DWORD tex_pixels[256 * 256];
		for (int j = 0; j < 256; ++j)
		{
			for (int i = 0; i < 256; ++i)
			{
				if (k == 0) tex_pixels[j * 256 + i] = (((i >> 7) ^ (j >> 7)) & 1) ? 0xffb4b4b4 : 0xff505050;// floor
				if (k == 1) tex_pixels[j * 256 + i] = (((j / 4 & 15) == 0) || (((i / 4 & 15) == 0) && ((((i / 4 & 31) == 0) ^ ((j / 4 >> 4) & 1)) == 0)))
					? 0xff3c3c3c : 0xffb4b4b4;// wall
				if (k == 2) tex_pixels[j * 256 + i] = (i / 4 == 0 || j / 4 == 0) ? 0xff505050 : 0xffb4b4b4;// ceiling
				if (k == 3) tex_pixels[j * 256 + i] = 0xffffffff;// blank
			}
		}
		OVR::TextureBuffer * generated_texture = new OVR::TextureBuffer(nullptr, false, false, OVR::Sizei(256, 256), 4, (unsigned char *)tex_pixels, 1);
		grid_material[k] = new OVR::ShaderFill(vshader, fshader, generated_texture);
	}

	glDeleteShader(vshader);
	glDeleteShader(fshader);

	// Add models to the scene
	OVR::Model * m = new OVR::Model(OVR::Vector3f(0, 0, 0), grid_material[1]);  // Walls
	m->AddSolidColorBox(startPosition[0] - startHalfsize[0], startPosition[1] - startHalfsize[1], startPosition[2] - startHalfsize[2],
		startPosition[0] + startHalfsize[0], startPosition[1] + startHalfsize[1], startPosition[2] + startHalfsize[2], 0xff808080); // Left Wall
	m->AllocateBuffers();
	scene->Add(m);


	RectangleObject *newSquare = new RectangleObject(m);
	newSquare->setState(Vector3(startPosition[0], startPosition[1], startPosition[2]), Vector3(0.0f, 0.0f, 0.0f), Vector3::GRAVITY, 1.0f, Vector3(startHalfsize[0], startHalfsize[1], startHalfsize[2]));
	rectangleObjects.push_back(newSquare);
}

// Update the physics engine for this frame
void PhysicsMain::updatePhysics()
{
	// Update the timing for this frame
	TimingData::get().update();
	float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
	if (duration <= 0.0f) return;
	else if (duration > 0.05f) duration = 0.05f;

	// Integrate all of the rigid bodies
	integrateRigidBodies(duration);

	// Check for and resolve collisions
	std::vector<Collision> collisionList;
	detectCollisions(&collisionList);
	resolveCollisions(&collisionList, duration);

	
	
	//// Draw all of the particles
	//for (unsigned int particleIndex = 0; particleIndex < particles.size(); particleIndex++)
	//{
	//	particles[particleIndex]->display();
	//}
	//
	//// Draw all cubes
	//for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < rectangleObjects.size(); rigidBodyIndex++)
	//{
	//	rectangleObjects[rigidBodyIndex]->display();
	//}

	//// Draw all spheres
	//for (unsigned int rigidBodyIndex = 0; rigidBodyIndex < sphereObjects.size(); rigidBodyIndex++)
	//{
	//	sphereObjects[rigidBodyIndex]->display();
	//}
}

// Resolve the found collisions
void PhysicsMain::resolveCollisions(std::vector<Collision> *collisionList, real duration)
{
	if (collisionList->size() > 0)
	{
		CollisionResolver resolver(collisionList->size() * 4, collisionList->size() * 4);
		resolver.resolveContacts(collisionList, duration);
	}
}

// Detect collisions
void PhysicsMain::detectCollisions(std::vector<Collision> *collisionList)
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
void PhysicsMain::integrateRigidBodies(real duration)
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
