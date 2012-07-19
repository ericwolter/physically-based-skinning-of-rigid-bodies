#include "Simulation.h"

#include <limits>
#include <iostream>
#include <algorithm>
#include <set>
using namespace std;

#define GRAVITY_FORCE -9.81f
#define THRESHOLD 0.1f

Simulation::Simulation() {}
Simulation::~Simulation() {}

CollisionState Simulation::areColliding(RigidBody &a, RigidBody &b)
{
    struct CollisionState collision1 = findSeperatingPlane(a, b);
    struct CollisionState collision2 = findSeperatingPlane(b, a);

    struct CollisionState collisionResult;
    collisionResult.type = CLEAR;

    if (collision1.type == CLEAR && collision2.type == CLEAR)
    {
        collisionResult.type = CLEAR;
    } 
    else if (collision1.type == PENETRATING || collision2.type == PENETRATING)
    {
        collisionResult.type = PENETRATING;
    } 
    else if (collision1.type == COLLIDING || collision2.type == COLLIDING)
    {
        collisionResult.type = COLLIDING;
        if (collision1.type == COLLIDING)
        {
            vector<Contact>::iterator contact;
            for (contact = collision1.contacts.begin(); contact != collision1.contacts.end(); contact++)
            {
                collisionResult.contacts.push_back(*contact);
            }
        }
        if (collision2.type == COLLIDING)
        {
            vector<Contact>::iterator contact;
            for (contact = collision2.contacts.begin(); contact != collision2.contacts.end(); contact++)
            {
                collisionResult.contacts.push_back(*contact);
            }
        }
    }

    return collisionResult;
}

CollisionState Simulation::findSeperatingPlane(RigidBody &a, RigidBody &b)
{
    std::vector<Vector3f> alreadyFoundContactPoints;

    struct CollisionState collision;
    collision.type = CLEAR;

    vector<Face>::iterator face1;
    for (face1 = a.mesh.faces.begin(); (face1 != a.mesh.faces.end() && collision.type != PENETRATING); face1++)
    {
        vector<Vector3f>::iterator vertex1;
        for (vertex1 = face1->vertices.begin(); (vertex1 != face1->vertices.end() && collision.type != PENETRATING); vertex1++)
        {
            Vector3f worldPoint = a.bodyPointToWorld(*vertex1);
            // cout << "worldPoint: " << worldPoint << endl;

            bool vertexIsInside = true;
            bool vertexIsColliding = false;
            float closestDistance = THRESHOLD+1;
            Vector3f closestNormal = Vector3f::Zero();

            vector<Face>::iterator face2;
            for (face2 = b.mesh.faces.begin(); face2 != b.mesh.faces.end(); face2++)
            {
                Vector3f planePoint = b.bodyPointToWorld(face2->vertices[0]);
                // cout << "planePoint: " << planePoint << endl;
                Vector3f planeNormal = b.normalToWorld(face2->normal);
                // cout << "planeNormal: " << planeNormal << endl;
                Vector3f fromPlaneToPoint = worldPoint - planePoint;

                float distance = fromPlaneToPoint.dot(planeNormal);
                // cout << "distance: " << distance << endl;

                if (distance > THRESHOLD) {
                    vertexIsInside = false;
                    break;
                } 
                else if (distance >= - THRESHOLD && distance <= THRESHOLD)
                {
                    if (fabs(distance) < closestDistance) {
                        closestDistance = fabs(distance);
                        closestNormal = planeNormal;
                    }
                    vertexIsInside = true;
                    vertexIsColliding = true;
                } else {
                    vertexIsInside = true;
                }
            }

            // cout << "vertexIsInside: " << vertexIsInside << endl;
            // cout << "vertexIsColliding " << vertexIsColliding << endl;

            if(vertexIsInside && !vertexIsColliding)
            {
                collision.type = PENETRATING;
            } 
            else if (vertexIsInside && vertexIsColliding)
            {
                bool foundOldPoint = false;
                vector<Vector3f>::iterator oldContactPoint;
                for (oldContactPoint = alreadyFoundContactPoints.begin(); oldContactPoint != alreadyFoundContactPoints.end(); oldContactPoint++)
                {
                    // cout << "oldContactPoint: " << *oldContactPoint << endl;
                    // cout << "newContactPoint: " << worldPoint << endl;
                    if (*oldContactPoint == worldPoint)
                    {
                        foundOldPoint = true;
                        break;
                    }
                }

                if (!foundOldPoint) {
                    alreadyFoundContactPoints.push_back(worldPoint);

                    collision.type = COLLIDING;
                    Contact c = Contact();
                    c.a = &a;
                    c.b = &b;
                    c.point = worldPoint;
                    c.normal = closestNormal;
                    collision.contacts.push_back(c);
                }
            }
        }
    }

    return collision;
}

void Simulation::step(float dt)
{
	cout << "applying external forces" << endl;
    vector<RigidBody>::iterator body;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
        body->force << 0.0f, GRAVITY_FORCE, 0.0f;
    }
    cout << "predicting future state" << endl;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
    	body->save();
        body->integrate(dt);
    }

    cout << "checking if bodies collide at all" << endl;
    bool bodiesAreColliding = false;
    for (int i = 0; i < bodies.size() && !bodiesAreColliding; i++)
    {
        for (int j = i + 1; j < bodies.size() && !bodiesAreColliding; j++)
        {
            struct CollisionState collision = areColliding(bodies[i], bodies[j]);
            if (collision.type != CLEAR)
            {
                bodiesAreColliding = true;
            }
        }
    }
    if (bodiesAreColliding)
    {
	    cout << "some bodies are colliding" << endl;

        restore();

        int loop = 0;
        float sampleTime = dt;
        dt = 0;
        while (loop < 16)
        {
            sampleTime = sampleTime / 2;
            cout << "sampleTime: " << sampleTime << endl;
            dt = dt + sampleTime;

            vector<RigidBody>::iterator body;
            int index = 0;
		    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
		    {
			    std::cout << "integrating body " << index << std::endl;
		    	body->save();
		        body->integrate(sampleTime);
		        index++;
		    }

            this->contacts.clear();
            bool bodiesAreColliding = false;
            for (int i = 0; i < bodies.size() && !bodiesAreColliding; i++)
            {
                for (int j = i + 1; j < bodies.size() && !bodiesAreColliding; j++)
                {
                    struct CollisionState collision = areColliding(bodies[i], bodies[j]);
                    if (collision.type == PENETRATING)
                    {
                        bodiesAreColliding = true;
                    }
                    else if (collision.type == COLLIDING)
                    {
                        cout << "COLLIDING: " << collision.contacts.size() << endl;
                        vector<Contact>::iterator contact;
                        for (contact = collision.contacts.begin(); contact != collision.contacts.end(); contact++)
                        {
                            this->contacts.push_back(*contact);   
                        }
                    }
                }
            }

            if (bodiesAreColliding)
            {
            	cout << "collision before < " << dt << endl;
                restore();
                dt = dt - sampleTime;
            }
            else
            {
            	cout << "collision after > " << dt << endl;
            }
            
            loop += 1;
            cout << "loop: " << loop << endl;
        }

        cout << "finding relevant contact points" << endl;
        cout << "number of contact points: " << this->contacts.size() << endl;

        // find relevant contact points
        // for all contact points -> resolve
        vector<Contact>::iterator contact;
        for (contact = this->contacts.begin(); contact != this->contacts.end(); contact++)
        {
            contact->resolve(0.5f);
        }
    }
}

void Simulation::render(Affine3f cameraTransform, GLuint modelToCameraMatrixUnif)
{
    vector<RigidBody>::iterator body;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
        body->render(cameraTransform, modelToCameraMatrixUnif);
    }
}

void Simulation::restore()
{
    vector<RigidBody>::iterator body;
    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
    {
        body->restore();
    }
}
