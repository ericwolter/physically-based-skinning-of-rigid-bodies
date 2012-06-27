#include "Simulation.h"

#include <limits>
#include <iostream>
using namespace std;

#define GRAVITY_FORCE -9.81f

Simulation::Simulation() {}
Simulation::~Simulation() {}

bool Simulation::findSeperatingPlane(RigidBody &a, RigidBody &b)
{
    vector<MatrixXf> potentialPlanesA;
    vector<MatrixXf> potentialPlanesB;
    // vector<MatrixXf> potentialPlanesEdges;

    vector<Face>::iterator face;
    for (face = a.mesh.faces.begin(); face != a.mesh.faces.end(); face++)
    {
        Vector3f planePoint = a.bodyPointToWorld(face->vertices[0]);
        Vector3f planeNormal = a.normalToWorld(face->normal);
        potentialPlanesA.push_back((MatrixXf(3, 2) << planePoint, planeNormal).finished());
    }
    for (face = b.mesh.faces.begin(); face != b.mesh.faces.end(); face++)
    {
        Vector3f planePoint = b.bodyPointToWorld(face->vertices[0]);
        Vector3f planeNormal = b.normalToWorld(face->normal);
        potentialPlanesB.push_back((MatrixXf(3, 2) << planePoint, planeNormal).finished());
    }

    // vector<Face>::iterator faceA;
    // for (faceA = a.mesh.faces.begin(); faceA != a.mesh.faces.end(); faceA++)
    // {
    //     vector<Vector3f>::iterator edgeA;
    //     for (edgeA = faceA->edges.begin(); edgeA != faceA->edges.end(); edgeA++)
    //     {
    //         vector<Face>::iterator faceB;
    //         for (faceB = b.mesh.faces.begin(); faceB != b.mesh.faces.end(); faceB++)
    //         {
    //             vector<Vector3f>::iterator edgeB;
    //             for (edgeB = faceB->edges.begin(); edgeB != faceB->edges.end(); edgeB++)
    //             {
    //                 Vector3f planePoint = a.bodyPointToWorld(edgeA->vertices[0]);
    //                 Vector3f planeNormal = edgeA.direction.cross(edgeB->direction).normalized();
    //                 potentialPlanesEdges.push_back((MatrixXf(3, 2) << planePoint, planeNormal).finished());
    //             }
    //         }
    //     }
    // }

    vector<MatrixXf>::iterator plane;
    for (plane = potentialPlanesA.begin(); plane != potentialPlanesA.end(); plane++)
    {
        if (testSeperatingPlane((*plane), b))
        {
        	cout << "found seperating plane" << endl;
            return true;
        }
    }
    for (plane = potentialPlanesB.begin(); plane != potentialPlanesB.end(); plane++)
    {
        if (testSeperatingPlane((*plane), a))
        {
        	cout << "found seperating plane" << endl;
            return true;
        }
    }

    // vector<MatrixXf(3, 2)>::iterator plane
    // for (plane = potentialPlanesEdges.begin(); plane != potentialPlanesEdges.end(); plane++)
    // {
    //     if (testSeperatingPlane(plane, b))
    //     {
    //         return true;
    //     }
    // }
 
    return false;
}

bool Simulation::testSeperatingPlane(MatrixXf &plane, RigidBody &body)
{
	cout << "BEGIN testSeperatingPlane" << endl;
    vector<Face>::iterator face;
    for (face = body.mesh.faces.begin(); face != body.mesh.faces.end(); face++)
    {
        vector<Vector3f>::iterator vertex;
        for (vertex = face->vertices.begin(); vertex != face->vertices.end(); vertex++)
        {
			cout << "testing plane: " << plane << endl;
			Vector3f worldPoint = body.bodyPointToWorld(*vertex);
			cout << "against point: " << worldPoint << endl;
            Vector3f fromPlaneToPoint = worldPoint - plane.col(0);
            Vector3f planeNormal = plane.col(1);

            float distance = fromPlaneToPoint.dot(planeNormal);
			cout << "distance: " << distance << endl;

            if (distance < 0)
            {
            	cout << "point is behind plane" << endl;
                return false;
            }
        }
    }

    return true;
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
            if (!findSeperatingPlane(bodies[i], bodies[j]))
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
        while (loop < 8)
        {
            sampleTime = sampleTime / 2;
        	cout << "sampleTime: " << sampleTime << endl;

            vector<RigidBody>::iterator body;
            int index = 0;
		    for (body = this->bodies.begin(); body != this->bodies.end(); body++)
		    {
			    std::cout << "integrating body " << index << std::endl;
		    	body->save();
		        body->integrate(sampleTime);
		        index++;
		    }

            bool bodiesAreColliding = false;
            for (int i = 0; i < bodies.size() && !bodiesAreColliding; i++)
            {
                for (int j = i + 1; j < bodies.size() && !bodiesAreColliding; j++)
                {
                    if (!findSeperatingPlane(bodies[i], bodies[j]))
                    {
                        bodiesAreColliding = true;
                    }
                }
            }

            if (bodiesAreColliding)
            {
            	cout << "collision before < " << sampleTime << endl;
                restore();
            }
            else
            {
            	cout << "collision after > " << sampleTime << endl;
                dt = dt - sampleTime;
            }
            
            loop += 1;
            cout << "loop: " << loop << endl;
        }

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
