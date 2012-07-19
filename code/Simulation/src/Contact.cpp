#include <iostream>

#include "Contact.h"

Contact::Contact() {}
Contact::~Contact() {}

const float THRESHOLD = 0.01f;

/* returns TRUE if bodies are colliding
 The THRESHOLD parameter just provides some numerically stability
 */
bool Contact::isColliding()
{
    Vector3f velocityA = a->linearVelocity + (a->angularVelocity.cross(point - a->position));
    Vector3f velocityB = b->linearVelocity + (b->angularVelocity.cross(point - b->position));

    float relativeVelocity = normal.dot(velocityA - velocityB);

    if (relativeVelocity > THRESHOLD) // moving away
    {
        return false;
    }
    else if (relativeVelocity > -THRESHOLD) // resting contact
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Contact::resolve(float epsilon)
{
    std::cout << "contact resolve" << std::endl;
    std::cout << "addressA: " << static_cast<void const *>(&a) << std::endl;
    std::cout << "addressB: " << static_cast<void const *>(&b) << std::endl;
    std::cout << "point: " << point << std::endl;
    std::cout << "positionA: "<< a->position << std::endl;
    std::cout << "linearA: "<< a->linearVelocity << std::endl;
    std::cout << "angularA: "<< a->angularVelocity << std::endl;
    std::cout << "positionB: "<< b->position << std::endl;
    std::cout << "linearB: "<< b->linearVelocity << std::endl;
    std::cout << "angularB: "<< b->angularVelocity << std::endl;
    Vector3f velocityA = a->linearVelocity + (a->angularVelocity.cross(point - a->position));
    std::cout << "velocityA: " << velocityA << std::endl;
    Vector3f velocityB = b->linearVelocity + (b->angularVelocity.cross(point - b->position));
    std::cout << "velocityB: " << velocityB << std::endl;

    Vector3f relativePositionA = point - a->position;
    std::cout << "relativePositionA: " << relativePositionA << std::endl;
    Vector3f relativePositionB = point - b->position;
    std::cout << "relativePositionB: " << relativePositionB << std::endl;

    float relativeVelocity = normal.dot(velocityA - velocityB);
    std::cout << "relativeVelocity: " << relativeVelocity << std::endl; 
    float numerator = -(1 + epsilon) * relativeVelocity;

    float term1 = a->massInverse;
    float term2 = b->massInverse;
    float term3 = normal.dot((a->inertiaInverse * relativePositionA.cross(normal)).cross(relativePositionA));
    float term4 = normal.dot((b->inertiaInverse * relativePositionB.cross(normal)).cross(relativePositionB));

    float j = numerator / (term1 + term2 + term3 + term4);
    Vector3f force = j * normal;
    std::cout << "force: " << force << std::endl;

    std::cout << "oldLinearMomA: " << a->linearMomentum << std::endl;
    a->linearMomentum += force;
    std::cout << "newLinearMomA: " << a->linearMomentum << std::endl;
    b->linearMomentum -= force;

    a->angularMomentum += relativePositionA.cross(force);
    b->angularMomentum -= relativePositionB.cross(force);

    // a->linearVelocity = a->linearMomentum * a->massInverse;
    // b->linearVelocity = b->linearMomentum * b->massInverse;

    // a->angularVelocity = a->inertiaInverse * a->angularMomentum;
    // b->angularVelocity = b->inertiaInverse * b->angularMomentum;
}