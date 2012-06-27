#include "Contact.h"

Contact::Contact() {}
Contact::~Contact() {}

const float THRESHOLD = 0.01f;

/* returns TRUE if bodies are colliding
 The THRESHOLD parameter just provides some numerically stability
 */
bool Contact::isColliding()
{
    Vector3f velocityA = a.linearVelocity + (a.angularVelocity.cross(point - a.position));
    Vector3f velocityB = b.linearVelocity + (b.angularVelocity.cross(point - b.position));

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
    Vector3f velocityA = a.linearVelocity + (a.angularVelocity.cross(point - a.position));
    Vector3f velocityB = b.linearVelocity + (b.angularVelocity.cross(point - b.position));

    Vector3f relativePositionA = point - a.position;
    Vector3f relativePositionB = point - b.position;

    float relativeVelocity = normal.dot(velocityA - velocityB);
    float numerator = -(1 + epsilon) * relativeVelocity;

    float term1 = a.massInverse;
    float term2 = b.massInverse;
    float term3 = normal.dot((a.inertiaInverse * relativePositionA.cross(normal)).cross(relativePositionA));
    float term4 = normal.dot((b.inertiaInverse * relativePositionB.cross(normal)).cross(relativePositionB));

    float j = numerator / (term1 + term2 + term3 + term4);
    Vector3f force = j * normal;

    a.linearMomentum += force;
    b.linearMomentum -= force;

    a.angularMomentum += relativePositionA.cross(force);
    b.angularMomentum -= relativePositionB.cross(force);

    // a.linearVelocity = a.linearMomentum * a.massInverse;
    // b.linearVelocity = b.linearMomentum * b.massInverse;

    // a.angularVelocity = a.inertiaInverse * a.angularMomentum;
    // b.angularVelocity = b.inertiaInverse * b.angularMomentum;
}