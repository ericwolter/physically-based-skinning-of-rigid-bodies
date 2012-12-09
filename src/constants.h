#ifndef __Thesis__constants__
#define __Thesis__constants__

#include <BulletDynamics/btBulletDynamicsCommon.h>

// the scenario to setup
#define SCENARIO 4

// the body type used - false = rigid, true = combined
static const bool body_type = true;

// the step size used for the simulation loop
static const float stepSize = 0.001f;

// the density of the particles attached to the combined body
// this specifices the number of particles on each edge forming the grid
static const int density = 1;

// the group size of the implicit shape matching groups
static const int groupSize = 5;

// the extrude factor used to project attached particles to outer particles
static const float extrude = 2.0f;

// margin used in numerically sensitive comparisons
static const float margin = 0.0001f;

// the stiffness used in shape matching
static const float stiffness = 0.01f;

// the friction factor used between two rigid bodies
static const float rigid_friction = 0.5f;

// the friction factors used for oriented particles
static const float linear_friction = 0.5f;
static const float angular_friction = 0.5f;

#if (SCENARIO == 1)
static const btVector3 pos_rigid_finger1 = btVector3(-2.5f, 3.0f, 0.0f);
static const btVector3 pos_rigid_finger2 = btVector3(2.5f, 3.0f, 0.0f);
static const btScalar angle_rigid_finger1 = 0.0f;
static const btScalar angle_rigid_finger2 = -0.0f;
static const btVector3 pos_combined_finger1 = btVector3(-3.5f, 3.0f, 0.0f);
static const btVector3 pos_combined_finger2 = btVector3(3.5f, 3.0f, 0.0f);
static const btScalar angle_combined_finger1 = 0.0f;
static const btScalar angle_combined_finger2 = -0.0f;
static const btVector3 size_block = btVector3(1.0f, 3.0f, 1.0f);
static const btVector3 pos_block = btVector3(0.0f, 3.5f, 0.0f);
#endif
#if (SCENARIO == 2)
static const btVector3 pos_rigid_finger1 = btVector3(-2.5f, 3.0f, 0.0f);
static const btVector3 pos_rigid_finger2 = btVector3(2.5f, 3.0f, 0.0f);
static const btScalar angle_rigid_finger1 = 0.3f;
static const btScalar angle_rigid_finger2 = -0.3f;
static const btVector3 pos_combined_finger1 = btVector3(-3.5f, 3.0f, 0.0f);
static const btVector3 pos_combined_finger2 = btVector3(3.5f, 3.0f, 0.0f);
static const btScalar angle_combined_finger1 = 0.3f;
static const btScalar angle_combined_finger2 = -0.3f;
static const btVector3 size_block = btVector3(1.0f, 3.0f, 1.0f);
static const btVector3 pos_block = btVector3(0.0f, 3.5f, 0.0f);
#endif
#if (SCENARIO == 3)
static const btVector3 pos_rigid_finger1 = btVector3(-2.5f, 3.0f, 0.0f);
static const btVector3 pos_rigid_finger2 = btVector3(2.5f, 3.0f, 0.0f);
static const btScalar angle_rigid_finger1 = 0.3f;
static const btScalar angle_rigid_finger2 = -0.3f;
static const btVector3 pos_combined_finger1 = btVector3(-3.5f, 3.0f, 0.0f);
static const btVector3 pos_combined_finger2 = btVector3(3.5f, 3.0f, 0.0f);
static const btScalar angle_combined_finger1 = 0.3f;
static const btScalar angle_combined_finger2 = -0.3f;
static const btVector3 size_block = btVector3(1.0f, 6.0f, 1.0f);
static const btVector3 pos_block = btVector3(0.0f, 6.5f, 0.0f);
#endif
#if (SCENARIO == 4)
static const btVector3 pos_rigid_finger1 = btVector3(-3.0f, 3.0f, 0.0f);
static const btVector3 pos_rigid_finger2 = btVector3(3.0f, 3.0f, 0.0f);
static const btScalar angle_rigid_finger1 = 0.3f;
static const btScalar angle_rigid_finger2 = -0.3f;
static const btVector3 pos_combined_finger1 = btVector3(-4.5f, 3.0f, 0.0f);
static const btVector3 pos_combined_finger2 = btVector3(4.5f, 3.0f, 0.0f);
static const btScalar angle_combined_finger1 = 0.3f;
static const btScalar angle_combined_finger2 = -0.3f;
static const btVector3 size_block = btVector3(1.0f, 6.0f, 6.0f);
static const btVector3 pos_block = btVector3(0.0f, 6.5f, 0.0f);
#endif

#endif