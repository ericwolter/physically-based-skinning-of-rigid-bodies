#ifndef __Thesis__constants__
#define __Thesis__constants__

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
static const float rigid_friction = 0.6f;

#endif