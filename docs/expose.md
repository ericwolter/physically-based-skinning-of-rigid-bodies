# Title
Combining oriented particles deformable-body simulation with rigid-body simulations

# Abstract
This paper presents a system that combines the oriented particles approach of simulating deformable-bodies with a traditional rigid-body simulation.
The system provides an interface between the two simulations. Oriented particles are arranged like a cushion around an inner rigid body. Forces acting on the outer particles are transferred to the core. Especially scenarios, which in the past required multiple rigid bodies connected which hinges, can now be simulated using just a single rigid-body with a cushion of oriented particles. The surrounding deformable body parts provide a large contact area, which enables more numerical stable integrations.

# Steps
1. Rigid-Body Simulation (early July)
2. Oriented Particles Simulation (end of July)
3. Combining Simulations (August)
4. Writing thesis (September)

## Rigid-Body Simulation (early July)
 - Implementing OpenGL graphics context (done)
 - Loading of convex polyhedrons from *.obj files (done)
 - Calculating intrinsics (done)
 - Implementing forces and integration (done)
 - Implementing colliding contact (in progress)
 - Implementing resting contact??? necessary???

## Oriented Particles Simulation (end of July)
 - Creating particles
 - Implementing shape matching
 - Implementing collisions

## Combing Simulations (August)
 - Define contact interface
 - Implementing force propagation

## Writing thesis (September)
 - Defining rough outline
 - 
 - 

# References
## Rigid-body simulation
### How to find the inertia tensor (or other mass properties) of a 3D solid body represented by a triangle mesh by Jonathan Blow, Atman J Binstock
Describes a simpler approach to calculating the intrinsics of any convex polyhedron. Directly used in the implementation for the rigid bodies.

### Physically Based Modeling: Principles and Practice by Andrew Witkin, David Baraff
The rigid-body simulation is directly modeled after this SIGGRAPH courses section on Rigid Body Dynamics. Both the way the overall simulation is handled as well as the approach to collision handling is taken from these course notes.

### Rigid Body Dynamics by Chris Hecker
These rather old articles provide a very good overview of the different parts and ideas behind rigid-body simulations. This is mostly used to fill in some understading gaps in the siggraph course

### Studienarbeit Rigid Body Simulation by Stefan Rilling
Is the first hit in google for "Rigid body simulation". Again used to get a better understanding of fundamental rigid-body simulations from another angle. Especially section on collision handling helps to better grasp the problem.

## Deformable-body simulation
### Solid Simulations with Oriented Particles by Müller et al.
Used as the canoical implementation of the pure oriented particles approach. 

### Meshless deformations based on shape matching by Müller et al.
Gives more background on the shape matching approach used for the oriented particles. This helps in understanding the underlying archiecture better and giving hints for the concrete implementation of oriented particles

### Position based dynamics by Müller et al.
This paper describes the original position based dynamics approach and being the basis for oriented particles provides valuable inside in the underlying physics and therfore helps in implementing the oriented particles themselves.

## Combined Simulation
The following papers currently just represent a pool of possible inspirations for implementing the combined system. They have not been extensivly checked for their concrete relevance and are just provided "as-is".
### Physically based models with rigid and deformable components by Terzopoulos
### Combining deformable- and rigid-body mechanics simulation by Jansson
### Mixing deformable and rigid-body mechanics simulation by Lenoir et al.
### Hybrid simulation of deformable solids by Sifakis et al.
### Two-way Coupling of Rigid and Deformable Bodies by Shinar et al.
### Adding Physics to Animated Characters with Oriented Particles by Müller et al.
### Coupled hard–soft tissue simulation with contact and constraints applied to jaw–tongue–hyoid dynamics by Stavness et al.