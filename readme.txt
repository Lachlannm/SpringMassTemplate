Lachlann Macuisdin
V00943411

CSC 486A Assignment 2 - Part 2

The scene TestCase located in the scenes folder contains the three cubes with different parameters for this part 
of the assignment. All parts of the assignment are completed based on the requirements given in brightspace.

1)
Every vertex given from the mesh filter is used to create a particle.

2)
Every particle has a spring connection to every other particle, with the rest length automatically set.

3)
The ground plane is created based on the object.

4)
The ground contact forces are initialized when contact is first detected, with the contact point being
the nearest point on the plane. The contact spring uses the given values.

5)
The mesh is updated automatically after the simulation step is completed.

6)
The spring forces are calculated as instructed for each pair of particles.

7)
The mesh bounds and normals are updated after the mesh vertices.

8)
Symplectic Euler is used in the simulation step.

9)
The simulator works as described in the notes.

10)
The test case used is the one as given in the template.