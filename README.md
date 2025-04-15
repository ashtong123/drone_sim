a basic sim for quadcopter drones.

I will start as simple as possible and increase complexity as time goes on. I think a good first effort is to
model the system as 5 points interacting. The first point will represent the center of mass of the drone,
and the other 4 points will represent the quad rotors. Note that the points will be constrained to one another,
restricting some degrees of freedom and simplifying the system. The constraints are:

1. all 5 points are coplanar
2. 1 point is in the center, and the other 4 are radially equidistant from it in the plane described in (1)
3. We will name the points as follows: d, m1, m2, m3, m4

We will also make some simplifying assumptions about the dynamics of the system. These are as follows:
1. The center of mass will be located at d
2. All forces and torques will be applied at m1-m4
3. The force and torque at a given point will both be subject to a common proportional reponse to a single
parameter, which we will call c1-c4.
4. There will be 2 coordinate systems of interest, the global csys and the drone csys. The drone csys will be 
defined wrt d and m1-m4 as follows:
	fig 1. Drone top view:
	
	Y		
    ^				 m1	  m2
	|		          \   /
	|					d
    .______> X        /   \
	Z				 m3	  m4

5. The drone position can be defined by the coordinate system transformation from the global csys to the 
local csys of the drone defined in (4)
6. The positions of m1-m4 can be defined by vectors originating at d in the local csys
7. There is also the question of the body shape to calculate moments of inertia and drag/aerodynamic effects.
For now, we will assume a square shaped body with the forces from the motors acting on the corners. Thus,
the width and length of the body will be defined by m1-m4, and the thickness will be defined separately

