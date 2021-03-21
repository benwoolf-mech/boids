# boids
Personal project attempting to simulate behaviour of 'bird-oid' objects, e.g boids. 


This script simulates a flock of two dimensional objects that move at constant speed, continuously correcting their course according their neighbours.
The resulting behaviour should resemble a flock of starlings, or a swarm of fish.
The boids have a limited field of view, hence they will only react to nearby boids.

This emergent behaviour is a product of three basic rules:

      1) Try to match the direction of all neary boids. (allignment force)
          -> Each boid will average the directions of all boids within its field of view and correct towards that heading.
          
      2) Try to be at the centre of the flock. (centre-of-mass force)
          -> Each boid will compute the 'centre of mass' for nearby boids and correct towards that point.
          
      3) Try not to collide with other boids. (anti-collision force)
          -> Each boid will make a heading change if its heading is directly towards another nearby boid.

The three forces essentially take the form of adjustments to the heading of a given boid at each time increment.
The forces can cancel eachother out and form an equilibrium, or can cause jumpy and erratic behaviour as the dominant force suddenly changes.

Some key paramaters can be adjusted to observe the effect on the system:
- Number of boids
- Speed of boids
- Boid field of view distance
- Strength of three forces (allignment, centre-of-mass and anti-collision)
- Timestep (default is 10ms, increase for faster computation but less smooth results)
- Size of the box in which the boids are moving

When running, the script will simulate for a fixed time period and then produce an animation of the results. 
