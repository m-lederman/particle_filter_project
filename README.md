# Particle Filter Project

## Group Members

David Pan, Max Lederman

## Implementation Plan

### initialize_particle_cloud

For this function, given a map of the maze, we will generate a randomly
sampled particle cloud that covers all of the empty spaces. We will test
our implementation using the `visualize_partices` program.

### update_particles_with_motion_model

To update the particle positions, we will move the particles in the same way
that we command the robot to move with some amount of noise added. To test
this, we will put many particles in one spot and see if the cloud resulting
from the motion model makes sense. 

### update_particle_weights_with_measurement_model

Given the current particle cloud and the measurements from the laser scan,
we would update the weights by simulating what the robot would see in each
particle's position and comparing it to the real measurements. We will test
this on a cloud with only a few particles in different positions and see if
the updated weights make sense.

### normalize_particles

To normalize the partices' importance weights, we will scale their weights
by the same factor so that their sum is 1. We will test this by checking the
sum of all of the particle weights before and after normalization.

### resample_particles

We will resample the particles by generating randomly selected particles from
the old population where the probability of choosing each particle is given
by its normalized weight. We will test this by doing runs with a small
number of particles.

### estimate_robot_pose

We will update the estimated pose of the robot by setting it to the particle
in the cloud that has the largest weight. We will test this by comparing it
to the real position of the robot.

### Incorporating Noise

We will incorporate noise into the motion model by making each particle move
and rotate slightly differently. We will also account for noise by averaging
the robot's laser scan readings with a few adjacent readings. We will
test our implentation of noise by putting many particles at one position and
seeing how they spread out over multiple steps.

### Timeline

Initializing particle cloud: Wed 4/13

Updating particles with motion model: Mon 4/18

Updating particle weights: Mon 4/18

Normalizing and resampling: Wed 4/20

Updating estimated pose: Fri 4/22

## Writeup

### Objectives

The main objective of this project was to implement a particle filter 
localization algorithm for determining the location of the robot given a map
of its environment and readings from its LiDAR and odometry. The robot is
steered around the environment via teleoperation, and as its sensors gather
more information, the particles converge on the robot's real location.

### High-level Description

ABC

### Components

#### Initializing the particle cloud

Location: `ParticleFilter.initialize_particle_cloud` in particle_filter.py

Code description: Our implementation creates the initial particle cloud by
placing particles uniformly randomly around the vacant cells in the map with
uniformly random orientations.

#### The movement model

Location: `ParticleFilter.update_particles_with_motion_model` in particle_filter.py

Code description: Our implementation first rotates the particles, then moves
them forward or backward. The amount to move the particles is determined by
taking the distance the robot moved in the direction of the average between
its previous yaw and current yaw.

#### The measurement model

Location: `ParticleFilter.update_particle_weights_with_measurement_model` in particle_filter.py

Code description: We used the likelihood field method for this function.

#### Resampling the particles

Location: `ParticleFilter.resample_particles` in particle_filter.py

Code description: This function uses the `draw_random_sample` method. We draw
particles from the current particle cloud with probabilities equal to their
weights. In doing this, we make sure to use copies of each chosen particle 
instead of object references to avoid multiple particles in the cloud
pointing to the same object.

#### Incorporating noise

Location: `ParticleFilter.update_particles_with_motion_model` in particle_filter.py

Code description:

#### Updating the estimated robot pose

Location: `ParticleFilter.update_estimated_robot_pose` in particle_filter.py

Code description:

#### Optimizing the parameters

### Challenges

### Future work

### Takeaways
