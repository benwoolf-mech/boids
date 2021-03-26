# -*- coding: utf-8 -*-
"""
Created on Sat Aug 29 18:29:01 2020

@author: benwo
"""

import math as mt
import matplotlib.pyplot as pl
from matplotlib import animation
from random import random

width = 10          # box size
n_boids = 30        # number of boids
view_dist = 1       # view distance for boids

dt = 0.01
timestop = 30       # how long to simulate for
number_frames = int(timestop/dt) + 1

# play with these factors to change magnitude of forces

allignment_factor = 1       # allignment with nearby boids
lin_collision_factor = 6    # collision avoidance 
com_factor = 3      # centre of mass force

# wall_collision_factor = 8

boid_speed = 2  # metres/sec

# function find smallest difference of two headings, with correct sign
# all angles stored in radians, where 0 is 'east'
def get_angle_diff(main, other):        
    diff = other - main
    if diff > mt.pi:
        diff = diff - 2*mt.pi
    if diff < - mt.pi:
        diff = diff + 2*mt.pi
    return(diff)

# use arctan to generate headings but in the format 0 < theta < 2*pi
def get_arctan(x,y):     
    theta = mt.atan2(y,x)
    if theta < 0:
        theta = theta + 2 * mt.pi
    return(theta)

# ignore box stuff, thats to try and implement wall avoidance later
# this class would contain all points that boids would attempt to avoid collision with
class Box:  	                   
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.x_list = []
        self.y_list = []

# define a Boid class to contain the position, speed and heading of each boid
class Boid:
    def __init__(self, position, direction, speed):
        
        self.position = position
        self.direction = direction
        self.speed = speed
        self.nearby_boids = []      # array of all boids that are within view
        self.is_near = False        # true if there are any boids within view
    
    def find_new_position(self):        # function to generate next position using current position and heading,
        self.position[0] = self.position[0] + mt.cos(self.direction) * self.speed * dt
        self.position[1] = self.position[1] + mt.sin(self.direction) * self.speed * dt
                
        for i in range(0,2):        # if the next position is outside the boundaries, move it to the opposite side
            if self.position[i] > width:
                self.position[i] = self.position[i] - width
            if self.position[i] < 0:
                self.position[i] = self.position[i] + width

    def find_nearby_boids(self):        # function to generate list of boids that are within the view distance
        for boid in boids:
            distance = mt.sqrt((boid.position[0] - self.position[0])**2 + (boid.position[1] - self.position[1])**2)
            if distance < view_dist and distance != 0:
                self.is_near = True
                self.nearby_boids.append(boid)
      
    def get_avg_direction(self):        # calculate average direction of travel for nearby boids
        sum_x = 0
        sum_y = 0
        for boid in self.nearby_boids:
            sum_x = sum_x + mt.cos(boid.direction)
            sum_y = sum_y + mt.sin(boid.direction)
        
        self.avg_direction = get_arctan(sum_x, sum_y)
        
    def allignment_correction(self):        # make a correction to direction of travel towards average for nearby boids
            
        self.difference = get_angle_diff(self.avg_direction, self.direction)            
        self.direction = self.direction - self.difference * dt * allignment_factor
            
    def avoid_boid_collision(self):         # compare current direction of travel to direction of nearby boids

        for boid in self.nearby_boids:
 
            x_dist =  boid.position[0] - self.position[0]
            y_dist =  boid.position[1] - self.position[1]
            distance = mt.sqrt(x_dist**2 + y_dist**2)
            heading = get_arctan(x_dist,y_dist)
            relative_heading = get_angle_diff(self.direction, heading)

            if abs(relative_heading) < 0.8 * mt.pi :            # doesn't correct if neighbour is behind boid
                if relative_heading > 0 :               # correction is stronger as distance to neighbour decreases
                    self.direction = self.direction - dt * lin_collision_factor * mt.exp( -distance)
                else:
                    self.direction = self.direction + dt * lin_collision_factor * mt.exp( -distance)

    def avoid_wall_collision(self):     # function to implement an obstacle avoidance capability

        nearby_obstacles = []
        rel_headings = []
        distances = []
        
        for point in box.obstacles:   
            
            x_dist =  point[0] - self.position[0]
            y_dist =  point[1] - self.position[1]
            distance = mt.sqrt(x_dist**2 + y_dist**2)
            
            
            if distance < view_dist:
                
                nearby_obstacles.append(point)
                
                heading = ( get_arctan(x_dist, y_dist) )
                
                rel_headings.append( get_angle_diff( self.direction, heading) )
                
                distances.append(distance)
                
        if len(nearby_obstacles) != 0:
            minimum_heading = min(rel_headings)
            distance_for_min_bearing = distances[rel_headings.index(min(rel_headings))]
            
            if abs(minimum_heading) < 0.25*mt.pi:
                if minimum_heading > 0 :
                    self.direction = self.direction - dt * wall_collision_factor * 1/(distance_for_min_bearing)
                else:
                    self.direction = self.direction + dt * wall_collision_factor * 1/(distance_for_min_bearing)

        
    def get_com_heading(self):      # generate centre of mass of neighbours and heading relative to current boid

        avg_x = 0
        avg_y = 0
        for boid in self.nearby_boids:
            avg_x = avg_x + boid.position[0]
            avg_y = avg_y + boid.position[1]

        avg_x = avg_x/len(self.nearby_boids)
        avg_y = avg_y/len(self.nearby_boids)

        x_dist = avg_x - self.position[0]
        y_dist = avg_y - self.position[1]
                    
        self.com_heading = get_arctan(x_dist, y_dist)  #direction towards centre of mass
            
    def aim_at_com(self):   #finds difference of com direction and current travel direction, corrects course

        difference = get_angle_diff(self.direction, self.com_heading) 
#        print(difference)   
        if abs(difference) < 0.7 * mt.pi:
            self.direction = self.direction + dt * difference * com_factor
        
        
def get_boid_directions():      # function to make list of boid directions for the animation
    cosines = []
    sines = []
    for boid in boids:
        cosines.append(mt.cos(boid.direction))
        sines.append(mt.sin(boid.direction))
    return[cosines, sines]


# ------------- MAIN SCRIPT ----------------

# initialise boids randomly distributed in space
boids = []
for i in range(0, n_boids ):
    boids.append(Boid(  [10*random(), 10*random()]  , random()*2*mt.pi, boid_speed))

# initialise list of timesteps containing list for x and y coords, for the animation
x_coords = [[]]
y_coords = [[]]

fig = pl.figure()
images = []

# for each boid, record boid position in timestep
for boid in boids:
    x_coords[-1].append(boid.position[0])
    y_coords[-1].append(boid.position[1])

for timestep in range(0, number_frames):    # for every timestep:
    
    x_coords.append([])     #insert x and y coordinate list into timestep list
    y_coords.append([])  
    
    for boid in boids:      # cycle through each boid
        
        boid.find_nearby_boids()    # create list of nearby boids

        if boid.is_near == True:        # if there are any boids within view:

            boid.get_avg_direction()            # find average direction of nearby boids
            boid.get_com_heading()             # find direction of centre of mass of nearby boids

            # comment out different forces to see how they act in isolation

            boid.allignment_correction()           # ALLIGNMENT FORCE
            boid.avoid_boid_collision()                  # COLLISION FORCE
            boid.aim_at_com()                      # CENTRE OF MASS FORCE
   
#        boid.avoid_wall_collision()             # AVOID WALLS
            
        boid.find_new_position()    # move to next position
        
        boid.nearby_boids = []      #reset nearby boid checks
        boid.direction = boid.direction % (2*mt.pi)     # if boid direction exceeds 2*pi, trim down
        boid.is_near = False   
   
    
    # for each boid, record boid position in timestep
    for boid in boids:
        x_coords[-1].append(boid.position[0])
        y_coords[-1].append(boid.position[1])
    
    directions = get_boid_directions()  # store all current boid headings for animation
    
    # produce a frame for the animation
    image = pl.quiver(x_coords[timestep], y_coords[timestep], directions[0], directions[1], color = 'olive', animated=True )

    pl.axis([0, 10, 0, 10])
    images.append([image])

a = animation.ArtistAnimation(fig, images, interval = 1000*dt , blit=True)
#pl.plot(box.x_list,box.y_list, color = 'black')
#%%


# test space for messing around, not part of the script

boids = []

boids.append(Boid( [5.0, 3.5] , 0 , 1))

x = []
y = []

for boid in boids:
    x.append(boid.position[0])
    y.append(boid.position[1])
    
boids[0].avoid_wall_collision()

pl.scatter(x,y) 
pl.axis([0,10,0,10])
pl.plot(box.x_list,box.y_list, color = 'black')

#%%




