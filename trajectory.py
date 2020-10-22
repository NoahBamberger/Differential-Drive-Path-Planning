<<<<<<< HEAD
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 14:05:15 2020

@author: Noah Bamberger

This program will generate a wheel velocity trajectory for a differential or
"tank" drive robot from a cubic spline path from the path file in this library.
It does this by iterating over a list of velocities indexed to each point
in the path, constraining it first by curvature of the spline, then by
acceleration and deceleration. The return of the get_trajectory_data function
is a json file with an array containing:
    
[0] waypoints (input points)
[1] time_index (list of time elapsed at each point)
[2] center_path (2d numpy array of robot path)
[3] center_velocity (velocity of center point of robot)
[4] right_velocity (velocity of right side of robot)
[5] left_velocity (velocity of left side of robot)
[6] right_path (2d numpy array of right side path)
[7] left_path (2d numpy array of left side path)
[8] isLoop (boolean variable describing if path is possible)
[9] TRACKWIDTH (trackwidth of robot)
[10] TRACKLENGTH (tracklength of robot)
[11] NUM_POINTS (number of index points)

All of these lists/arrays with the exceptions of waypoints,isLoop,TRACKWIDTH,
and TRACKLENGTH are indexed to match each other by their list indexes.
"""
import json
import numpy as np
import math

class Trajectory():
    def __init__(self, path_x, path_y, waypoints, TRACKWIDTH, TRACKLENGTH, 
                 NUM_POINTS, MAX_VELOCITY, MAX_ACCELERATION, MAX_DECELERATION,
                 FINAL_VELOCITY, INITIAL_VELOCITY):
        self.TRACKWIDTH = TRACKWIDTH
        self.TRACKLENGTH = TRACKLENGTH
        self.NUM_POINTS = NUM_POINTS
        self.MAX_VELOCITY = MAX_VELOCITY
        self.MAX_ACCELERATION = MAX_ACCELERATION
        self.MAX_DECELERATION = MAX_DECELERATION
        self.FINAL_VELOCITY = FINAL_VELOCITY
        self.INITAL_VELOCITY = INITIAL_VELOCITY
        
        self.isLoop = False
        self.path_x = path_x
        self.path_y = path_y
        self.waypoints = waypoints
        
    def differentiate(self,x,y):
        dx = np.diff(x)
        dy = np.diff(y)
        dx = np.append(dx, dx[-1])
        dy = np.append(dy, dy[-1])
        return dx,dy
    
    def curvature(self,x,y):    
        #take first and second differential of spline
        dx,dy = self.differentiate(x,y)
        ddx,ddy = self.differentiate(dx,dy)
        radius_of_curvature = [0 for i in range (len(dx))]
        curvature = [0 for i in range (len(dx))]
        
        #calculate curvature at each point in spline
        for i in range (len(dx)):
            curvature[i] = (abs(dx[i] * ddy[i]
                            -ddx[i] * dy[i]))/(math.pow((math.pow(dx[i],2.0)
                            +math.pow(dy[i],2.0)),(1.5)))
        curvature = self.reasonable_value(curvature)
        
        for i in range(len(curvature)):
            radius_of_curvature[i] = 1/curvature[i]
        return self.reasonable_value(radius_of_curvature)
    
    def reasonable_value(self,x):
        for i in range(len(x)):
            if x[i] == 0:
                x[i] = .000001
            if x[i] > 10000:
                x[i] = 10000
            return x
        
    def curvature_constraint(self):
        #calculate the radius of curvature at every point on spline
        self.center_radius = self.curvature(self.path_x,self.path_y)
        
        #iterate through all points
        for i in range(len(self.path_x)):
            #calculate radius of outer wheel by adding half the trackwidth
            outer_path_radius = self.center_radius[i] + (self.TRACKWIDTH/2)
            if outer_path_radius < self.TRACKWIDTH:
                self.isLoop = True
            
            #calculate maximum reachable angular velocity with radius
            angular_velocity = self.MAX_VELOCITY/outer_path_radius
            
            #convert from alpha to tangential velocity
            self.center_velocity[i] = angular_velocity * self.center_radius[i]
            
    def step_length(self):
        #initialize arcLength to zero to be adjusted, arcLength = spline length
        arc_length = 0
        
        #iterate through all but the last point in even_x and even_y
        for i in range(0,len(self.path_x)-1):
            #calculations for total spline length
            a = np.array((self.path_x[i],self.path_y[i]))
            b = np.array((self.path_x[i+1],self.path_y[i+1]))
            arc_length += np.linalg.norm(a-b)
        
        #return spline length divided by number of steps for step size
        return arc_length/(len(self.path_x))
    
    def acceleration_constraint(self,step_size):
        
        #set initial value of center_velocity to starting velocity of robot
        self.center_velocity[0] = self.INITIAL_VELOCITY
        
        #iterate through all but the first point in center_velocity
        for i in range (1,len(self.center_velocity),1):
            #accel is maximum reachable velocity for that point
            accel = math.sqrt((math.pow((self.center_velocity[i-1]),2.0))+2.0*
                              self.MAX_ACCELERATION*step_size) 
            self.center_velocity[i] = min(accel,self.center_velocity[i])
            
    def deceleration_constraint(self,step_size):
        #initialize last value of center_velocity to be zero
        self.center_velocity[-1] = self.FINAL_VELOCITY
        
        #iterate through all but the first two points backwards
        for i in range (len(self.center_velocity)-2,0,-1):
            #decel is adjusted velocity based on last velocity
            decel = math.sqrt((math.pow((self.center_velocity[i+1]),2.0))+2.0*
                              self.MAX_DECELERATION*step_size)
            self.center_velocity[i] = min(decel,self.center_velocity[i])
            
    def offset_velocity(self): 
        #calculate radii of right and left paths and initialize velocity arrays
        self.right_velocity,self.left_velocity,right_radius,left_radius = self.offset_radii_and_velocity()
        
        #adjust radii to be more accurate by using radius_center
        right_radius,left_radius = self.offset_radius(right_radius,left_radius)
        
        #iterate through all but the last point in center_velocity
        for i in range (len(self.center_velocity)-1):  
            
            #calculate angular velocity at point i
            angular_velocity = self.center_velocity[i]/self.center_radius[i]
            
            #convert angular velocity to tangential velocities of each side
            self.right_velocity[i] = angular_velocity * right_radius[i]
            self.left_velocity[i] = angular_velocity * left_radius[i]
            
    #adjust right and left radii arrays to make them more accurate
    def offset_radius(self,right_radius,left_radius):
        #iterate through all points in arrays
        for i in range(len(right_radius)):
            
            #if right side is on the outside
            if right_radius[i] > left_radius[i]:
                right_radius[i] = self.center_radius[i] + self.TRACKWIDTH/2
                left_radius[i] = self.center_radius[i] - self.TRACKWIDTH/2
            
            #if left side is on the outside
            elif right_radius[i] < left_radius[i]:
                right_radius[i] = self.center_radius[i] - self.TRACKWIDTH/2
                left_radius[i] = self.center_radius[i] + self.TRACKWIDTH/2
                
            #if the path is straight
            else:
                right_radius[i] = self.center_radius[i]
                left_radius[i] = self.center_radius[i]
        return right_radius,left_radius
    
    def offset(self,coordinates,offset_distance):
        coordinates = iter(coordinates)
        x1, y1 = next(coordinates)
        points = []
        for x2, y2 in coordinates:
            try:
                tangent = (y2 - y1) / (x2 - x1)
                perpendicular_slope = -1/tangent  
            except ZeroDivisionError:
                continue
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            sign = ((perpendicular_slope > 0) == (x1 > x2)) * 2 - 1
            delta_x = sign * offset_distance / ((1 
                                                 +perpendicular_slope**2)**0.5)
            delta_y = perpendicular_slope * delta_x
            points.append((mid_x + delta_x, mid_y + delta_y))
            x1, y1 = x2, y2
        return points
    
    def offset_path(self):
        right_path = self.offset(self.path, self.TRACKWIDTH/2)
        left_path = self.offset(self.path, -1*self.TRACKWIDTH/2)
        
        #set x and y values for each path
        self.right_x = [i[0] for i in right_path]
        self.right_y = [i[1] for i in right_path]
        self.left_x = [i[0] for i in left_path]
        self.left_y = [i[1] for i in left_path]
        
        return right_path, left_path
    
    #initialize right and left velocity arrays and calculate radii
    def offset_radii_and_velocity(self):
        right_velocity = [0 for i in range (len(self.center_velocity)-1)]
        left_velocity = [0 for i in range (len(self.center_velocity)-1)]
        right_radius = self.curvature(self.right_x,self.right_y)
        left_radius = self.curvature(self.left_x,self.left_y)
        return right_velocity,left_velocity,right_radius,left_radius
    
    #calculate time elapsed for each step, used for indexing while running
    def calculate_elapsed_time(self,step_size):
        self.time_step = [0 for i in range (len(self.center_velocity))]
        for i in range (1,len(self.center_velocity),1):
            self.time_step[i] = step_size/self.center_velocity[i]
        self.time_elapsed = []
        time_elapsed = 0
        #add up all the time steps up until point i
        for time_step in self.time_step:
            time_elapsed += time_step
            self.time_elapsed.append(time_elapsed)
            
    def adjust(self):
        center_path = [[0.0,0.0] for i in range(self.num_points-1)]
        for i in range(len(self.evenly_spaced)-1):
            center_path[i][0]=self.evenly_spaced[i][0]
            center_path[i][1]=self.evenly_spaced[i][1]
        center_path = np.array(center_path)
        time_index = []
        center_velocity = []
        for i in range(len(self.time_elapsed)-1):
            time_index.append(self.time_elapsed[i])
        for i in range(len(self.center_velocity)-1):
            center_velocity.append(self.center_velocity[i])
        return center_path, time_index, center_velocity
    
    def package_data(self,right_path,left_path):
        center_path, time_index, center_velocity = self.adjust()
        trajectory = [self.waypoints,time_index,center_path,center_velocity,
                      self.right_velocity,self.left_velocity,
                      right_path,left_path,self.isLoop,self.TRACKWIDTH,
                      self.TRACKLENGTH,self.NUM_POINTS]
        with open('data.json', 'w', encoding='utf-8') as file:
            json.dumps(trajectory, file, ensure_ascii=False, indent=4)
        return trajectory
        
    def get_trajectory_data(self):
        self.curvature_constraint()
        step_size = self.step_length()
        self.acceleration_constraint(step_size)
        self.deceleration_constraint(step_size)
        right_path, left_path = self.offset_path()
        self.offset_velocity()
        self.calculate_elapsed_time()
        self.adjust()
        trajectory = self.package_data(right_path, left_path)
        return trajectory
        
=======
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 14:05:15 2020

@author: noahm
"""
import json
import numpy as np
import math

class Trajectory():
    def __init__(self, path_x, path_y, waypoints, TRACKWIDTH, TRACKLENGTH, 
                 NUM_POINTS, MAX_VELOCITY, MAX_ACCELERATION, MAX_DECELERATION,
                 FINAL_VELOCITY, INITIAL_VELOCITY):
        self.TRACKWIDTH = TRACKWIDTH
        self.TRACKLENGTH = TRACKLENGTH
        self.NUM_POINTS = NUM_POINTS
        self.MAX_VELOCITY = MAX_VELOCITY
        self.MAX_ACCELERATION = MAX_ACCELERATION
        self.MAX_DECELERATION = MAX_DECELERATION
        self.FINAL_VELOCITY = FINAL_VELOCITY
        self.INITAL_VELOCITY = INITIAL_VELOCITY
        
        self.isLoop = False
        self.path_x = path_x
        self.path_y = path_y
        self.waypoints = waypoints
        
    def differentiate(self,x,y):
        dx = np.diff(x)
        dy = np.diff(y)
        dx = np.append(dx, dx[-1])
        dy = np.append(dy, dy[-1])
        return dx,dy
    
    def curvature(self,x,y):    
        #take first and second differential of spline
        dx,dy = self.differentiate(x,y)
        ddx,ddy = self.differentiate(dx,dy)
        radius_of_curvature = [0 for i in range (len(dx))]
        curvature = [0 for i in range (len(dx))]
        
        #calculate curvature at each point in spline
        for i in range (len(dx)):
            curvature[i] = (abs(dx[i] * ddy[i]
                            -ddx[i] * dy[i]))/(math.pow((math.pow(dx[i],2.0)
                            +math.pow(dy[i],2.0)),(1.5)))
        curvature = self.reasonable_value(curvature)
        
        for i in range(len(curvature)):
            radius_of_curvature[i] = 1/curvature[i]
        return self.reasonable_value(radius_of_curvature)
    
    def reasonable_value(self,x):
        for i in range(len(x)):
            if x[i] == 0:
                x[i] = .000001
            if x[i] > 10000:
                x[i] = 10000
            return x
        
    def curvature_constraint(self):
        #calculate the radius of curvature at every point on spline
        self.center_radius = self.curvature(self.path_x,self.path_y)
        
        #iterate through all points
        for i in range(len(self.path_x)):
            #calculate radius of outer wheel by adding half the trackwidth
            outer_path_radius = self.center_radius[i] + (self.TRACKWIDTH/2)
            if outer_path_radius < self.TRACKWIDTH:
                self.isLoop = True
            
            #calculate maximum reachable angular velocity with radius
            angular_velocity = self.MAX_VELOCITY/outer_path_radius
            
            #convert from alpha to tangential velocity
            self.center_velocity[i] = angular_velocity * self.center_radius[i]
            
    def step_length(self):
        #initialize arcLength to zero to be adjusted, arcLength = spline length
        arc_length = 0
        
        #iterate through all but the last point in even_x and even_y
        for i in range(0,len(self.path_x)-1):
            #calculations for total spline length
            a = np.array((self.path_x[i],self.path_y[i]))
            b = np.array((self.path_x[i+1],self.path_y[i+1]))
            arc_length += np.linalg.norm(a-b)
        
        #return spline length divided by number of steps for step size
        return arc_length/(len(self.path_x))
    
    def acceleration_constraint(self,step_size):
        
        #set initial value of center_velocity to starting velocity of robot
        self.center_velocity[0] = self.INITIAL_VELOCITY
        
        #iterate through all but the first point in center_velocity
        for i in range (1,len(self.center_velocity),1):
            #accel is maximum reachable velocity for that point
            accel = math.sqrt((math.pow((self.center_velocity[i-1]),2.0))+2.0*
                              self.MAX_ACCELERATION*step_size) 
            self.center_velocity[i] = min(accel,self.center_velocity[i])
            
    def deceleration_constraint(self,step_size):
        #initialize last value of center_velocity to be zero
        self.center_velocity[-1] = self.FINAL_VELOCITY
        
        #iterate through all but the first two points backwards
        for i in range (len(self.center_velocity)-2,0,-1):
            #decel is adjusted velocity based on last velocity
            decel = math.sqrt((math.pow((self.center_velocity[i+1]),2.0))+2.0*
                              self.MAX_DECELERATION*step_size)
            self.center_velocity[i] = min(decel,self.center_velocity[i])
            
    def offset_velocity(self): 
        #calculate radii of right and left paths and initialize velocity arrays
        self.right_velocity,self.left_velocity,right_radius,left_radius = self.offset_radii_and_velocity()
        
        #adjust radii to be more accurate by using radius_center
        right_radius,left_radius = self.offset_radius(right_radius,left_radius)
        
        #iterate through all but the last point in center_velocity
        for i in range (len(self.center_velocity)-1):  
            
            #calculate angular velocity at point i
            angular_velocity = self.center_velocity[i]/self.center_radius[i]
            
            #convert angular velocity to tangential velocities of each side
            self.right_velocity[i] = angular_velocity * right_radius[i]
            self.left_velocity[i] = angular_velocity * left_radius[i]
            
    #adjust right and left radii arrays to make them more accurate
    def offset_radius(self,right_radius,left_radius):
        #iterate through all points in arrays
        for i in range(len(right_radius)):
            
            #if right side is on the outside
            if right_radius[i] > left_radius[i]:
                right_radius[i] = self.center_radius[i] + self.TRACKWIDTH/2
                left_radius[i] = self.center_radius[i] - self.TRACKWIDTH/2
            
            #if left side is on the outside
            elif right_radius[i] < left_radius[i]:
                right_radius[i] = self.center_radius[i] - self.TRACKWIDTH/2
                left_radius[i] = self.center_radius[i] + self.TRACKWIDTH/2
                
            #if the path is straight
            else:
                right_radius[i] = self.center_radius[i]
                left_radius[i] = self.center_radius[i]
        return right_radius,left_radius
    
    def offset(self,coordinates,offset_distance):
        coordinates = iter(coordinates)
        x1, y1 = next(coordinates)
        points = []
        for x2, y2 in coordinates:
            try:
                tangent = (y2 - y1) / (x2 - x1)
                perpendicular_slope = -1/tangent  
            except ZeroDivisionError:
                continue
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            sign = ((perpendicular_slope > 0) == (x1 > x2)) * 2 - 1
            delta_x = sign * offset_distance / ((1 
                                                 +perpendicular_slope**2)**0.5)
            delta_y = perpendicular_slope * delta_x
            points.append((mid_x + delta_x, mid_y + delta_y))
            x1, y1 = x2, y2
        return points
    
    def offset_path(self):
        right_path = self.offset(self.path, self.TRACKWIDTH/2)
        left_path = self.offset(self.path, -1*self.TRACKWIDTH/2)
        
        #set x and y values for each path
        self.right_x = [i[0] for i in right_path]
        self.right_y = [i[1] for i in right_path]
        self.left_x = [i[0] for i in left_path]
        self.left_y = [i[1] for i in left_path]
        
        return right_path, left_path
    
    #initialize right and left velocity arrays and calculate radii
    def offset_radii_and_velocity(self):
        right_velocity = [0 for i in range (len(self.center_velocity)-1)]
        left_velocity = [0 for i in range (len(self.center_velocity)-1)]
        right_radius = self.curvature(self.right_x,self.right_y)
        left_radius = self.curvature(self.left_x,self.left_y)
        return right_velocity,left_velocity,right_radius,left_radius
    
    #calculate time elapsed for each step, used for indexing while running
    def calculate_elapsed_time(self,step_size):
        self.time_step = [0 for i in range (len(self.center_velocity))]
        for i in range (1,len(self.center_velocity),1):
            self.time_step[i] = step_size/self.center_velocity[i]
        self.time_elapsed = []
        time_elapsed = 0
        #add up all the time steps up until point i
        for time_step in self.time_step:
            time_elapsed += time_step
            self.time_elapsed.append(time_elapsed)
            
    def adjust(self):
        center_path = [[0.0,0.0] for i in range(self.num_points-1)]
        for i in range(len(self.evenly_spaced)-1):
            center_path[i][0]=self.evenly_spaced[i][0]
            center_path[i][1]=self.evenly_spaced[i][1]
        center_path = np.array(center_path)
        time_index = []
        center_velocity = []
        for i in range(len(self.time_elapsed)-1):
            time_index.append(self.time_elapsed[i])
        for i in range(len(self.center_velocity)-1):
            center_velocity.append(self.center_velocity[i])
        return center_path, time_index, center_velocity
    
    def package_data(self,right_path,left_path):
        center_path, time_index, center_velocity = self.adjust()
        trajectory = [self.waypoints,time_index,center_path,center_velocity,
                      self.right_velocity,self.left_velocity,
                      right_path,left_path,self.isLoop,self.TRACKWIDTH,
                      self.TRACKLENGTH,self.NUM_POINTS]
        with open('data.json', 'w', encoding='utf-8') as file:
            json.dumps(trajectory, file, ensure_ascii=False, indent=4)
        return trajectory
        
    def get_trajectory_data(self):
        self.curvature_constraint()
        step_size = self.step_length()
        self.acceleration_constraint(step_size)
        self.deceleration_constraint(step_size)
        right_path, left_path = self.offset_path()
        self.offset_velocity()
        self.calculate_elapsed_time()
        self.adjust()
        trajectory = self.package_data(right_path, left_path)
        return trajectory
        
>>>>>>> 4e2a8f6f32eea0cea4fa2833548516b36856f9f8
        