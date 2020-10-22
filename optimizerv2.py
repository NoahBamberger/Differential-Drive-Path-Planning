<<<<<<< HEAD
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 23:33:46 2020

@author: Noah Bamberger

This program will generate an optimal path along preset waypoints using the
path and trajectory files from this library.
"""

from trajectory import Trajectory
from path import Path

class OptimizerV2():
    def __init__(self):
        self.scales = []
        for i in range(1,51,1):
            self.scales.append((1/50)*i)
    def optimize_partial(self, waypoints,tangents,INITIAL_VELOCITY,TRACKWIDTH, 
                                    TRACKLENGTH,NUM_POINTS,MAX_VELOCITY, 
                                    MAX_ACCELERATION, 
                                    MAX_DECELERATION,FINAL_VELOCITY):
        for i in self.scales:
            path = Path()
            path_x,path_y = path.evenly_space()
            trajectory = Trajectory(path_x, path_y, waypoints, TRACKWIDTH, 
                                    TRACKLENGTH, NUM_POINTS, MAX_VELOCITY, 
                                    MAX_ACCELERATION, 
                                    MAX_DECELERATION,FINAL_VELOCITY, 
                                    INITIAL_VELOCITY)
            trajectory_data = trajectory.get_trajectory_data()
            if trajectory.isLoop == False:
                print(i)
                return i, trajectory_data
    def segment_waypoints(self, points, tangents):
        segmented_points = []
        segmented_tangents = []
        for i in range(0,len(points)- 1,1):
            segmented_points.append([points[i], points[i+1]])
            segmented_tangents.append([tangents[i],tangents[i+1]])
        return (segmented_points,segmented_tangents)
        
    def optimize_total(self, points, tangents):
        segmented_points, segmented_tangents = self.segment_waypoints(points, 
                                                                      tangents)
        last_velocity = 0
        multiple_path = []
        for segment in range(len(segmented_points)):
             i,trajectory = self.optimize_partial(segmented_points[segment],
                                                  segmented_tangents[segment],
                                                  last_velocity)
             path = trajectory[2]
             for i in path:
                 multiple_path.append(i)
             last_velocity = trajectory[3][-1]
        return multiple_path
    
    
=======
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 18 23:33:46 2020

@author: noahm
"""

from trajectory import Trajectory
from path import Path

class OptimizerV2():
    def __init__(self):
        self.scales = []
        for i in range(1,51,1):
            self.scales.append((1/50)*i)
    def optimize_partial(self, waypoints,tangents,INITIAL_VELOCITY,TRACKWIDTH, 
                                    TRACKLENGTH,NUM_POINTS,MAX_VELOCITY, 
                                    MAX_ACCELERATION, 
                                    MAX_DECELERATION,FINAL_VELOCITY):
        for i in self.scales:
            path = Path()
            path_x,path_y = path.evenly_space()
            trajectory = Trajectory(path_x, path_y, waypoints, TRACKWIDTH, 
                                    TRACKLENGTH, NUM_POINTS, MAX_VELOCITY, 
                                    MAX_ACCELERATION, 
                                    MAX_DECELERATION,FINAL_VELOCITY, 
                                    INITIAL_VELOCITY)
            trajectory_data = trajectory.get_trajectory_data()
            if trajectory.isLoop == False:
                print(i)
                return i, trajectory_data
    def segment_waypoints(self, points, tangents):
        segmented_points = []
        segmented_tangents = []
        for i in range(0,len(points)- 1,1):
            segmented_points.append([points[i], points[i+1]])
            segmented_tangents.append([tangents[i],tangents[i+1]])
        return (segmented_points,segmented_tangents)
        
    def optimize_total(self, points, tangents):
        segmented_points, segmented_tangents = self.segment_waypoints(points, 
                                                                      tangents)
        last_velocity = 0
        multiple_path = []
        for segment in range(len(segmented_points)):
             i,trajectory = self.optimize_partial(segmented_points[segment],
                                                  segmented_tangents[segment],
                                                  last_velocity)
             path = trajectory[2]
             for i in path:
                 multiple_path.append(i)
             last_velocity = trajectory[3][-1]
        return multiple_path
    
    
>>>>>>> 4e2a8f6f32eea0cea4fa2833548516b36856f9f8
    