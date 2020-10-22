<<<<<<< HEAD
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 10 20:25:07 2020

@author: Noah Bamberger

This program will generate a spline that will intersect preset waypoints at
preset headings, or "handles". It will generate (num_points) discrete 
coordinates along the spline. These points will be evenly spaced by distance.
"""

import numpy as np
from scipy import interpolate
import interparcv2


class Path():
    def __init__(self, waypoints, headings, num_points):
        self.RESOLUTION = float(.1)
        self.TRACKWIDTH = 28
        
        self.waypoints = np.asarray(waypoints)
        self.headings = np.asarray(headings)
        self.num_points = num_points
        
        
    def sample(self, scale):
        self.num_points, self.dim = self.setpoints.shape
        
        #multiply length of handles by scale
        self.headings = np.dot(self.headings, scale*np.eye(2))
        point_sequence_length = self.length()
        
        #nSamples becomes number of sample points along spline
        self.num_samples = int(point_sequence_length/ self.RESOLUTION)

        #evenly space points along the spline parametrically           
        self.sorted_value,step = np.linspace(0,point_sequence_length
                                             ,self.num_samples,retstep=True) 
    
        
    def length(self):
        difference_along_axis = np.diff(self.waypoints, axis=0)
        dist_between_points = np.linalg.norm(difference_along_axis, axis=1)              
        cumsum_along_segments = np.cumsum(dist_between_points)                            
        self.cumsum = np.hstack([[0],cumsum_along_segments])                       
        return cumsum_along_segments[-1]   
    
    
    def format_data(self):
        assert(len(self.waypoints) == len(self.headings))
        data = np.empty([self.num_points,self.dim], dtype=object)
        for index,point in enumerate(self.waypoints):
            data[index,:] = self.format_data(index,point)
        return data
    
    
    def fuse_data(self,index,point):
        tangent = self.headings[index]
        assert(tangent is None or len(tangent)==self.dim)
        fuse = list(zip(point,tangent) if tangent is not None else zip(point,))
        return fuse
    
    
    def interpolate(self):
        data = self.format_data()
        samples = np.zeros([self.num_samples, self.dim])
        #interpolate cubic spline
        for i in range(self.dim):
            polynomial = interpolate.BPoly.from_derivatives(self.cumsum,
                                                            data[:,i])
            samples[:,i] = polynomial(self.s)
        #return x and y coordinate lists for interpolated points
        return samples[:,0],samples[:,1]
    
       
    def evenly_space(self):
        #get interpolated values from spline, in other words generate spline
        interpolated_x,interpolated_y = self.interpolate()
        
        #evely space points along spline by distance
        self.evenly_spaced = interparcv2.interparc(self.num_points,
                                                   interpolated_x,
                                                   interpolated_y,"linear")
        
        #extract x and y values from evenly_spaced, makes life easier
        evenly_spaced_x = self.evenly_spaced[:,0]
        evenly_spaced_y = self.evenly_spaced[:,1]
        
        return evenly_spaced_x, evenly_spaced_y
=======
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 10 20:25:07 2020

@author: noahm
"""

import numpy as np
from scipy import interpolate
import interparcv2


class Path():
    def __init__(self, waypoints, headings, num_points):
        self.RESOLUTION = float(.1)
        self.TRACKWIDTH = 28
        
        self.waypoints = np.asarray(waypoints)
        self.headings = np.asarray(headings)
        self.num_points = num_points
        
        self.center_velocity = [0 for i in range(self.num_points)]
        
        
    def sample(self, scale):
        #set nPoints and dim variables for use later
        self.num_points, self.dim = self.setpoints.shape
        
        #adjust headings array with scale
        self.headings = np.dot(self.headings, scale*np.eye(2))
        
        #l becomes last element of d array
        point_sequence_length = self.length()
        
        #nSamples becomes number of sample points along spline
        self.num_samples = int(point_sequence_length/ self.RESOLUTION)

        #evenly space points along the spline parametrically           
        self.sorted_value,step = np.linspace(0,point_sequence_length
                                             ,self.num_samples,retstep=True) 
    
        
    def length(self):
        difference_along_axis = np.diff(self.waypoints, axis=0)
        dist_between_points = np.linalg.norm(difference_along_axis, axis=1)              
        cumsum_along_segments = np.cumsum(dist_between_points)                            
        self.cumsum = np.hstack([[0],cumsum_along_segments])                       
        return cumsum_along_segments[-1]   
    
    
    def format_data(self):
        assert(len(self.waypoints) == len(self.headings))
        data = np.empty([self.num_points,self.dim], dtype=object)
        for index,point in enumerate(self.waypoints):
            data[index,:] = self.format_data(index,point)
        return data
    
    
    def fuse_data(self,index,point):
        tangent = self.headings[index]
        assert(tangent is None or len(tangent)==self.dim)
        fuse = list(zip(point,tangent) if tangent is not None else zip(point,))
        return fuse
    
    
    def interpolate(self):
        data = self.format_data()
        samples = np.zeros([self.num_samples, self.dim])
        #interpolate cubic spline
        for i in range(self.dim):
            polynomial = interpolate.BPoly.from_derivatives(self.cumsum,
                                                            data[:,i])
            samples[:,i] = polynomial(self.s)
        #return x and y coordinate lists for interpolated points
        return samples[:,0],samples[:,1]
    
       
    def evenly_space(self):
        #get interpolated values from spline, in other words generate spline
        interpolated_x,interpolated_y = self.interpolate()
        
        #evely space points along spline by distance
        self.evenly_spaced = interparcv2.interparc(self.num_points,
                                                   interpolated_x,
                                                   interpolated_y,"linear")
        
        #extract x and y values from evenly_spaced, makes life easier
        evenly_spaced_x = self.evenly_spaced[:,0]
        evenly_spaced_y = self.evenly_spaced[:,1]
        
        return evenly_spaced_x, evenly_spaced_y
>>>>>>> 4e2a8f6f32eea0cea4fa2833548516b36856f9f8
    