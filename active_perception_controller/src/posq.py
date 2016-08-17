#!/usr/bin/env python

import scipy as sp
import numpy as np
import pdb
import matplotlib.pyplot as plt 

def posq_cl(rho,alpha,phi,k_p = 1.,k_v = 0.3,k_a=4.2,k_f = -0.7):
	v = k_p*np.tanh(k_v*rho)
	w = k_a*alpha + k_f*phi
	return v,w
def angle_map(angle):
	if angle >2*np.pi:
		print "WARNING"
	if angle <-2*np.pi:
		print "WARNING"
	if angle >np.pi:
	    return angle - 2*np.pi
	elif angle < -np.pi:
	    return angle+2*np.pi
	else: 
		return angle
def get_raf(x1,x2):
	#inputs are arrays of x,y,theta
	d = x2[:2] - x1[:2]
	rho = np.linalg.norm(x1[:2]-x2[:2])
	#print x1,x2
	phi = angle_map(x2[2]-x1[2])
	#phi = angle_map(x1[2] - x2[2])
	ang = np.arctan2(d[1],d[0])
	alpha = angle_map(ang-x1[2])
	# alpha = angle_map(x1[2] - ang)
	return rho,alpha,phi
def kinematics_forward(x1,v,w,dt=0.01):
	# forward kinematics for a wheeles robot
	out = np.zeros(3)
	out[0] = x1[0] + dt*v*np.cos(x1[2])
	out[1] = x1[1] +dt*v*np.sin(x1[2])
	out[2] = angle_map(x1[2] + dt*w)
	#print out[2]
	return out 

def dynamics(alpha,rho,v,w):
	rho_dot = -np.cos(alpha)*v
	alpha_dot = (np.sin(alpha)/rho)*v - w
	phi_dot = -w
	return rho_dot,alpha_dot,phi_dot

def simulate(x1,x2,steps = None,w_saturation=1.5,v_min = 0.3,eps = 0.1,dt = 0.1,return_steps = False):
	eps_phi =0.1
	path = np.array([x1])
	x = []
	rho,alpha,phi  = get_raf(x1,x2)
	new_po = x1
	reached = False
	#path.append(new_po)

	count =0
	for i in xrange(steps):
		# if i == 20:
		# 	pdb.set_trace()
		v,w = posq_cl(rho,alpha,phi)
		if np.abs(w)>w_saturation:
			w= np.sign(w)*w_saturation
		if v < v_min:
			v = v_min
		#x.append(kinematics_forward(new_po,v,w,dt=dt))
		new_po = kinematics_forward(new_po,v,w,dt=dt)
		rho,alpha,phi  = get_raf(new_po,x2)
		#print "RHO",rho,eps,count
		path = np.append(path,np.array([new_po]),axis=0)
		if (rho<eps and np.abs(phi)<eps_phi):
			reached=True
			break
	if return_steps ==True:
		return np.array(path),reached,i
	else:
		return np.array(path),reached
if __name__ =="__main__":
	x1 = np.array([0,0,0])
	x2 = np.array([0,5,0.0])
	path,reached = simulate(x1,x2,steps = 100)
	plt.scatter(path[:,0],path[:,1])
	plt.show()
