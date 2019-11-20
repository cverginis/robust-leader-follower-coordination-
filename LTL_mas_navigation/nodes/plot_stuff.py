#!/usr/bin/env python

import numpy as np
import rospy
import std_msgs.msg
import nav_msgs.msg
import mav_msgs.msg
import geometry_msgs.msg
import rosgraph_msgs.msg
import std_srvs.srv
import utilities.utility_functions as utility_functions
import tf.transformations
from dynamic_reconfigure.server import Server
from numpy import linalg 
import rosbag
import time
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.patches as mpatches
from matplotlib import animation
from numpy import random

def create_sphere(c,r,m):
	phi, theta = np.mgrid[0.0:np.pi:m, 0.0:2.0*np.pi:m]
	x = r*np.sin(phi)*np.cos(theta) + c[0]
	y = r*np.sin(phi)*np.sin(theta) + c[1]
	z = r*np.cos(phi) + c[2]
	return x,y,z


adapt_plot_bool = False
betas_plot_bool = False
u_plot_bool = False
leader_error_plot_bool = False
record_video_bool = False

print 'Reading bag files..'
time_bag = rosbag.Bag('time.bag')
ep_bag = rosbag.Bag('firefly_leader_error.bag')
bags = [rosbag.Bag('firefly_1_pos.bag'), rosbag.Bag('firefly_2_pos.bag'), rosbag.Bag('firefly_3_pos.bag'), rosbag.Bag('firefly_4_pos.bag'), rosbag.Bag('firefly_5_pos.bag'), rosbag.Bag('firefly_6_pos.bag')]
theta_hat_bags = [rosbag.Bag('firefly_1_theta_hat.bag'), rosbag.Bag('firefly_2_theta_hat.bag'), rosbag.Bag('firefly_3_theta_hat.bag'), rosbag.Bag('firefly_4_theta_hat.bag'), rosbag.Bag('firefly_5_theta_hat.bag'), rosbag.Bag('firefly_6_theta_hat.bag')]
d_hat_bags = [rosbag.Bag('firefly_1_d_hat.bag'), rosbag.Bag('firefly_2_d_hat.bag'), rosbag.Bag('firefly_3_d_hat.bag'), rosbag.Bag('firefly_4_d_hat.bag'), rosbag.Bag('firefly_5_d_hat.bag'), rosbag.Bag('firefly_6_d_hat.bag')]
f_hat_bags = [rosbag.Bag('firefly_1_f_hat.bag'), rosbag.Bag('firefly_2_f_hat.bag'), rosbag.Bag('firefly_3_f_hat.bag'), rosbag.Bag('firefly_4_f_hat.bag'), rosbag.Bag('firefly_5_f_hat.bag'), rosbag.Bag('firefly_6_f_hat.bag')]
input_bags = [rosbag.Bag('firefly_1_inputs.bag'), rosbag.Bag('firefly_2_inputs.bag'), rosbag.Bag('firefly_3_inputs.bag'), rosbag.Bag('firefly_4_inputs.bag'), rosbag.Bag('firefly_5_inputs.bag'), rosbag.Bag('firefly_6_inputs.bag')]


pos_array_cur = [[] for i in range(6)]
input_array_cur = [[] for i in range(6)]
theta_hat_array = [[] for i in range(6)]
d_hat_array = [[] for i in range(6)]
f_hat_array = [[] for i in range(6)]
time_l = []
ep_leader = []
time_st = []
dt = 50* 10**(-3)	#100ms
final_time = 277

idx = 0
for msg in ep_bag.read_messages():	 
	cur_time_st = msg[2].to_sec()		
	if cur_time_st > idx and cur_time_st < final_time:	
		ep_leader.append(msg[1].data)
		last_time = cur_time_st
		idx += dt


idx = 0
for msg in time_bag.read_messages():			
	cur_time_st = msg[2].to_sec()
	if cur_time_st > idx and cur_time_st < final_time:	
		time_l.append(msg[1].data)
		last_time = cur_time_st
		idx += dt

if adapt_plot_bool:
	for i in range(len(theta_hat_bags)):
		bag = theta_hat_bags[i]	
		idx = 0
		for msg in bag.read_messages():		
			cur_time_st = msg[2].to_sec()
			#print cur_time_st		
			if cur_time_st > idx and cur_time_st < final_time:						
				theta_hat_array[i].append(msg[1].data)
				idx += dt

	for i in range(len(d_hat_bags)):
		bag = d_hat_bags[i]	
		idx = 0
		for msg in bag.read_messages():		
			cur_time_st = msg[2].to_sec()
			#print cur_time_st		
			if cur_time_st > idx and cur_time_st < final_time:						
				d_hat_array[i].append(msg[1].data)
				idx += dt

	for i in range(len(f_hat_bags)):
		bag = f_hat_bags[i]	
		idx = 0
		for msg in bag.read_messages():		
			cur_time_st = msg[2].to_sec()
			#print cur_time_st		
			if cur_time_st > idx and cur_time_st < final_time:						
				f_hat_array[i].append(msg[1].data)
				idx += dt


for i in range(len(bags)):
	bag = bags[i]	
	idx = 0
	for msg in bag.read_messages():		
		cur_time_st = msg[2].to_sec()
		#print cur_time_st		
		if cur_time_st > idx and cur_time_st < final_time:			
			pos_cur = np.array([msg[1].pose.pose.position.x, msg[1].pose.pose.position.y, msg[1].pose.pose.position.z])
			pos_array_cur[i].append(pos_cur)
			idx += dt
		
for i in range(len(input_bags)):
	bag = input_bags[i]	
	idx = 0
	for msg in bag.read_messages():		
		cur_time_st = msg[2].to_sec()
		#print cur_time_st		
		if cur_time_st > idx and cur_time_st < final_time:			
			input_cur = np.array([msg[1].thrust.x, msg[1].thrust.y, msg[1].thrust.z])
			input_array_cur[i].append(input_cur)
			idx += dt


for bag in bags:
	bag.close()


print 'Done!'

print len(ep_leader), len(time_l), len(pos_array_cur[0]), len(pos_array_cur[1]), len(pos_array_cur[2]), len(pos_array_cur[3]), len(pos_array_cur[4]), len(pos_array_cur[5])
print len(theta_hat_array[0]), len(theta_hat_array[1]), len(theta_hat_array[2]), len(theta_hat_array[3]), len(theta_hat_array[4]), len(theta_hat_array[5])
print len(d_hat_array[0]), len(d_hat_array[1]), len(d_hat_array[2]), len(d_hat_array[3]), len(d_hat_array[4]), len(d_hat_array[5])
#plt.plot(time_l,ep_leader)
#plt.show()

lengt = len(ep_leader)
E_con = np.array([ [0,1], [0,2], [0,3], [2,3], [2,4], [4,5], [1,5]])


if betas_plot_bool:
	r = 0.35
	d_con = 3
	beta_bound = 10**4
	con_distance_meas = 0 #inter agent distance where we start taking con. into account (greater than)
	con_offset = d_con**2 - con_distance_meas**2
	col_distance_meas = d_con #inter agent distance where we start taking coll. into account (less than)
	col_offset = col_distance_meas**2 - 4*r**2


	E_full = np.array([ [0,1],[0,2], [0,3], [0,4], [0,5], [1,2], [1,3], [1,4], [1,5], [2,3], [2,4], [2,5], [3,4], [3,5], [4,5] ])


	beta_cols = []
	A = np.array([[col_offset**5,col_offset**4, col_offset**3],[5*col_offset**4, 4*col_offset**3, 3*col_offset**2], [20*col_offset**3, 12*col_offset**2, 6*col_offset]])
	B = np.array([beta_bound,0,0])
	coeff = np.dot(np.linalg.inv(A),B)
	for m in range(15):
		m_1 = E_full[m][0]
		m_2 = E_full[m][1]

		cur_beta = []
		for t in range(len(pos_array_cur[0])):
			iota = np.linalg.norm(pos_array_cur[m_1][t] - pos_array_cur[m_2][t])**2 - 4*r**2
			if iota <= col_offset:
				cur_beta.append(coeff[0]*iota**5 + coeff[1]*iota**4 + coeff[2]*iota**3)
			else:
				cur_beta.append(beta_bound)
		beta_cols.append(cur_beta)


	beta_cons = []
	A = np.array([[con_offset**5,con_offset**4, con_offset**3],[5*con_offset**4, 4*con_offset**3, 3*con_offset**2], [20*con_offset**3, 12*con_offset**2, 6*con_offset]])
	B = np.array([beta_bound,0,0])
	coeff = np.dot(np.linalg.inv(A),B)		

	for m in range(7):
		m_1 = E_con[m][0]
		m_2 = E_con[m][1]

		cur_beta = []
		for t in range(len(pos_array_cur[0])):
			eta = d_con**2 - np.linalg.norm( pos_array_cur[m_1][t]  - pos_array_cur[m_2][t])**2     
			if eta <= con_offset:
				cur_beta.append(coeff[0]*eta**5 + coeff[1]*eta**4 + coeff[2]*eta**3)
			else:
				cur_beta.append(beta_bound)
		beta_cons.append(cur_beta)

	beta_mul_total = []
	for t in range(len(pos_array_cur[0])):	
		beta_mul = 1
		for beta_col in beta_cols:	
			beta_mul*= 1/beta_col[t]
		for beta_con in beta_cons:	
			beta_mul*= 1/beta_con[t]
	    
		beta_mul_total.append(beta_mul)

	plt.rc('text', usetex=True)
	plt.rc('font', family='serif')

	f1 = plt.figure()
	plt.plot(time_l,beta_mul_total, linewidth=2)
	plt.grid(True)
	#plt.plot(time_l, np.ones(len(time_l)),'k--')
	axes = plt.gca()
	#print axes.get_ylim()
	plt.xlim(0,final_time)
	plt.axis([0, final_time, -0.1e-74, 1.5e-74])
	plt.xlabel(r"$t \ [s]$", fontsize = 25)
	plt.ylabel(r"$ \prod \frac{1}{\beta_{c,m}} \prod \frac{1}{\beta_{n,l}}, \ m\in \bar{\mathcal{M}}, l\in\mathcal{M}_0 $", fontsize = 30)
	plt.tick_params(labelsize = 20)
	plt.tight_layout()
	plt.savefig('/home/cverginis/catkin_ws/src/LTL_mas_navigation/nodes/betas.eps', format='eps', dpi=500)	# 210 - 300

# this is another inset axes over the main axes
#a = plt.axes([0.2, 0.15, .5, .1])
#plt.plot(time_l,beta_mul_total,linewidth=2)
#plt.axis([0, final_time, 0, 1e+74])
#plt.yticks([0, 5e+73, 1e+74])


if adapt_plot_bool:

	theta_hat_mul_total = []
	for t in range(len(theta_hat_array[0])):
		theta_hat_mul = 1
		for theta_hat in theta_hat_array:
			theta_hat_mul*= theta_hat[t]
		theta_hat_mul_total.append(theta_hat_mul)

	d_hat_mul_total = []
	for t in range(len(d_hat_array[0])):
		d_hat_mul = 1
		for d_hat in d_hat_array:
			d_hat_mul*= d_hat[t]
		d_hat_mul_total.append(d_hat_mul)

	f_hat_mul_total = []
	for t in range(len(f_hat_array[0])):
		f_hat_mul = 1
		for f_hat in f_hat_array:
			f_hat_mul*= f_hat[t]
		f_hat_mul_total.append(f_hat_mul)

	plt.rc('text', usetex=True)
	plt.rc('font', family='serif')

	f2 = plt.figure()
	plt.grid(True)
	plt.plot(time_l,theta_hat_mul_total, 'b', linewidth=2, label = r"$\prod \| \hat{\theta}_i \| $")
	plt.plot(time_l,d_hat_mul_total, 'r', linewidth=2, label = r"$\prod  \hat{d}_{b_i}$")
	plt.plot(time_l,f_hat_mul_total, 'g', linewidth=2, label = r"$\prod  \hat{f}_{b_i}$")
	plt.xlim(0,final_time)
	plt.xlabel(r"$t \ [s]$", fontsize = 20)
	plt.ylabel('Adaptation signals', fontsize = 24)
	plt.legend(loc='center right', fontsize = 20)
	plt.tick_params(labelsize = 20)

	a = plt.axes([0.25, 0.2, .5, .1])
	plt.axis([0, final_time, 0, 2e-8])
	plt.xlim(0,final_time)
	plt.yticks([0, 1e-8, 2e-8])
	plt.grid(b=None, which='major')
	plt.plot(time_l,theta_hat_mul_total, 'b', linewidth=2)
	plt.plot(time_l,d_hat_mul_total, 'r', linewidth=2)
	plt.plot(time_l,f_hat_mul_total, 'g', linewidth=2)
	plt.tick_params(labelsize = 20)	
	plt.savefig('/home/cverginis/catkin_ws/src/LTL_mas_navigation/nodes/adaptation_laws.eps', format='eps', dpi=500)	# 210 - 300



f3 = plt.figure()
ax = p3.Axes3D(f3)
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
ax.grid(b=None, which='major')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_xlim(-3,5)
ax.set_ylim(-3,5)
ax.set_zlim(0,6)
regions = np.array([[0, 0, 5], [4, 5, 3],[-2, 4, 2],[3, -2, 3]])
x,y,z = create_sphere(regions[0],0.05,10j)
ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='r', alpha=0.5, linewidth=0)
x,y,z = create_sphere(regions[1],0.05,10j)
ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='g', alpha=0.5, linewidth=0)
x,y,z = create_sphere(regions[2],0.05,10j)
ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='b', alpha=0.5, linewidth=0)
x,y,z = create_sphere(regions[3],0.05,10j)
ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='k', alpha=0.5, linewidth=0)
ax.view_init(27, -26)



ax.text(0,0,5.1,r"$x_{d,1}$",fontsize=15,color='k',weight='bold')
ax.text(4,5,3.1,r"$x_{d,2}$",fontsize=15,color='k',weight='bold')
ax.text(-2,4,2.1,r"$x_{d,3}$",fontsize=15,color='k',weight='bold')
ax.text(3,-2,3.1,r"$x_{d,4}$",fontsize=15,color='k',weight='bold')

#ax.color = ((0.1,0.1,0.1))

blue_patch = mpatches.Patch(color='blue', label='UAV 1')
red_patch = mpatches.Patch(color='red', label='UAV 2')
green_patch = mpatches.Patch(color='green', label='UAV 3')
yellow_patch = mpatches.Patch(color='yellow', label='UAV 4')
black_patch = mpatches.Patch(color='black', label='UAV 5')
white_patch = mpatches.Patch(color=[0.75,0.75,0.75], label='UAV 6')

llocation = (0.13,0.55)
plt.legend(handles=[blue_patch,red_patch,green_patch,yellow_patch,black_patch,white_patch], loc=llocation, bbox_to_anchor=[0.15,0.55], prop={'size':12})


j = 50
#maximum = 1000	    #1000	#2500	#4000	#5540
#initial = 1000		#1		#1000	#2500	#4000
limits = [[1000,1],[2500,1000],[4000,2500], [5540,4000]]
colors = ['b','r', 'g', 'y', 'k', [0.75,0.75,0.75]]

k=3 	#k = 0,...,3

maximum = limits[k][0]
initial = limits[k][1]
path_to_save = '/home/cverginis/catkin_ws/src/LTL_mas_navigation/nodes/matplot_solution_' + str(k) + '.jpg'
for i in range(initial,maximum,j):

	for ag in range(0,6):
		x_left = np.linspace(-0.1,0.1,5) + pos_array_cur[ag][i][0]
		y_left = np.linspace(0.1,-0.1,5) + pos_array_cur[ag][i][1]
		x_right = x_left
		y_right = np.linspace(-0.1,0.1,5) + pos_array_cur[ag][i][1]

		x_sp, y_sp, z_sp = create_sphere(np.array([pos_array_cur[ag][i][0],pos_array_cur[ag][i][1],pos_array_cur[ag][i][2]]), 0.35, 10j )
		ax.plot_surface(x_sp, y_sp, z_sp,rstride=1, cstride=1, color=colors[ag], alpha=0.05, linewidth=0)
		ax.plot(x_left,y_left,pos_array_cur[ag][i][2]*np.ones(5),color=colors[ag])
		ax.plot(x_right,y_right,pos_array_cur[ag][i][2]*np.ones(5),color=colors[ag])

#connectivity lines
for m in range(7):
		m_1 = E_con[m][0]
		m_2 = E_con[m][1]

		ax.plot([pos_array_cur[m_1][i][0],pos_array_cur[m_2][i][0]], [pos_array_cur[m_1][i][1],pos_array_cur[m_2][i][1]], [pos_array_cur[m_1][i][2],pos_array_cur[m_2][i][2]],'k' )

plt.savefig(path_to_save, format='jpg')


ind = 231
labels = [r"$u_1(t)$",r"$u_2(t)$",r"$u_3(t)$",r"$u_4(t)$",r"$u_5(t)$",r"$u_6(t)$"]
if u_plot_bool:
	#fig, ax = 
	f4 = plt.figure()
	for ag in range(len(input_array_cur)):
		plt.subplot(ind)
		plt.plot(time_l,input_array_cur[ag],linewidth=2.0)
		plt.grid(True)
		plt.title(labels[ag])
		#plt.plot(time_l,input_array_cur[ag], label = r"$y-axis$")
		#plt.plot(time_l,input_array_cur[ag], label = r"$z-axis$")					
		if ind >= 234:
			plt.xlabel(r"$t \ [s]$",fontsize=20)
		#if ind != 231 and ind != 234:
		#	plt.yticks([])
		if ind == 231:
				plt.ylabel(r"$ u_i(t), i=1,2,3 $", fontsize=25)
		if ind == 234:
				plt.ylabel(r"$ u_i(t), i=4,5,6 $",fontsize=25)
		
		plt.xlim(0,final_time)
		plt.ylim(-5,20)
		plt.legend(('x','y','z'),loc='center right')
		

		ind+=1
		#plt.subplot(232)
		#plt.plot(time_l,input_array_cur[1])
	plt.savefig('/home/cverginis/catkin_ws/src/LTL_mas_navigation/nodes/control_inputs.pdf', format='pdf')	# 210 - 300


if leader_error_plot_bool:
	f5 = plt.figure()
	plt.plot(time_l,ep_leader,linewidth=2.0)
	plt.grid(True)
	plt.xlabel(r"$t \ [s]$",fontsize=20)
	plt.xlim(0,final_time)
	plt.ylabel(r"$ \| s_e(t) \| + \| e_{v_1}(t)\| $", fontsize=35)
	plt.tick_params(labelsize = 20)
	plt.savefig('/home/cverginis/catkin_ws/src/LTL_mas_navigation/nodes/e_leader.pdf', format='pdf')	# 210 - 300
	

#record_video_bool = False
#print [pos_array_cur[0][i][0] for i in range(0,lengt)]
#print np.asarray([pos_array_cur[0][i][0] for i in range(0,lengt)])

#x_left_cur = -0.12 + np.asarray([pos_array_cur[ag][i][0] for i in range(0,final_time)])
if record_video_bool:
	fig = plt.figure()
	ax = p3.Axes3D(fig)
	plt.rc('text', usetex=True)
	plt.rc('font', family='serif')
	ax.grid(b=None, which='major')
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_zlabel('z [m]')
	ax.set_xlim(-3,5)
	ax.set_ylim(-3,5)
	ax.set_zlim(0,6)
	regions = np.array([[0, 0, 5], [4, 5, 3],[-2, 4, 2],[3, -2, 3]])
	x,y,z = create_sphere(regions[0],0.05,10j)
	ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='r', alpha=0.5, linewidth=0)
	x,y,z = create_sphere(regions[1],0.05,10j)
	ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='g', alpha=0.5, linewidth=0)
	x,y,z = create_sphere(regions[2],0.05,10j)
	ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='b', alpha=0.5, linewidth=0)
	x,y,z = create_sphere(regions[3],0.05,10j)
	ax.plot_surface(x, y, z,  rstride=1, cstride=1, color='k', alpha=0.5, linewidth=0)
	ax.view_init(27, -26)



	ax.text(0,0,5.1,r"$x_{d,1}$",fontsize=15,color='k',weight='bold')
	ax.text(4,5,3.1,r"$x_{d,2}$",fontsize=15,color='k',weight='bold')
	ax.text(-2,4,2.1,r"$x_{d,3}$",fontsize=15,color='k',weight='bold')
	ax.text(3,-2,3.1,r"$x_{d,4}$",fontsize=15,color='k',weight='bold')


	blue_patch = mpatches.Patch(color='blue', label='UAV 1')
	red_patch = mpatches.Patch(color='red', label='UAV 2')
	green_patch = mpatches.Patch(color='green', label='UAV 3')
	yellow_patch = mpatches.Patch(color='yellow', label='UAV 4')
	black_patch = mpatches.Patch(color='black', label='UAV 5')
	white_patch = mpatches.Patch(color=[0.75,0.75,0.75], label='UAV 6')

	llocation = (0.13,0.55)
	plt.legend(handles=[blue_patch,red_patch,green_patch,yellow_patch,black_patch,white_patch], loc=llocation, bbox_to_anchor=[0.15,0.55], prop={'size':12})
	data = []	
	offsets = [-0.12,-0.1,-0.05,0.05,0.1,0.12]
	#offsets = [0.12,0.1,0.05,0.05,0.1,0.12]
	step_size = 2
	for ag in range(0,len(pos_array_cur)):
		for j in range(len(offsets)):
			x_sol_cur = np.asarray([pos_array_cur[ag][i][0] for i in range(0,lengt,step_size)])
			y_sol_cur = np.asarray([pos_array_cur[ag][i][1] for i in range(0,lengt,step_size)])
			z_sol_cur = np.asarray([pos_array_cur[ag][i][2] for i in range(0,lengt,step_size)])

			x_left_cur = offsets[j] + x_sol_cur
			y_left_cur = offsets[j] + y_sol_cur
			x_right_cur = x_left_cur
			y_right_cur = offsets[5-j] + y_sol_cur

			#print(np.size(x_sol_cur))
			cur_data = np.hstack((x_sol_cur,y_sol_cur,z_sol_cur)).reshape(3,len(x_sol_cur))
			data.append(cur_data)
			cur_data = np.hstack((x_left_cur,y_left_cur,z_sol_cur)).reshape(3,len(x_sol_cur))
			data.append(cur_data)
			cur_data = np.hstack((x_right_cur,y_right_cur,z_sol_cur)).reshape(3,len(x_sol_cur))
			data.append(cur_data)

	for m in range(7):
		m_1 = E_con[m][0]
		m_2 = E_con[m][1]
		
		x_sol_cur_1 = np.asarray([pos_array_cur[m_1][i][0] for i in range(0,lengt,step_size)])
		y_sol_cur_1 = np.asarray([pos_array_cur[m_1][i][1] for i in range(0,lengt,step_size)])
		z_sol_cur_1 = np.asarray([pos_array_cur[m_1][i][2] for i in range(0,lengt,step_size)])

		x_sol_cur_2 = np.asarray([pos_array_cur[m_2][i][0] for i in range(0,lengt,step_size)])
		y_sol_cur_2 = np.asarray([pos_array_cur[m_2][i][1] for i in range(0,lengt,step_size)])
		z_sol_cur_2 = np.asarray([pos_array_cur[m_2][i][2] for i in range(0,lengt,step_size)])		

		num_points = 30
		x_points = np.asarray([np.linspace(x_sol_cur_1[i],x_sol_cur_2[i],num=num_points) for i in range(0,len(x_sol_cur_1))]).reshape(len(x_sol_cur_1),num_points).T
		y_points = np.asarray([np.linspace(y_sol_cur_1[i],y_sol_cur_2[i],num=num_points) for i in range(0,len(x_sol_cur_1))]).reshape(len(x_sol_cur_1),num_points).T
		z_points = np.asarray([np.linspace(z_sol_cur_1[i],z_sol_cur_2[i],num=num_points) for i in range(0,len(x_sol_cur_1))]).reshape(len(x_sol_cur_1),num_points).T

		#print np.size(x_points)
		for k in range(num_points):
			#print np.size(x_points[k])
			cur_data = np.hstack((x_points[k],y_points[k],z_points[k])).reshape(3,len(x_sol_cur_1))
			data.append(cur_data)

		#ax.plot([pos_array_cur[m_1][i][0],pos_array_cur[m_2][i][0]], [pos_array_cur[m_1][i][1],pos_array_cur[m_2][i][1]], [pos_array_cur[m_1][i][2],pos_array_cur[m_2][i][2]],'k' )

	lines = []
	for dat in range(len(data)):
		cur = data[dat]
		if dat <= 18:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'b',marker='o')[0] )
		elif dat <= 36:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'r',marker='o')[0] )
		elif dat <= 54:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'g',marker='o')[0] )
		elif dat <= 72:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'y',marker='o')[0] )
		elif dat <= 90:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'k',marker='o')[0] )
		elif dat <= 107:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = [.8,.8,.8], marker='o')[0] )
		else:
			lines.append( ax.plot(cur[0, 0:1], cur[1, 0:1], cur[2, 0:1],color = 'k',marker='.')[0] )


	def animate(num,dataLines,lines):
		for line, data in zip(lines,dataLines):
			line.set_data(data[0:2, num])
			line.set_3d_properties(data[2, num])

		elevation = 37 - 10*np.cos(0.0025*num)
		azimuth = -31 + 25*np.sin(0.005*num)
		ax.view_init(elevation, azimuth)
		#print(num)
		return lines


	## FOR THE FIRST VIDEO

	# simulation_video = animation.FuncAnimation(fig, animate, len(x_sol_cur_1), fargs=(data, lines),interval=1, blit=False)

	# Writer = animation.writers['ffmpeg']
	# writer = Writer(fps=5, metadata=dict(artist='Me'), bitrate=250)
	# simulation_video.save('simulation_video.mp4', writer=writer)



	#animate THE DISTANCES
	
	r = 0.35
	d_con = 3
	beta_bound = 10**4
	con_distance_meas = 0 #inter agent distance where we start taking con. into account (greater than)
	con_offset = d_con**2 - con_distance_meas**2
	col_distance_meas = d_con #inter agent distance where we start taking coll. into account (less than)
	col_offset = col_distance_meas**2 - 4*r**2


	E_full = np.array([ [0,1],[0,2], [0,3], [0,4], [0,5], [1,2], [1,3], [1,4], [1,5], [2,3], [2,4], [2,5], [3,4], [3,5], [4,5] ])


	iotas = []
	etas = []
	plot_labels_iota = []
	plot_labels_eta = []
	for m in range(15):
		m_1 = E_full[m][0]
		m_2 = E_full[m][1]

		#string = r'$\| x_{value1} - x_{value2} \| - 4r^2$'.format(value1 = '{'+str(m_1)+'}', value2 = '{'+str(m_2)+'}')
		string = r'$\iota_{value}$'.format(value = '{'+str(m)+'}')

		#plot_labels.append("$||x_$" + str(m_1) + "$-x_$" + str(m_2) + "$|| - 4r^2$")
		plot_labels_iota.append(string)

		iota = []
		for t in range(0,lengt,step_size):
			iota.append(np.linalg.norm(pos_array_cur[m_1][t] - pos_array_cur[m_2][t])**2 - 4*r**2)
		#if m == 0:
			#print iota
		iotas.append(iota)


	for m in range(7):
		m_1 = E_con[m][0]
		m_2 = E_con[m][1]


		string = r'$\eta_{value}$'.format(value = '{'+str(m)+'}')

		#plot_labels.append("$||x_$" + str(m_1) + "$-x_$" + str(m_2) + "$|| - 4r^2$")
		plot_labels_eta.append(string)

		eta = []
		for t in range(lengt):
			eta.append(d_con**2 - np.linalg.norm( pos_array_cur[m_1][t]  - pos_array_cur[m_2][t])**2 )
		etas.append(eta)

	
		
	fig = plt.figure()
	ax1 = plt.axes(xlim=(0,277),ylim=(0,20))
	line, = ax1.plot([], [], lw=2)
	colors = [[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[.5,.5,.5],[1,.5,.5],[.5,1,.5],[.5,.5,1],[1,1,.5],[1,.5,1],[.5,1,1],[1,.5,.5]]
	lines = []
	for index in range(15):
	    lobj = ax1.plot([],[],lw=2,color=colors[index],label = plot_labels_iota[index])[0]
	    lines.append(lobj)

	plt.grid(True)
	plt.xlabel(r"$t \ [s]$", fontsize = 25)
	l=plt.legend()
	l.set_zorder(20) 
	plt.ylabel(r"$\iota_m(t), \ m\in\bar{\mathcal{M}}$", fontsize = 25)
	plt.title(r"$\iota_m = \|x_{m_1}-x_{m_2}\|^2-(r_{m_1}+r_{m_2})^2$",fontsize = 20)

	def init_iotas():
	    for line in lines:
	        line.set_data([],[])
	        line.set_zorder(0)
	    return lines


	x_list = [[] for i in range(15)]
	y_list = [[] for i in range(15)]

	
	def animate_iotas(i):

		#print iotas[0][i]		
		for k in range(15):
			x_list[k].append(time_l[i])
			y_list[k].append(iotas[k][i/4])
			

		#time.sleep(.5)
		#x = gps_data[0][0, i]
		#y = gps_data[1][0, i]
		#x1.append(x)
		#y1.append(y)

		#x = gps_data[0][1,i]
		#y = gps_data[1][1,i]
		#x2.append(x)
		#y2.append(y)

		#xlist = [x1, x2]
		#ylist = [y1, y2]

		#print 'x,y = ', x,y
		#print 'lists = ', xlist, ylist	
		#time.sleep(1)
		for lnum,line in enumerate(lines):
			line.set_data(x_list[lnum], y_list[lnum])
			line.set_zorder(0)
		#for lnum,line in enumerate(lines):
		#	line.set_data(xlist[lnum], ylist[lnum])
		return lines

	# call the animator.  blit=True means only re-draw the parts that have changed.
	iotas_video = animation.FuncAnimation(fig, animate_iotas, init_func=init_iotas,
                               frames=np.arange(0,lengt,4), interval=10, blit=False)

	Writer = animation.writers['ffmpeg']
	writer = Writer(fps=5, metadata=dict(artist='Me'), bitrate=175)
	iotas_video.save('iotas_video.mp4', writer=writer)
	
	print 'Done with iotas video'

	##################### ETAS

	fig = plt.figure()
	ax1 = plt.axes(xlim=(0,277),ylim=(0,10))
	line, = ax1.plot([], [], lw=2)
	colors = [[0,0,0],[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1]]
	lines = []
	for index in range(7):
	    lobj = ax1.plot([],[],lw=2,color=colors[index],label = plot_labels_eta[index])[0]
	    lines.append(lobj)

	plt.grid(True)
	plt.xlabel(r"$t \ [s]$", fontsize = 25)
	l=plt.legend()
	l.set_zorder(20) 
	plt.ylabel(r"$\eta_l(t), \ l\in \mathcal{M}_0$", fontsize = 25)
	plt.title(r"$\eta_l = \underline{d}^2_{n,l}-\|x_{l_1}-x_{l_2}\|^2$",fontsize = 20)

	def init_etas():
	    for line in lines:
	        line.set_data([],[])
	        line.set_zorder(0)
	    return lines


	x_list = [[] for i in range(7)]
	y_list = [[] for i in range(7)]

	
	def animate_etas(i):

		#print iotas[0][i]
		for k in range(7):
			x_list[k].append(time_l[i])
			y_list[k].append(etas[k][i/4])
			

		#time.sleep(.5)
		#x = gps_data[0][0, i]
		#y = gps_data[1][0, i]
		#x1.append(x)
		#y1.append(y)

		#x = gps_data[0][1,i]
		#y = gps_data[1][1,i]
		#x2.append(x)
		#y2.append(y)

		#xlist = [x1, x2]
		#ylist = [y1, y2]

		#print 'x,y = ', x,y
		#print 'lists = ', xlist, ylist	
		#time.sleep(1)
		for lnum,line in enumerate(lines):
			line.set_data(x_list[lnum], y_list[lnum])
			line.set_zorder(0)
		#for lnum,line in enumerate(lines):
		#	line.set_data(xlist[lnum], ylist[lnum])
		return lines

	# call the animator.  blit=True means only re-draw the parts that have changed.
	etas_video = animation.FuncAnimation(fig, animate_etas, init_func=init_etas,
                               frames=np.arange(0,lengt,4), interval=10, blit=False)

	Writer = animation.writers['ffmpeg']
	writer = Writer(fps=5, metadata=dict(artist='Me'), bitrate=175)
	etas_video.save('etas_video.mp4', writer=writer)

	print 'Done with etas video'

plt.show()    

      