#!/usr/bin/env python

import numpy as np

import rospy

import std_msgs.msg

import nav_msgs.msg

import mav_msgs.msg

import geometry_msgs.msg

import rosgraph_msgs.msg

import yaw_controller.yaw_controller as yaw_controller

import std_srvs.srv

import utilities.utility_functions as utility_functions

import tf.transformations

from dynamic_reconfigure.server import Server

from numpy import linalg 


def position_and_velocity_from_odometry(odometry):
    x = np.array([odometry.pose.pose.position.x,\
                  odometry.pose.pose.position.y,\
                  odometry.pose.pose.position.z])

    # TODO: naming of child_frame_id
    if odometry.child_frame_id == 'firefly/base_link':

        # velocity is in the body reference frame
        v_body = np.array([odometry.twist.twist.linear.x,\
                           odometry.twist.twist.linear.y,\
                           odometry.twist.twist.linear.z])

        quaternion = np.array([odometry.pose.pose.orientation.x,\
                               odometry.pose.pose.orientation.y,\
                               odometry.pose.pose.orientation.z,\
                               odometry.pose.pose.orientation.w])

        # TODO
        rotation_matrix  = utility_functions.rot_from_quaternion(quaternion)

        v = np.dot(rotation_matrix,v_body)

    else:
        # velocity is in the body reference frame
        v = np.array([odometry.twist.twist.linear.x,\
                      odometry.twist.twist.linear.y,\
                      odometry.twist.twist.linear.z])

    

    return x,v



def callback_uav_pose_reference(data):  
  global reference_pose
  reference_pose = data
  return 

def callback_sub_uav_odometry(data):
  global agent_pose
  agent_pose = data
  return

def callback_sub_othet_uav_odometry_1(data):  
  global other_agents_pose_1
  other_agents_pose_1 = data
  return

def callback_sub_othet_uav_odometry_2(data):  
  global other_agents_pose_2
  other_agents_pose_2 = data
  return

def callback_sub_othet_uav_odometry_3(data):  
  global other_agents_pose_3
  other_agents_pose_3 = data
  return

def callback_sub_othet_uav_arrival_1(data):  
  global other_agent_1_arrived
  other_agent_1_arrived = True
  return

def callback_sub_othet_uav_arrival_2(data):  
  global other_agent_1_arrived
  other_agent_2_arrived = True
  return

def callback_sub_othet_uav_arrival_3(data):  
  global other_agent_3_arrived
  other_agent_3_arrived = True
  return

def callback_start_MAS(data):
  global start_MAS
  start_MAS = True
  return

def navigation():

  rospy.init_node('navigation_node') 
  global reference_pose
  global agent_pose 
  global other_agents_pose_1  #agent 2 for 1, agent 1 for 2, agent 1 for 3, agent 1 for 4
  global other_agents_pose_2  #agent 3 for 1, agent 3 for 2, agent 2 for 3, agent 2 for 4
  global other_agents_pose_3  #agent 4 for 1, agent 4 for 2, agent 4 for 3, agent 3 for 4
  global PoI
  global other_agent_1_arrived
  global other_agent_2_arrived
  global other_agent_3_arrived
  global time 
  global start_MAS

  time = 0
  other_agent_1_arrived = False
  other_agent_2_arrived = False
  other_agent_3_arrived = False
  start_MAS = False

  PoI = np.array([[5, 5, 3], [-5, -0.10, 3],[2, 3, 5],[0, -4, 2]])

  #total of 5 agents

  force_pub = rospy.Publisher('uav_force_reference', mav_msgs.msg.TorqueThrust, queue_size = 100)



  #rospy.Subscriber('uav_pose_reference', geometry_msgs.msg.PoseStamped, callback_uav_pose_reference)
  rospy.Subscriber('uav_odometry', nav_msgs.msg.Odometry,callback_sub_uav_odometry)
  rospy.Subscriber('other_uav_odometry_1', nav_msgs.msg.Odometry,callback_sub_othet_uav_odometry_1)
  rospy.Subscriber('other_uav_odometry_2', nav_msgs.msg.Odometry,callback_sub_othet_uav_odometry_2)
  rospy.Subscriber('other_uav_odometry_3', nav_msgs.msg.Odometry,callback_sub_othet_uav_odometry_3)

  rospy.Subscriber('other_uav_arrival_1', std_msgs.msg.Bool,callback_sub_othet_uav_arrival_1)
  rospy.Subscriber('other_uav_arrival_2', std_msgs.msg.Bool,callback_sub_othet_uav_arrival_2)
  rospy.Subscriber('other_uav_arrival_3', std_msgs.msg.Bool,callback_sub_othet_uav_arrival_3)

  rospy.Subscriber('/start_MAS', std_msgs.msg.Bool, callback_start_MAS)
  #rospy.Subscriber('/clock', rosgraph_msgs.msg.Clock, callback_get_clock)

  temp_pub = rospy.Publisher('temp_topic', mav_msgs.msg.TorqueThrust, queue_size = 100)
  
  time = rospy.get_time()

 
  priority = rospy.get_param("priority")
  agent_number = rospy.get_param("agent_number")
  region_idx = rospy.get_param("first_region")

  xd = PoI[region_idx]
  #xd_dot = np.zeros(3)
  #xd_ddot = np.zeros(3)

  #0.5,3,1.5,5,5,0.5,0.1 with xd = [10,0,3]
  #0.5, 3, 0.5, 3, 1.5, 5, 10, 0.5, 0.1 with xd = [10,10,3]

  #GAINS

  kp_x = .05 #0.5 for 10     ..*0.05 
  kv_x = 2       #3
  ki_x = 0 #.01  
  
  kp_y = .05 #0.5 for 10    
  kv_y = 2
  ki_y = 0 # .01
  
  kp_z = 0.05  #1.5 for 3
  kv_z = 1
  ki_z = 0 #.01

  k_e_tilde = 0.05

  #k_con = 5  #10-15
  #k_col = 0.5  #0.5
  
  k_theta = 0.1
  k_f_b = 0.1
  k_d_b = 0.1

  ki = 2
  ki = 2 

  agent_pose = nav_msgs.msg.Odometry()
  other_agents_pose_1 = nav_msgs.msg.Odometry()
  other_agents_pose_2 = nav_msgs.msg.Odometry()
  other_agents_pose_3 = nav_msgs.msg.Odometry()
  #rospy.sleep(2)

  mass = 1.56779

  a_hat = 0
  a_hat_dot = 0
  d_b_hat = 0
  d_b_hat_dot = 0
  f_b_hat = 0
  f_b_hat_dot = 0
  theta_hat = 0
  theta_hat_dot = 0

  rate = rospy.Rate(100) 
  dt = 0.01

  

  #xd = np.array([10,-0.5,3])
  

  navigation_term = np.zeros(3)

  dissip_term = np.zeros(3)

  counter = 1

  mode = 0

  arrived_first_time = False

  integrator = np.zeros(3)

  
  while not rospy.is_shutdown():
    
    #print rospy.get_rostime()

    time = rospy.get_time()

    #desired traj, initial point [5, -0.10, 3]
    #xd[0] = 5 + 5*np.sin(0.1*time)
    #xd[1] = -0.1 + 5*np.cos(0.1*time)
    #xd[2] = 3+ 0.1*time

    #xd_dot[0] = 0.5*np.cos(0.1*time)
    #xd_dot[1] = -0.5*np.sin(0.1*time)
    #xd_dot[2] = 0.1

    #xd_ddot[0] = -0.05*np.sin(0.1*time)
    #xd_ddot[1] = -0.05*np.cos(0.1*time)
    #xd_ddot[2] = 0
  




    


    # get up
    #error_int = np.zeros(3)
    while not start_MAS:
      x,v = position_and_velocity_from_odometry(agent_pose)
      x_d = np.array([x[0],x[1],3])
      error = x - x_d
      error_d = v
      error_int += 0.01*error
      pid_p = .7
      pid_d = 2
      pid_i = 1
      force = -pid_p*error - pid_d*error_d #- pid_i*error_int
      force[2] += 9.81*1.56
      mesage_to_pub = mav_msgs.msg.TorqueThrust()
      mesage_to_pub.thrust.x = force[0]
      mesage_to_pub.thrust.y = force[1]
      mesage_to_pub.thrust.z = force[2]
      #print 'BEFORE. agent = ', agent_number, ', control = ', force

      
      #print 'error = ', error
      #print 'force = ', force

      force_pub.publish(mesage_to_pub)


    x,v = position_and_velocity_from_odometry(agent_pose)
    x_other_1, v_other_1 = position_and_velocity_from_odometry(other_agents_pose_1)
    x_other_2, v_other_2 = position_and_velocity_from_odometry(other_agents_pose_2)
    x_other_3, v_other_3 = position_and_velocity_from_odometry(other_agents_pose_3)

    if priority == 1:
      mode = 1
    else:
      mode = 0
    
    a_hat = a_hat + dt*a_hat_dot
    d_b_hat = d_b_hat + dt*d_b_hat_dot
    f_b_hat = f_b_hat + dt*f_b_hat_dot
    theta_hat = theta_hat + dt*theta_hat_dot

    #print 'agent number and priority and mode =  ', agent_number, priority, mode


    ep = np.zeros(3)
    if mode==1:
      ep = x - xd
      integrator = integrator + dt*ep
      ki = .1
      ki = .1
  
    u = np.zeros(3)
    e3 = np.array([0.0,0.0,1.0])

    r = 0.35
    d_con = 4 



    #edges = { (1,2), (1,3), (1,4), (3,4) }
    eta_con_1 = d_con**2 - np.linalg.norm(x-x_other_1)**2     
    eta_con_2 = 0    
    eta_con_3 = 0
    
    eta_con_dot_1 = -2*np.dot((x-x_other_1), (v - v_other_1))
    eta_con_dot_2 = 0
    eta_con_dot_3 = 0


    grad_beta_con_2 = np.zeros(3)
    grad_beta_con_3 = np.zeros(3)
    grad_beta_con_dot_2 = np.zeros(3)
    grad_beta_con_dot_3 = np.zeros(3)

    if agent_number == 1:
      eta_con_2 = d_con**2 - np.linalg.norm(x-x_other_2)**2
      eta_con_3 = d_con**2 - np.linalg.norm(x-x_other_3)**2
      eta_con_dot_2 = -2*np.dot((x-x_other_2), (v - v_other_2))
      eta_con_dot_3 = -2*np.dot((x-x_other_3), (v - v_other_3))

    if agent_number == 3:
      eta_con_3 = d_con**2 - np.linalg.norm(x-x_other_3)**2
      eta_con_dot_3 = -2*np.dot((x-x_other_3), (v - v_other_3))

    if agent_number == 4:
      eta_con_3 = d_con**2 - np.linalg.norm(x-x_other_3)**2
      eta_con_dot_3 = -2*np.dot((x-x_other_3), (v - v_other_3))

    
    iota_col_1 = np.linalg.norm(x-x_other_1)**2 - 4*r**2
    iota_col_2 = np.linalg.norm(x-x_other_2)**2 - 4*r**2
    iota_col_3 = np.linalg.norm(x-x_other_3)**2 - 4*r**2

    iota_col_dot_1 = 2*np.dot(x-x_other_1, v-v_other_1)
    iota_col_dot_2 = 2*np.dot(x-x_other_2, v-v_other_2)
    iota_col_dot_3 = 2*np.dot(x-x_other_3, v-v_other_3)


    con_distance_meas = 0 #inter agent distance where we start taking con. into account (greater than)
    con_offset = d_con**2 - con_distance_meas**2

    beta_bound = 10**2

    if eta_con_1 < 0:
      beta_con_1 = 0
      grad_beta_con_1 = np.zeros(3)
      grad_beta_con_dot_1 = np.zeros(3)

    elif eta_con_1 < con_offset:

                  #[con_offset^5 con_offset^4 con_offset^3;5*con_offset^4 4*con_offset^3 3*con_offset^2;20*con_offset^3 12*con_offset^2 6*con_offset];

    # beta_con(m) = coeff(1)*eta(m)^5 + coeff(2)*eta(m)^4 + coeff(3)*eta(m)^3;
    # grad_beta_con_m1(:,m) = -1/beta_con(m)^2 * ( 5*coeff(1)*eta(m)^4 + 4*coeff(2)*eta(m)^3 + 3*coeff(3)*eta(m)^2 )*(-2*(p(:,m_1)-p(:,m_2)));
    # grad_beta_con_m1_dot(:,m) = -1/beta_con(m)^2 * ( 5*coeff(1)*eta(m)^4 + 4*coeff(2)*eta(m)^3 + 3*coeff(3)*eta(m)^2 )*(-2*(v(:,m_1)-v(:,m_2)))+ ...
    #                      2/beta_con(m)^3 * ( 5*coeff(1)*eta(m)^4 + 4*coeff(2)*eta(m)^3 + 3*coeff(3)*eta(m)^2 )^2*(-2*(p(:,m_1)-p(:,m_2)))'*(v(:,m_1)-v(:,m_2))*(-2*(p(:,m_1)-p(:,m_2))) + ...
    #                        -1/beta_con(m)^2 * ( 20*coeff(1)*eta(m)^3 + 12*coeff(2)*eta(m)^2 + 6*coeff(3)*eta(m) )*(-2*(p(:,m_1)-p(:,m_2)))'*(v(:,m_1)-v(:,m_2))*(-2*(p(:,m_1)-p(:,m_2)));
            
      A = np.array([[con_offset**5,con_offset**4, con_offset**3],[5*con_offset**4, 4*con_offset**3, 3*con_offset**2], [20*con_offset**3, 12*con_offset**2, 6*con_offset]])
      B = np.array([beta_bound,0,0])

      coeff = np.dot(np.linalg.inv(A),B)
      

      beta_con_1 = coeff[0]*eta_con_1**5 + coeff[1]*eta_con_1**4 + coeff[2]*eta_con_1**3
      grad_beta_con_1 = -1/beta_con_1**2*(5*coeff[0]*eta_con_1**4 + 4*coeff[1]*eta_con_1**3 + 3*coeff[2]*eta_con_1**2)*(-2*(x-x_other_1))

      beta_con_dot_1 = (5*coeff[0]*eta_con_1**4 + 4*coeff[1]*eta_con_1**3 + 3*coeff[2]*eta_con_1**2) * eta_con_dot_1
      term1 = 2/beta_con_1**3 * beta_con_dot_1 * (5*coeff[0]*eta_con_1**4 + 4*coeff[1]*eta_con_1**3 + 3*coeff[2]*eta_con_1**2)*(-2*(x-x_other_1))
      term2 = -1/beta_con_1**2 * (20*coeff[0]*eta_con_1**3 + 12*coeff[1]*eta_con_1**2 + 6*coeff[2]*eta_con_1)*eta_con_dot_1*(-2*(x-x_other_1))
      term3 = -1/beta_con_1**2*(5*coeff[0]*eta_con_1**4 + 4*coeff[1]*eta_con_1**3 + 3*coeff[2]*eta_con_1**2)*(-2*(v-v_other_1))
      grad_beta_con_dot_1 = term1 + term2 + term3

    else:
      beta_con_1 = beta_bound
      grad_beta_con_1 = np.zeros(3)
      grad_beta_con_dot_1 = np.zeros(3)


    if agent_number==1:
      if eta_con_2 < 0:
        beta_con_2 = 0
        grad_beta_con_2 = np.zeros(3)
        grad_beta_con_dot_2 = np.zeros(3)
      
      elif eta_con_2 < con_offset:

        A = np.array([[con_offset**5,con_offset**4, con_offset**3],[5*con_offset**4, 4*con_offset**3, 3*con_offset**2], [20*con_offset**3, 12*con_offset**2, 6*con_offset]])
        B = np.array([beta_bound,0,0])

        coeff = np.dot(np.linalg.inv(A),B)
      
        beta_con_2 = coeff[0]*eta_con_2**5 + coeff[1]*eta_con_2**4 + coeff[2]*eta_con_2**3
        grad_beta_con_2 = -1/beta_con_2**2*(5*coeff[0]*eta_con_2**4 + 4*coeff[1]*eta_con_2**3 + 3*coeff[2]*eta_con_2**2)*(-2*(x-x_other_2))

        beta_con_dot_2 = (5*coeff[0]*eta_con_2**4 + 4*coeff[1]*eta_con_2**3 + 3*coeff[2]*eta_con_2**2 )*eta_con_dot_2
        term1 = 2/beta_con_2**3 * beta_con_dot_2 * (5*coeff[0]*eta_con_2**4 + 4*coeff[1]*eta_con_2**3 + 3*coeff[2]*eta_con_2**2)*(-2*(x-x_other_2))
        term2 = -1/beta_con_2**2 * (20*coeff[0]*eta_con_2**3 + 12*coeff[1]*eta_con_2**2 + 6*coeff[2]*eta_con_2)*eta_con_dot_2*(-2*(x-x_other_2))
        term3 = -1/beta_con_2**2*(5*coeff[0]*eta_con_2**4 + 4*coeff[1]*eta_con_2**3 + 3*coeff[2]*eta_con_2**2)*(-2*(v-v_other_2))
        grad_beta_con_dot_2 = term1 + term2 + term3

      else:
        beta_con_2 = beta_bound
        grad_beta_con_2 = np.zeros(3)
        grad_beta_con_dot_2 = np.zeros(3)


    if agent_number==1 or agent_number==3 or agent_number==4:
      if eta_con_3 < 0:
        beta_con_3 = 0
        grad_beta_con_3 = np.zeros(3)
        grad_beta_con_dot_3 = np.zeros(3)
      
      elif eta_con_3 < con_offset:

        A = np.array([[con_offset**5,con_offset**4, con_offset**3],[5*con_offset**4, 4*con_offset**3, 3*con_offset**2], [20*con_offset**3, 12*con_offset**2, 6*con_offset]])
        B = np.array([beta_bound,0,0])

        coeff = np.dot(np.linalg.inv(A),B)
      
        beta_con_3 = coeff[0]*eta_con_3**5 + coeff[1]*eta_con_3**4 + coeff[2]*eta_con_3**3
        grad_beta_con_3 = -1/beta_con_3**2*(5*coeff[0]*eta_con_3**4 + 4*coeff[1]*eta_con_3**3 + 3*coeff[2]*eta_con_3**2)*(-2*(x-x_other_3))

        beta_con_dot_3 = (5*coeff[0]*eta_con_3**4 + 4*coeff[1]*eta_con_3**3 + 3*coeff[2]*eta_con_3**2)*eta_con_dot_3
        term1 = 2/beta_con_3**3 * beta_con_dot_3 * (5*coeff[0]*eta_con_3**4 + 4*coeff[1]*eta_con_3**3 + 3*coeff[2]*eta_con_3**2)*(-2*(x-x_other_3))
        term2 = -1/beta_con_3**2 * (20*coeff[0]*eta_con_3**3 + 12*coeff[1]*eta_con_3**2 + 6*coeff[2]*eta_con_3)*eta_con_dot_3*(-2*(x-x_other_3))
        term3 = -1/beta_con_3**2*(5*coeff[0]*eta_con_3**4 + 4*coeff[1]*eta_con_3**3 + 3*coeff[2]*eta_con_3**2)*(-2*(v-v_other_3))
        grad_beta_con_dot_3 = term1 + term2 + term3 
      else:
        beta_con_3 = beta_bound
        grad_beta_con_3 = np.zeros(3)
        grad_beta_con_dot_3 = np.zeros(3)


    col_distance_meas = 4 #inter agent distance where we start taking coll. into account (less than)
    col_offset = col_distance_meas**2 - 4*r**2


    if iota_col_1 <= 0:
      beta_col_1 = 0
      grad_beta_col_1 = np.zeros(3)
      grad_beta_col_dot_1 = np.zeros(3)
            
    elif iota_col_1 <= col_offset:

      A = np.array([[col_offset**5,col_offset**4, col_offset**3],[5*col_offset**4, 4*col_offset**3, 3*col_offset**2], [20*col_offset**3, 12*col_offset**2, 6*col_offset]])
      B = np.array([beta_bound,0,0])

      coeff = np.dot(np.linalg.inv(A),B)
      
      # beta_col(m) = coeff(1)*Delta(m)^5 + coeff(2)*Delta(m)^4 + coeff(3)*Delta(m)^3;
      #       grad_beta_col_m1(:,m) = -1/beta_col(m)^2 * ( 5*coeff(1)*Delta(m)^4 + 4*coeff(2)*Delta(m)^3 + 3*coeff(3)*Delta(m)^2 )*[2*(p(:,m_1)-p(:,m_2))];
      #       grad_beta_col_m1_dot(:,m) = -1/beta_col(m)^2 * ( 5*coeff(1)*Delta(m)^4 + 4*coeff(2)*Delta(m)^3 + 3*coeff(3)*Delta(m)^2 )*(2*(v(:,m_1)-v(:,m_2)))+ ...
      #                                    2/beta_col(m)^3 * ( 5*coeff(1)*Delta(m)^4 + 4*coeff(2)*Delta(m)^3 + 3*coeff(3)*Delta(m)^2 )^2*(2*(p(:,m_1)-p(:,m_2)))'*(v(:,m_1)-v(:,m_2))*2*(p(:,m_1)-p(:,m_2)) + ...
      #                                   -1/beta_col(m)^2 * ( 20*coeff(1)*Delta(m)^3 + 12*coeff(2)*Delta(m)^2 + 6*coeff(3)*Delta(m) )*(2*(p(:,m_1)-p(:,m_2)))'*(v(:,m_1)-v(:,m_2))*2*(p(:,m_1)-p(:,m_2));


      beta_col_1 = coeff[0]*iota_col_1**5 + coeff[1]*iota_col_1**4 + coeff[2]*iota_col_1**3
      grad_beta_col_1 = -1/beta_col_1**2*(5*coeff[0]*iota_col_1**4 + 4*coeff[1]*iota_col_1**3 + 3*coeff[2]*iota_col_1**2)*(2*(x-x_other_1))

      beta_col_dot_1 = (5*coeff[0]*iota_col_1**4 + 4*coeff[1]*iota_col_1**3 + 3*coeff[2]*iota_col_1**2)*iota_col_dot_1
      term1 = 2/beta_col_1**3 * beta_col_dot_1 * (5*coeff[0]*iota_col_1**4 + 4*coeff[1]*iota_col_1**3 + 3*coeff[2]*iota_col_1**2)*(2*(x-x_other_1))
      term2 = -1/beta_col_1**2 * (20*coeff[0]*iota_col_1**3 + 12*coeff[1]*iota_col_1**2 + 6*coeff[2]*iota_col_1)*iota_col_dot_1*(2*(x-x_other_1))
      term3 = -1/beta_col_1**2*(5*coeff[0]*iota_col_1**4 + 4*coeff[1]*iota_col_1**3 + 3*coeff[2]*iota_col_1**2)*(2*(v-v_other_1))
      grad_beta_col_dot_1 = term1 + term2 + term3 

    else:
      beta_col_1 = beta_bound
      grad_beta_col_1 = np.zeros(3)    
      grad_beta_col_dot_1 = np.zeros(3)        
             
    
    if iota_col_2 <= 0:
      beta_col_2 = 0
      grad_beta_col_2 = np.zeros(3)
      grad_beta_col_dot_2 = np.zeros(3)  
            
    elif iota_col_2 <= col_offset:

      A = np.array([[col_offset**5,col_offset**4, col_offset**3],[5*col_offset**4, 4*col_offset**3, 3*col_offset**2], [20*col_offset**3, 12*col_offset**2, 6*col_offset]])
      B = np.array([beta_bound,0,0])

      coeff = np.dot(np.linalg.inv(A),B)
      #print A
      #print
      #print 'coef = ', coeff 

      beta_col_2 = coeff[0]*iota_col_2**5 + coeff[1]*iota_col_2**4 + coeff[2]*iota_col_2**3
      grad_beta_col_2 = -1/beta_col_2**2*(5*coeff[0]*iota_col_2**4 + 4*coeff[1]*iota_col_2**3 + 3*coeff[2]*iota_col_2**2)*(2*(x-x_other_2))

      beta_col_dot_2 = (5*coeff[0]*iota_col_2**4 + 4*coeff[1]*iota_col_2**3 + 3*coeff[2]*iota_col_2**2)*iota_col_dot_2
      term1 = 2/beta_col_2**3 * beta_col_dot_2 * (5*coeff[0]*iota_col_2**4 + 4*coeff[1]*iota_col_2**3 + 3*coeff[2]*iota_col_2**2)*(2*(x-x_other_2))
      term2 = -1/beta_col_2**2 * (20*coeff[0]*iota_col_2**3 + 12*coeff[1]*iota_col_2**2 + 6*coeff[2]*iota_col_2)*iota_col_dot_2*(2*(x-x_other_2))
      term3 = -1/beta_col_2**2*(5*coeff[0]*iota_col_2**4 + 4*coeff[1]*iota_col_2**3 + 3*coeff[2]*iota_col_2**2)*(2*(v-v_other_2))
      grad_beta_col_dot_2 = term1 + term2 + term3 

    else:
      beta_col_2 = beta_bound
      grad_beta_col_2 = np.zeros(3)
      grad_beta_col_dot_2 = np.zeros(3)  


    if iota_col_3 <= 0:
      beta_col_3 = 0
      grad_beta_col_3 = np.zeros(3)
      grad_beta_col_dot_3 = np.zeros(3) 
            
    elif iota_col_3 <= col_offset:

      A = np.array([[col_offset**5,col_offset**4, col_offset**3],[5*col_offset**4, 4*col_offset**3, 3*col_offset**2], [20*col_offset**3, 12*col_offset**2, 6*col_offset]])
      B = np.array([beta_bound,0,0])

      coeff = np.dot(np.linalg.inv(A),B)
      #print A
      #print
      #print 'coef = ', coeff 

      beta_col_3 = coeff[0]*iota_col_3**5 + coeff[1]*iota_col_3**4 + coeff[2]*iota_col_3**3
      grad_beta_col_3 = -1/beta_col_3**2*(5*coeff[0]*iota_col_3**4 + 4*coeff[1]*iota_col_3**3 + 3*coeff[2]*iota_col_3**2)*(2*(x-x_other_3))

      beta_col_dot_3 = (5*coeff[0]*iota_col_3**4 + 4*coeff[1]*iota_col_3**3 + 3*coeff[2]*iota_col_3**2)*iota_col_dot_3
      term1 = 2/beta_col_3**3 * beta_col_dot_3 * (5*coeff[0]*iota_col_3**4 + 4*coeff[1]*iota_col_3**3 + 3*coeff[2]*iota_col_3**2)*(2*(x-x_other_3))
      term2 = -1/beta_col_3**2 * (20*coeff[0]*iota_col_3**3 + 12*coeff[1]*iota_col_3**2 + 6*coeff[2]*iota_col_3)*iota_col_dot_3*(2*(x-x_other_3))
      term3 = -1/beta_col_3**2*(5*coeff[0]*iota_col_3**4 + 4*coeff[1]*iota_col_3**3 + 3*coeff[2]*iota_col_3**2)*(2*(v-v_other_3))
      grad_beta_col_dot_3 = term1 + term2 + term3 


    else:
      beta_col_3 = beta_bound
      grad_beta_col_3 = np.zeros(3)
      grad_beta_col_dot_3 = np.zeros(3) 
    

      

    #Desired velocity:
    beta_term_con = -grad_beta_con_1 - grad_beta_con_2 - grad_beta_con_3 
    beta_term_col = -grad_beta_col_1 - grad_beta_col_2 - grad_beta_col_3  

    beta_term_con_dot = -grad_beta_con_dot_1 - grad_beta_con_dot_2 - grad_beta_con_dot_3 
    beta_term_col_dot = -grad_beta_col_dot_1 - grad_beta_col_dot_2 - grad_beta_col_dot_3

    if mode==1:
      navigation_term[0] = - kp_x*ep[0]
      navigation_term[1] = - kp_y*ep[1] 
      navigation_term[2] = - kp_z*ep[2] 
      v_des = navigation_term + ki*(beta_term_col + beta_term_con) 
      v_des_dot = np.array([-kp_x*v[0],-kp_y*v[1],-kp_z*v[2]]) + ki*(beta_term_col_dot + beta_term_con_dot) 
    else:
      v_des = ki*(beta_term_col + beta_term_con) 
      v_des_dot = ki*(beta_term_col_dot + beta_term_con_dot) 

    e_v = v - v_des
    
    dissip_term[0]   = - kv_x*e_v[0]
    dissip_term[1]   = - kv_y*e_v[1]
    dissip_term[2]   = - kv_z*e_v[2]

    if mode==1:
      e_tilde = ep
    else:
      e_tilde = np.zeros(3)

    grav = 9.81
    #dynamics: 
    #  Y = [px_ddot    0     0     0; 
    #       py_ddot    0     0     0;
    #       pz_ddot+g  0     0     0; ]
    Y = np.array( [v_des_dot[0],v_des_dot[1],v_des_dot[2]+grav])
    
    #      [0          0   -w2*w3  w2*w3;
    #       0         w1*w3  0    -w1*w3;
    #       0        -w1*w2 w1*w2  0;]

    control = beta_term_col + beta_term_con - k_e_tilde*e_tilde + Y*theta_hat + dissip_term - np.sign(e_v)*np.linalg.norm(v,1)*f_b_hat - np.sign(e_v)*d_b_hat




    d_b_hat_dot = k_d_b*np.linalg.norm(e_v)
    f_b_hat_dot = k_f_b*np.linalg.norm(e_v,1)*np.linalg.norm(v)
    theta_hat_dot = -k_theta*np.dot(Y,e_v)

    # print 'control = ', control,
    # print 'd b hat dot = ',  d_b_hat_dot
    # print 'f b hat dot = ', f_b_hat_dot
    # print 'theta hat dot = ',  theta_hat_dot

    #control = navigation_term  + 9.81*mass*e3 - a_hat*v -k_con*grad_beta_con - k_col*grad_beta_col
    #control = navigation_term + dissip_term -a_hat*v + 9.81*mass*e3  -k_con*grad_beta_con_1 -k_con*grad_beta_con_2 -k_con*grad_beta_con_3 - k_col*grad_beta_col_1 - k_col*grad_beta_col_2 - k_col*grad_beta_col_3
    #_hat_dot = k_a*np.linalg.norm(v)**2

    #print control
    #if agent_number==1 or agent_number==4:
    #print 'agent_number= ', agent_number, 'navigation_term = ', navigation_term
    #print 'x = ', x
    #print 'x_other_1 = ', x_other_1
    #print 'dist = ', np.linalg.norm(x-x_other_1)
    #print 'eta_con 1 = ', eta_con_1
    #print 'eta_con 2 = ', eta_con_2
    #print 'iota_col 1 = ', iota_col_1
    #print 'iota_col 2 = ', iota_col_2
    #print 'ep = ', ep
    #print 'navigation_term = ', navigation_term
    # print 'agent = ', agent_number, ', x other 1 = ', x_other_1
    # print 'agent = ', agent_number, ', x other 2 = ', x_other_2
    # print 'agent = ', agent_number, ', x other 3 = ', x_other_3
    print 'agent = ', agent_number, ', ep = ', ep
    # print 'agent = ', agent_number, ', mode = ', mode
    print 'agent = ', agent_number, ', navigation_term = ', navigation_term
    print 'agent = ', agent_number, ', adapttation theta term = ', Y*theta_hat
    print 'agent = ', agent_number, ', adapttation f term = ', -np.sign(e_v)*np.linalg.norm(v,1)*f_b_hat
    print 'agent = ', agent_number, ', adapttation d term = ', -np.sign(e_v)*d_b_hat
    #print 'agent = ', agent_number, ', control = ', np.array([control[0],control[1],force[2]])
    print 'agent = ', agent_number, ', control = ', control

    # print 'agent = ', agent_number, ', x = ', x
    # print 'agent = ', agent_number, ', xd = ', xd
    # print 'agent = ', agent_number, ', |x-xd| = ', np.linalg.norm(x-xd)
    #print 'agent = ', agent_number, ', other agent pose 1 = ', x_other_1
    #print 'agent = ', agent_number, ', other agent pose 2 = ', x_other_2
    #print 'agent = ', agent_number, ', other agent pose 3 = ', x_other_3


    print 'agent = ', agent_number, ', grad beta col 1 = ', - grad_beta_col_1 
    print 'agent = ', agent_number, ', grad beta col 2 = ', - grad_beta_col_2 
    print 'agent = ', agent_number, ', grad beta col 3 = ', - grad_beta_col_3
    # print 'agent = ', agent_number, ', dist_1 = ', np.linalg.norm(x-x_other_1)
    # print 'agent = ', agent_number, ', dist_2 = ', np.linalg.norm(x-x_other_2)
    # print 'agent = ', agent_number, ', dist_3 = ', np.linalg.norm(x-x_other_3)
    print 'agent = ', agent_number, ', grad_beta_con 1 = ', - grad_beta_con_1 
    print 'agent = ', agent_number, ', grad_beta_con 2 = ', - grad_beta_con_2 
    print 'agent = ', agent_number, ', grad_beta_con 3 = ', - grad_beta_con_3
    # print 'agent = ', agent_number, ', velocity = ', v
    # print 'agent = ', agent_number, ', a_hat*v = ', a_hat*v
    # print 'agent = ', agent_number, ', total conn. term = ', -k_con*grad_beta_con_1 -k_con*grad_beta_con_2 -k_con*grad_beta_con_3 
    # print 'agent = ', agent_number, ', total col. term = ', -k_col*grad_beta_col_1 -k_col*grad_beta_col_2 -k_col*grad_beta_col_3 

    #print 'control = ', control
    

    mesage_to_pub = mav_msgs.msg.TorqueThrust()
    mesage_to_pub.thrust.x = control[0]
    mesage_to_pub.thrust.y = control[1]
    #mesage_to_pub.thrust.z = control[2]
    mesage_to_pub.thrust.z = force[2] #keep z component

    tmp_mesage_to_pub = mav_msgs.msg.TorqueThrust()
    tmp_mesage_to_pub.thrust.x = x_other_1[0]
    tmp_mesage_to_pub.thrust.y = x_other_1[1]
    tmp_mesage_to_pub.thrust.z = x_other_1[2]

    force_pub.publish(mesage_to_pub)
    #temp_pub.publish(tmp_mesage_to_pub)

    
    rate.sleep()



if __name__ == '__main__':     

    try:        
        navigation()
    except rospy.ROSInterruptException:
        pass