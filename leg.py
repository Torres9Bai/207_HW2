import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib


# For symbolic processing
import sympy
from sympy import symbols
from sympy import sin, cos, asin, acos, pi
from sympy.utilities.lambdify import lambdify
from sympy import Matrix


class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """
    
    global l1,l2,l_base,l0,theta0_sym,theta1_sym,alpha0_sym,alpha1_sym,encoder2angle,home

    #### Variables outside the init function are constants of the class
    # leg geometry
    l1 = 7  # NEED TO UPDATE units of cm
    l2 = 14  # NEED TO UPDATE units of cm
    l0 = 7.7  # NEED TO UPDATE units of cm
    
    theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols(
        'theta1_sym theta2_sym alpha1_sym alpha2_sym', real=True)
   

    # motor controller parameters
    encoder2angle = 2048 * 4

    ### Methods
    # Classes are initiated with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables

    def __init__(self, simulate = False):
        """
        This is the constructor for the leg class. Whenever you make a new leg
        this code will be called. We can optionally make a leg that will simulate
        the computations without needing to be connected to the ODrive
        """

        self.simulate = False #simulate

        # make the option to code without having the odrive connected
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  # easier handles to the motor commands
            self.m1 = self.drv.motor1
            
            # current positions
            m0_pos, m1_pos = self.get_joint_pos()
            self.joint_0_pos = m0_pos
            self.joint_1_pos = m1_pos

        else:
            self.drv = None

        # home angles
        self.joint_0_home = 0
        self.joint_1_home = 0

        

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        print('here2')
        self.J = self.compute_jacobian()
        # self.Jacobian_inv = self.Jacobian.inv

    def connect_to_controller(self):
        """
        Connects to the motor controller
        """
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###
    def get_joint_pos(self):
        """
        Get the current joint positions and store them in self.joint_0_pos and self.joint_1_pos in degrees.
        Also, return these positions using the return statement to terminate the function
        """
        # if simulating exit function
        if self.simulate == True:
            return (-1, -1)
        
        else:
            self.joint_0_pos=self.m0.encoder.pll_pos/(2048*4)*2*pi+pi/2
            self.joint_1_pos=self.m1.encoder.pll_pos/(2048*4)*2*pi+pi/2        

            return (self.joint_0_pos, self.joint_1_pos)

    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return
        
        
        else:
            self.home=(self.m0.encoder.pll_pos/(2048*4)*2*pi+pi/2,self.m1.encoder.pll_pos/(2048*4)*2*pi+pi/2)
            #print(self.home)
            return(home)
        
        #return(theta_0, theta_1)
        

        #
        # Your code here
        #


    def set_joint_pos(self, theta0, theta1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            self.joint_0_pos=theta0
            self.joint_1_pos=theta1
            return
        
        else:
            #self.m0.pos_setpoint=theta0*2048*4/(2*pi)-pi/2
            #self.m0.pos_setpoint(((theta0-self.home[0]))/(2*pi)*(2048*4),0,0)  
            #self.m1.pos_setpoint(((theta1-self.home[1]))/(2*pi)*(2048*4),0,0)
            self.set_home()
            self.m0.pos_setpoint(((theta0-(self.m0.encoder.pll_pos/(2048*4)*2*pi+pi/2)))/(2*pi)*(2048*4),0,0)  
            self.m1.pos_setpoint(((theta1-(self.m1.encoder.pll_pos/(2048*4)*2*pi+pi/2)))/(2*pi)*(2048*4),0,0)
            
        
        
        

        #
        # Your code here
        #


    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            return
        else:
            self.m0.pos_setpoint=self.home[0]
            self.m1.pos_setpoint=self.home[1]
        


        #
        # Your code here
        #

    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            (theta_0,theta_1)=self.inverse_kinematics(x,y)
            self.joint_0_pos=theta_0
            self.joint_1_pos=theta_1
            print(theta_0,theta_1)
            return
        else:
            (theta_0,theta_1)=self.inverse_kinematics(x,y)
            self.m0.pos_setpoint=theta_0-self.home[0]
            self.m1.pos_setpoint=theta_1-self.home[1]
        
       
        #inverse_kinematics(self, x, y)
        
        

        #
        # Your code here
        #

    
    
    
    
    def move_t(self, tt, xx, yy):
    
        if self.simulate == True:
            theta00=[]
            theta11=[]
            alpha00=[]
            alpha11=[]
            for i in range(tt):
                (theta0,theta1)=self.inverse_kinematics(xx[i],yy[i])
                theta00.append(theta0)
                theta11.append(theta1)
                (alpha0,alpha1)=self.compute_internal_angles(theta0,theta1)
                alpha00.append(alpha0)
                alpha11.append(alpha1)
            return(theta00,theta11,alpha00,alpha11)
        else:
            for i in range(tt):
                self.set_foot_pos(xx[i],yy[i])  
    
    
    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            fig=plt.figure()
            ax=plt.subplot(1,1,1)
            for i in range (tt):
                self.set_foot_pos(xx[i],yy[i])
                self.draw_leg(ax)
                
                plt.pause(0.1)
                plt.show()
            
            
            
        else:
            for i in range (tt):
                self.set_foot_pos(xx[i],yy[i])
        
        

        #
        # Your code here
        #

    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0, theta_1):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """

        
        l3=sympy.sqrt(l1**2 + l0**2-2*l1*l0*cos(theta_1));
        alpha2=acos((l0**2+l3**2-l1**2)/(2*l0*l3));
        alpha3=pi-theta_0-alpha2;
        l4=sympy.sqrt(l1**2+l3**2-2*l1*l3*cos(alpha3));
        alpha4=acos((l4**2+l1**2-l3**2)/(2*l1*l4));
        alpha5=acos((l4**2)/(2*l4*l2));
        
        alpha_0=pi-alpha4-alpha5+theta_0
        
        l5=sympy.sqrt(l1**2+l0**2-2*l1*l0*cos(3.14-theta_0));
        alpha6=acos((l0**2+l5**2-l1**2)/(2*l0*l5));
        alpha7=theta_1-alpha6;
        l6=sympy.sqrt(l1**2+l5**2-2*l1*l5*cos(alpha7));
        alpha8=acos((l1**2+l6**2-l5**2)/(2*l6*l1));
        alpha9=acos((l6**2+l2**2-l2**2)/(2*l6*l2));
        
        alpha_1=alpha8+alpha9-pi+theta_1
        

        return (alpha_0, alpha_1)

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """      
        #initiate the symbolic variables
        #theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols('theta1_sym theta2_sym alpha1_sym alpha2_sym', real=True)
        
        
        
        (alpha0_sym,alpha1_sym)=self.compute_internal_angles(theta0_sym,theta1_sym)
        
        x=l0/2+l1*cos(theta0_sym)+l2*cos(alpha0_sym);
        y=l1*sin(theta0_sym)+l2*sin(alpha0_sym);
        
        J=Matrix([[sympy.diff(x,theta0_sym),sympy.diff(x,theta1_sym)],[sympy.diff(y,theta0_sym),sympy.diff(y,theta1_sym)]])

        #
        # Your code here that solves J as a matrix
        #

        return J

    def inverse_kinematics(self, x, y):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """
        
        
        
        #theta_0=self.joint_0_pos
        #theta_1=self.joint_1_pos
        
        theta_0=pi/2
        theta_1=pi/2
        #J1=self.J.subs([(theta0_sym,theta_0),(theta1_sym,theta_1)])
        #J1=sympy.N(J1)
        #J_inv=J1.pinv()
        e=1
        while e>10^(-1):
            cur_theta=Matrix([[theta_0],[theta_1]])
            (alpha_0,alpha_1)=self.compute_internal_angles(theta_0, theta_1)
            x0=l0/2+l1*cos(theta_0)+l2*cos(alpha_0)
            y0=l1*sin(theta_0)+l2*sin(alpha_0)
            J1=self.J.subs([(theta0_sym,theta_0),(theta1_sym,theta_1)])
            J1=sympy.N(J1)
            J_inv=J1.pinv()
            delta=Matrix([[x-x0],[y-y0]])
            newtheta=0.01*J_inv*delta+cur_theta
            theta_0=newtheta[0]
            theta_1=newtheta[1]
            e=(x-x0)**2+(y-y0)**2
            #print(e)
            if e<1e-1:
                break
                
                
        
        
        
        
        
        

        #
        # Your code here that solves J as a matrix
        #

        return (theta_0, theta_1)

    ###
    ### Visualization functions
    ###
    def draw_leg(self,ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """

        theta2, theta1 = self.joint_0_pos, self.joint_1_pos
        link1, link2, width = l1, l2, l0

        (alpha2, alpha1)= self.compute_internal_angles(theta2,theta1)

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return (x, y)

        if ax == False:
            ax = plt.gca()
            ax.cla()

        ax.plot(-width / 2, 0, 'ok')
        ax.plot(width / 2, 0, 'ok')

        ax.plot([-width / 2, 0], [0, 0], 'k')
        ax.plot([width / 2, 0], [0, 0], 'k')

        ax.plot(-width / 2 + np.array([0, link1 * cos(theta1)]), [0, link1 * sin(theta1)], 'k')
        ax.plot(width / 2 + np.array([0, link1 * cos(theta2)]), [0, link1 * sin(theta2)], 'k')

        ax.plot(-width / 2 + link1 * cos(theta1) + np.array([0, link2 * cos(alpha1)]), \
                link1 * sin(theta1) + np.array([0, link2 * sin(alpha1)]), 'k');
        ax.plot(width / 2 + link1 * cos(theta2) + np.array([0, link2 * cos(alpha2)]), \
                np.array(link1 * sin(theta2) + np.array([0, link2 * sin(alpha2)])), 'k');

        ax.plot(width / 2 + link1 * cos(theta2) + link2 * cos(alpha2), \
                np.array(link1 * sin(theta2) + link2 * sin(alpha2)), 'ro');

        ax.axis([-(l1+l2), (l1+l2), -(l1+l2), (l1+l2)])
        ax.invert_yaxis()