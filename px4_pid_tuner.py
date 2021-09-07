#!/usr/bin/env python3
"""
Python script for system identification and PID tuning of PX4 PID loops using PX4 logs (ulog only).
@author Mohamed Abdelkader
@email mohamedashraf123@gmail.com
@year 2019

USE IT AT YOUR OWN RISK!
"""

# Argument parser
import argparse

# Mainly for numpy arrays...
import numpy as np

# Need FFT ?
import numpy.fft as fft

# For optimization
import scipy.optimize
import scipy.signal

# Read ulog files and prepare data
import px4tools as px
from collections import OrderedDict
import pandas as pd
from past.utils import old_div

# System identification
from sippy import *
from sippy import functionset as fset
from sippy import functionsetSIM as fsetSIM

# Control design
import control
import control.matlab as cnt

# To convert discrete-time transfer function to continuous-time
import harold

# For genetic optimization
from deap import base
from deap import creator
from deap import tools
import random

# Plotting
import matplotlib.pyplot as plt


###########################################################
"""
Data Preparation class
"""
class PrepData:
    def __init__(self, file=None, data_start_ind = 0, data_end_ind = -1, rolling_window_u = 100, rolling_window_y = 100, Ts = 0.001, data='roll', plot=True):
        """
        Constructor
        @file String of path to ulog file
        @dt desired sampling time
        @data_start_ind Starting index of data
        @data_end_ind Last index of data. -1 means up to last element
        @data type of data required {'roll', 'pitch', 'yaw'}
        @rolling_window moving average window length
        @Ts resampling times in seconds
        """
        
        self._raw_data = None
        if file is None:
            print("Path to ulg file is not provided")
            return
        
        self._raw_data = px.read_ulog(file)

        self._dt = Ts # will be calculated from data time  index, see process_data()
        self._Ts = Ts

        self._full_data_length = None # after resampling

        self._rolling_window_u = rolling_window_u # moving average window length for controller output filtering
        self._rolling_window_y = rolling_window_y # moving average window length for plant output filtering

        self._data_start_ind = data_start_ind
        self._data_end_ind = data_end_ind

        data_str = ['roll', 'pitch', 'yaw']
        if data not in data_str:
            print("data is not among ['roll', 'pitch', 'yaw']. Defaulting to 'roll' ")
            data = 'roll'
            
        self._data_name = data

        # Returned variables
        self._t = None # time vector
        self._u_raw = None # plant input data
        self._y_raw = None # plant output data
        self._u_filtered = None
        self._y_filtered = None

        self._trimmed_raw_data = None # between _data_start_ind and _data_end_ind

        self._plot = plot

    def process_data(self):
        """
        Returns processed data (time t,input u, output y) for the selected data type ['roll', 'pitch', 'yaw']
        Current processing:
            * resampling (mostly upsampling) to get rid of certain noise frequency
            * rolling window (moving) average

        TODO: make some analysis to advise if data is suitable for sysid. Something similar to MATLAB function advice()
        https://www.mathworks.com/help/ident/ref/advice.html
        """
        
        if self._raw_data is None:
            print("No data available!")
            return

        print('Processing data for '+self._data_name)

        # Extract topics of interest and make data the same length as the controller output t_actuator_controls_0_0
        d_concat = self._raw_data.concat( topics=['t_actuator_controls_0_0','t_vehicle_angular_velocity_0'],on="t_actuator_controls_0_0")

        # Find _dt from one of t_actuator_controls_0_0 fields e.g. t_actuator_controls_0_0__f_control_0_
        #t = (d_concat['t_actuator_controls_0_0__f_control_0_'].index.total_seconds() * 1e3).values
        #t_diff = np.diff(t)
        #dt_ms = int(t_diff.sum()/len(t_diff))
        #print("dt_ms", dt_ms)

        # or compute dt from user input (_Ts)
        dt_ms = int(self._Ts *1000)
        dt_ms_str = str(dt_ms)+'L'

        self._dt= dt_ms/1000.

        # Resample e with dt_ms
        df_rs = d_concat.resample(dt_ms_str).mean().interpolate() #method='spline', order=2s
        df_rs.index = [i * self._dt for i in range(len(df_rs.index))]
        self._full_data_length = len(df_rs.index)

        self._trimmed_raw_data = df_rs[[
                            't_actuator_controls_0_0__f_control_0_','t_vehicle_angular_velocity_0__f_xyz_0_',
                            't_actuator_controls_0_0__f_control_1_','t_vehicle_angular_velocity_0__f_xyz_1_',
                            't_actuator_controls_0_0__f_control_2_','t_vehicle_angular_velocity_0__f_xyz_2_']]

        self._trimmed_raw_data.rename(columns={'t_actuator_controls_0_0__f_control_0_':'ATTC_Roll',
                                't_actuator_controls_0_0__f_control_1_':'ATTC_Pitch',
                                't_actuator_controls_0_0__f_control_2_':'ATTC_Yaw',
                                't_vehicle_angular_velocity_0__f_xyz_0_':'rollspeed',
                                't_vehicle_angular_velocity_0__f_xyz_1_':'pitchspeed',
                                't_vehicle_angular_velocity_0__f_xyz_2_':'yawspeed'}, 
                        inplace=True)

        self._full_data_length = len(df_rs.index)
        # Time steps
        Time = np.array([i * self._dt for i in range(self._full_data_length)])
        Time = np.expand_dims(Time, axis=0)

        # Rate controller output is stored in the following fields
        # roll_rate control output: t_actuator_controls_0_0__f_control_0_
        # pitch_rate control output: t_actuator_controls_0_0__f_control_1_
        # yaw_rate control output: t_actuator_controls_0_0__f_control_2_
        
        if self._data_name == 'roll':
            data_str_u = 't_actuator_controls_0_0__f_control_0_'
            data_str_y = 't_vehicle_angular_velocity_0__f_xyz_0_'
        elif self._data_name == 'pitch':
            data_str_u = 't_actuator_controls_0_0__f_control_1_'
            data_str_y = 't_vehicle_angular_velocity_0__f_xyz_1_'
        elif self._data_name == 'yaw':
            data_str_u = 't_actuator_controls_0_0__f_control_2_'
            data_str_y = 't_vehicle_angular_velocity_0__f_xyz_2_'

        raw_u = df_rs[data_str_u] # rate controller output
        raw_u_v = raw_u.values # ndarray
        raw_u_bias = raw_u.rolling(self._rolling_window_u, min_periods=1,center=True).mean() # min_periods=1 used to avoid NaNs in the first elements
        #raw_u_filtered = raw_u - raw_u_bias
        raw_u_filtered = raw_u_bias
        raw_u_filtered_v = raw_u_filtered.values # ndarray
        raw_u_filtered_v = np.expand_dims(raw_u_filtered_v, axis=0) 
        raw_u_v = np.expand_dims(raw_u_v, axis=0)

        #Feedback signals
        # roll speed: t_vehicle_attitude_0__f_rollspeed
        # pitch speed: t_vehicle_attitude_0__f_pitchspeed
        # yaw speed: t_vehicle_attitude_0__f_yawspeed

        raw_feedback = df_rs[data_str_y]
        raw_feedback_v = raw_feedback.values # ndarray
        #raw_feedback_acc = raw_feedback.diff().fillna(df_rs[data_str_y])/ self._dt
        raw_feedback_bias = raw_feedback.rolling(self._rolling_window_y, min_periods=1,center=True).mean()
        #raw_feedback_filtered = raw_feedback - raw_feedback_bias
        raw_feedback_filtered = raw_feedback_bias
        raw_feedback_filtered_v = raw_feedback_filtered.values # ndarray
        raw_feedback_filtered_v = np.expand_dims(raw_feedback_filtered_v, axis=0)
        raw_feedback_v = np.expand_dims(raw_feedback_v, axis=0)


        # Sanity checks
        if self._data_end_ind > self._full_data_length or self._data_end_ind < 0 :
            self._data_end_ind = self._full_data_length

        if self._data_start_ind > self._full_data_length or self._data_end_ind < 0:
            self._data_start_ind = 0

        # Time
        self._t = Time #[:,self._data_start_ind:self._data_end_ind]

        # Plant raw I/O
        self._u_raw  = raw_u_v #[:,self._data_start_ind:self._data_end_ind]
        self._y_raw = raw_feedback_v #[:,self._data_start_ind:self._data_end_ind]
        # Plant filtered I/O
        self._u_filtered = raw_u_filtered_v #[:,self._data_start_ind:self._data_end_ind]
        self._y_filtered = raw_feedback_filtered_v #[:,self._data_start_ind:self._data_end_ind]

        if self._plot:
            ax1 = plt.subplot(2, 2, 1)
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._y_raw[0,self._data_start_ind:self._data_end_ind])
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._y_filtered[0,self._data_start_ind:self._data_end_ind])
            plt.xlabel("Time[s]")
            plt.ylabel(self._data_name+" speed feedback [rad/s]")
            plt.title(self._data_name+' speed feedback: Raw vs. Filtered')
            plt.legend(['Raw', 'Filtered'])
            plt.grid(True)

            plt.subplot(2, 2, 2, sharex=ax1)
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._u_raw[0,self._data_start_ind:self._data_end_ind])
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._u_filtered[0,self._data_start_ind:self._data_end_ind])
            plt.xlabel("Time[s]")
            plt.ylabel(self._data_name+" speed control [rad/s]")
            plt.title(self._data_name+' rate controller Output: Raw vs. Filtered')
            plt.legend(['Raw', 'Filtered'])
            plt.grid(True)

            plt.subplot(2, 2, 3, sharex=ax1)
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._y_raw[0,self._data_start_ind:self._data_end_ind])
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._u_raw[0,self._data_start_ind:self._data_end_ind])
            plt.xlabel("Time[s]")
            plt.ylabel(self._data_name+" rate [rad/s]")
            plt.title(self._data_name+' rate raw input and output')
            plt.legend(['Output', 'Input'])
            plt.grid(True)
            
            plt.subplot(2, 2, 4, sharex=ax1)
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._y_filtered[0,self._data_start_ind:self._data_end_ind])
            plt.plot(self._t[0,self._data_start_ind:self._data_end_ind], self._u_filtered[0,self._data_start_ind:self._data_end_ind])
            plt.xlabel("Time[s]")
            plt.ylabel(self._data_name+" rate [rad/s]")
            plt.title(self._data_name+' rate filtered input and output')
            plt.legend(['Output', 'Input'])
            plt.grid(True)

            
            plt.show()

    def get_data(self):
        if self._raw_data is None:
            print("Run process_data() first!")

        # return self._trimmed_raw_data.iloc[self._data_start_ind : self._data_end_ind, :]

        return self._full_data_length, self._t[:,self._data_start_ind:self._data_end_ind], self._u_filtered[:,self._data_start_ind:self._data_end_ind], self._y_filtered[:,self._data_start_ind:self._data_end_ind], self._dt

    def set_param(self, start_ind = 0, end_ind=-1):
        self._data_start_ind = start_ind
        self._data_end_ind = end_ind



###########################################################
"""
System identification class
"""
class PX4SYSID:
    """
    Uses SIPPY package to do sys ID and provide a 2nd order system in continuous time domain
    """
    def __init__(self, t, u, y, use_subspace =True, subspace_method='N4SID', subspace_p=250, Ts=0.001, u_name='input', y_name='ouput', plot = True, verbose = False):
        """
        @t: array of time samples [seconds]
        @u input array
        @y output array
        @use_subspace: Uses N4SID if True, otherwises it uses ARMAX
        @subspace_method subspace method in case use_subspace=True. Methods are N4SID, CVA, MOESP
        @subspace_p see SIPPY docs for explanation
        @Ts: Data sampling time
        """
        self._use_subspace = use_subspace
        self._subspace_p = subspace_p
        self._subspace_method = subspace_method
        self._Ts = Ts

        self._u = u
        self._y = y
        self._t = t # time vector

        self._u_name = u_name
        self._y_name = y_name

        # Reuturned identified system
        self._A, self._B, self._C, self._D = None, None, None, None # state space
        self._sys_tf = None # transfer function
        self._zeros = None
        self._fitness = 0.  # How much the id system fits the actual output


        self._plot = plot
        self._verbose = verbose

        self.do_sys_id()

    def do_sys_id(self):
        num_poles = 2
        num_zeros = 1
        if not self._use_subspace:
            method = 'ARMAX'
            #sys_id = system_identification(self._y.T, self._u[0], method, IC='BIC', na_ord=[0, 5], nb_ord=[1, 5], nc_ord=[0, 5], delays=[0, 5], ARMAX_max_iterations=300, tsample=self._Ts, centering='MeanVal')
            sys_id = system_identification(self._y.T, self._u[0], method, ARMAX_orders=[num_poles,1,1,0],ARMAX_max_iterations=300, tsample=self._Ts, centering='MeanVal')

            print(sys_id.G)
            
            if self._verbose:
                print(sys_id.G)
                print("System poles of discrete G: ",cnt.pole(sys_id.G))


            # Convert to continuous tf
            G = harold.Transfer(sys_id.NUMERATOR,sys_id.DENOMINATOR, dt=self._Ts)
            G_cont = harold.undiscretize(G, method='zoh')
            self._sys_tf = G_cont
            self._A, self._B, self._C, self._D = harold.transfer_to_state(G_cont, output='matrices')

            if self._verbose:
                print("Continuous tf:" , G_cont)
            
        
            # Convert to state space, because ARMAX gives transfer function
            ss_roll = cnt.tf2ss(sys_id.G)
            A = np.asarray(ss_roll.A)
            B = np.asarray(ss_roll.B)
            C = np.asarray(ss_roll.C)
            D = np.asarray(ss_roll.D)
            if self._verbose:
                print(ss_roll)

            # simulate identified system using input from data
            xid, yid = fsetSIM.SS_lsim_process_form(A, B, C, D, self._u)
            y_error = self._y - yid
            self._fitness = 1 - (y_error.var()/self._y.var())**2
            if self._verbose:
                print("Fittness %", self._fitness*100)

            if self._plot:
                plt.figure(1)
                plt.plot(self._t[0], self._y[0])
                plt.plot(self._t[0],yid[0])
                plt.xlabel("Time")
                plt.title("Time response Y(t)=U*G(t)")
                plt.legend([self._y_name, self._y_name+'_identified: '+'{:.3f} fitness'.format(self._fitness)])
                plt.grid()
                plt.show()

        else:
            sys_id = system_identification(self._y, self._u, self._subspace_method, SS_fixed_order=num_poles, SS_p=self._subspace_p, SS_f=50, tsample=self._Ts, SS_A_stability=True, centering='MeanVal')
            #sys_id = system_identification(self._y, self._u, self._subspace_method, SS_orders=[1,10], SS_p=self._subspace_p, SS_f=50, tsample=self._Ts, SS_A_stability=True, centering='MeanVal')
            if self._verbose:
                print("x0", sys_id.x0)
                print("A",sys_id.A)
                print("B",sys_id.B)
                print("C",sys_id.C)
                print("D",sys_id.D)

            A = sys_id.A
            B = sys_id.B
            C = sys_id.C
            D = sys_id.D

            # Get discrete transfer function from state space
            sys_tf = cnt.ss2tf(A, B, C, D)
            if self._verbose:
                print("TF ***in z domain***", sys_tf)

            # Get numerator and denominator
            (num,den) = cnt.tfdata(sys_tf)

            # Convert to continuous tf
            G = harold.Transfer(num,den, dt=self._Ts)
            if self._verbose:
                print(G)
            G_cont = harold.undiscretize(G, method='zoh')
            self._sys_tf = G_cont
            self._A, self._B, self._C, self._D = harold.transfer_to_state(G_cont, output='matrices')
            if self._verbose:
                print("Continuous tf:" , G_cont)

            # get zeros
            tmp_tf = cnt.ss2tf(self._A, self._B, self._C, self._D)
            self._zeros = cnt.zero(tmp_tf)

            # simulate identified system using discrete system
            xid, yid = fsetSIM.SS_lsim_process_form(A, B, C, D, self._u, sys_id.x0)
            y_error = self._y - yid
            self._fitness = 1 - (y_error.var()/self._y.var())**2
            if self._verbose:
                print("Fittness %", self._fitness*100)

            if self._plot:
                plt.figure(1)
                plt.plot(self._t[0], self._y[0])
                plt.plot(self._t[0],yid[0])
                plt.xlabel("Time")
                plt.title("Time response Y(t)=U*G(t)")
                plt.legend([self._y_name, self._y_name+'_identified: '+'{:.3f} fitness'.format(self._fitness)])
                plt.grid()
                plt.show()

    def get_data(self):
        return self._A, self._B, self._C, self._D, self._sys_tf, self._fitness, self._zeros


###########################################################
"""
LQR-based PID design class
"""
class PX4PIDDesign:
    """
    Class for LQR-based PID design. Currently, for a 2nd order system only
    Reference: https://ieeexplore.ieee.org/document/4242954
    """
    def __init__(self, A, B, C, D, q=[1., 1. , 1.], r = 1.0, dt=0.001 , verobse=False):
        self._A = A
        self._B = B
        self._C = C
        self._D = D

        # LQR weights
        self._q = q
        self._r = r

        # Calculated gains
        self._K = None

        self._dt = dt

        self._verbose = verobse

    def get_gains(self):
        if self._K is None:
            print("Gains are not computed yet.")

        return self._K
        

    def set_weights(self, q=[1.0, 1.0, 1.0], r=1.0):
        self._q = q
        self._r = r

    def pid_design(self):

        n_x = self._A.shape[0] # number of sates
        n_u = self._B.shape[1] # number of inputs
        n_y = self._C.shape[0] # number of outputs

        # Augmented state system (for LQR)
        A_lqr = np.block([ [ self._A , np.zeros( (n_x,n_y) )],
                            [self._C, np.zeros((n_y,n_y))]
                        ])
        B_lqr = np.block([
                            [self._B],
                            [np.zeros((n_y,1))]
                        ])

        # Define Q,R
        Q = np.diag(self._q)
        R = np.array([self._r])
        # Solve for P in continuous-time algebraic Riccati equation (CARE)
        #print("A_lqr shape", A_lqr.shape)
        #print("Q shape",Q.shape)
        (P,L,F) = cnt.care(A_lqr, B_lqr, Q, R)

        if self._verbose:
            print("P matrix", P)
            print("Feedback gain", F)

        # Calculate Ki_bar, Kp_bar
        Kp_bar = np.array( [ F[0][0:n_x] ] )
        Ki_bar = np.array( [ F[0][n_x:] ] )
        
        if self._verbose:
            print("Kp_bar", Kp_bar)
            print("Ki_bar", Ki_bar)

        # Calculate the PID kp kd gains
        C_bar = np.block([
                            [ self._C ],
                            [ self._C.dot(self._A) - (self._C.dot(self._B)).dot(Kp_bar) ]
                        ])

        if self._verbose:
            print("C_bar", C_bar)
            print("C_bar shape", C_bar.shape)

        # Calculate PID kp ki gains
        kpd = Kp_bar.dot(np.linalg.inv(C_bar))
        if self._verbose:
            print("kpd: ",kpd, "with shape: ", kpd.shape)
        kp = kpd[0][0]
        kd = kpd[0][1]
        # ki gain
        ki = (1. + kd* self._C.dot(self._B)).dot(Ki_bar)
        self._K = [kp, ki[0][0], kd]

        G_plant = cnt.ss2tf(self._A, self._B, self._C, self._D)

        # create PID transfer function
        d_tc = 1./125. # Derivative time constant at nyquist freq
        p_tf = self._K[0]
        i_tf = self._K[1]*cnt.tf([1],[1,0])
        d_tf = self._K[2]*cnt.tf([1,0],[d_tc,1]) 
        pid_tf = cnt.parallel(p_tf, i_tf, d_tf)
        open_loop_tf = cnt.series(pid_tf, G_plant)
        closedLoop_tf = cnt.feedback(open_loop_tf,1)

        if self._verbose:
            print(" *********** PID gains ***********")
            print("kp: ", self._K[0] )
            print("ki: ",  self._K[1])
            print("kd: ", self._K[2])
            print(" *********************************")

        return pid_tf, closedLoop_tf

    def step_response(self):

        if self._verbose:
            print("[PID Design] Calculating PID gains")
        pid_tf, closedLoop_tf = self.pid_design()

        

        end_time = 5  # [s]
        npts = int(old_div(end_time, self._dt)) + 1
        T = np.linspace(0, end_time, npts)
        yout, T = cnt.step(closedLoop_tf, T)

        u, _, _ = cnt.lsim(pid_tf,1-yout,T)

        return T, u, yout

             
    def plotStepResponse(self):
        """
        Plots the step response of the closed loop, using gains in K.
        """

        T, u, yout = self.step_response()

        fig, ((ax1),(ax2)) = plt.subplots(2,1)
        ax1.plot(T, yout)
        ax1.set_title('Step Response')
        ax2.plot(T, u)
        ax2.set_title('Controller Output')
        plt.show()


###########################################################
"""
GA class
"""
class GeneticOptimization:
    def __init__(self, A,B,C,D, num_pop = 50, num_gen = 10, w1=0.5,w2=0.5, dt=0.001, verbose=False):
        """
        @param G Plant transfer function
        @param dt Sampling time
        @param num_pop populaion size
        @param num_gen Maximum number of generations
        """
        self._weight_min = 0.5
        self._weight_max = 1000.0

        self._w1 = w1
        self._w2 = w2

        self._dt = dt

        self._num_pop = num_pop
        self._num_gen = num_gen

        # Best solution (computed)
        self._best_sol = None

        self._verbose = verbose

        self._pid = PX4PIDDesign(A, B, C, D, dt=dt, verobse=False)

    def weight_value(self):
        """
        @return K: random values of weights within gain bounds
        """
        return random.uniform(self._weight_min, self._weight_max)

    def run_ga(self):
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMin)

        toolbox = base.Toolbox()

        # Weight generator 
        #                      define 'gain_value' to be an attribute ('gene')
        #                      which corresponds to non-negative real numbers sampled uniformly
        #                      from the range [0,100]
        toolbox.register("gain_value", self.weight_value)

        # Structure initializers
        #                         define 'individual' to be an individual
        #                         consisting of 4 'weight_value' elements ('genes')
        toolbox.register("individual", tools.initRepeat, creator.Individual, 
            toolbox.gain_value, 4)

        # define the population to be a list of individuals
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)

        #----------
        # Operator registration
        #----------
        # register the goal / fitness function
        toolbox.register("evaluate", self.evaluatePID)

        # register the crossover operator
        toolbox.register("mate", tools.cxTwoPoint)

        # register a mutation operator with a probability to
        # flip each attribute/gene of 0.05
        toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)

        # operator for selecting individuals for breeding the next
        # generation: each individual of the current generation
        # is replaced by the 'fittest' (best) of three individuals
        # drawn randomly from the current generation.
        toolbox.register("select", tools.selTournament, tournsize=3)

        # create an initial population of 300 individuals (where
        # each individual is a list of integers)
        pop = toolbox.population(n=self._num_pop)

        # CXPB  is the probability with which two individuals
        #       are crossed
        #
        # MUTPB is the probability for mutating an individual
        CXPB, MUTPB = 0.5, 0.2
        
        print(" [GA] Start of evolution")
        
        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit
        if self._verbose:
            print(" [GA] Evaluated %i individuals" % len(pop))

        # Extracting all the fitnesses of 
        fits = [ind.fitness.values[0] for ind in pop]

        # Variable keeping track of the number of generations
        g = 0
        
        # Begin the evolution
        print(" [GA] evaluating {} generations".format(self._num_gen))
        while g < self._num_gen:
            # A new generation
            g = g + 1
            print(" [GA] -------------------------- Generation %i --------------------------" % g)
            
            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))
        
            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):

                # cross two individuals with probability CXPB
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)

                    # fitness values of the children
                    # must be recalculated later
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:

                # mutate an individual with probability MUTPB
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values
        
            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            if self._verbose:
                print(" [GA] Evaluated %i individuals" % len(invalid_ind))
            
            # The population is entirely replaced by the offspring
            pop[:] = offspring
            
            # Gather all the fitnesses in one list and print the stats
            fits = [ind.fitness.values[0] for ind in pop]
            
            length = len(pop)
            mean = sum(fits) / length
            sum2 = sum(x*x for x in fits)
            std = abs(sum2 / length - mean**2)**0.5
            
            if self._verbose:
                print(" [GA] Min %s" % min(fits))
                print(" [GA] Max %s" % max(fits))
                print(" [GA] Avg %s" % mean)
                print(" [GA] Std %s" % std)
        
        print(" [GA] -- End of (successful) evolution --")
        
        best_ind = tools.selBest(pop, 1)[0]
        self._best_sol = best_ind
        #print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))
        q = [best_ind[0], best_ind[1], best_ind[2]]
        r = best_ind[3]
        print("optimal q", q)
        print("optimal r", r)

        return q, r

    def evaluatePID(self, W):
        """
        Computes cost associated with system response (e.g. step response) of the closed loop, using LQR weights W.
        The cost to minimize is the time domain performance index defined in Equaiton 22 in
        # https://www.sciencedirect.com/science/article/pii/S0307904X12005197 
        # sum_t(w_1 * t * err_t^2 + w_2 * u_t^2) * dt
        # err = setpoint_t - ouput_t

        @param W array of  4 LQR weights, W[0:3] = [q1, q2, q3]; W[3] = R
        @param G plant continuous-time transfer function
        @param Q scalar, state weight
        @param R scalar, input weight
        @param dt time step [sec] for control input discretization

        @return J cost >= 0
        """

        q = [W[0], W[1], W[2]]
        r = W[3]
        # Simulate step response with PID
        self._pid.set_weights(q = q, r=r)
        try:
            self._pid.pid_design()
            t, u, yout = self._pid.step_response()

            J = self._dt*sum(t * self._w1 * (1-yout)**2 + self._w2 * u**2)
        except:
            J=1e9

        return J,

    def get_gains(self):
        if self._best_sol is None:
            print("Run optimization first!")

        q = [self._best_sol[0], self._best_sol[1], self._best_sol[2]]
        r = self._best_sol[3]
        # Simulate step response with PID
        self._pid.set_weights(q = q, r=r)
        self._pid.pid_design()

        return self._pid.get_gains()

def main(args):

    AUTO_DATA_SEL = args["autoSelect"]

    ################## Prepare date ##################
    dt = float(args["samplingTime"])
    start_t, end_t = float(args["start"]), float(args["end"])
    start_i = int(start_t/dt)
    if end_t < 0.0:
        end_i = -1
    else:
        end_i = int(end_t/dt)
    data  = PrepData( file=args["file"], data_start_ind = start_i, data_end_ind = end_i, rolling_window_y=50, rolling_window_u=50, Ts = dt, data=args["axis"], plot=True)
    data.process_data()
    full_data_len, t, u, y, dt = data.get_data()

    if args["showDataOnly"]:
        return

    if not AUTO_DATA_SEL:
        sysid = PX4SYSID(t, u, y, use_subspace =True, subspace_method='N4SID', Ts=dt, u_name='input', y_name='ouput', subspace_p=400, plot = True)
        best_A, best_B, best_C, best_D, tf, best_fit, zeros = sysid.get_data()
        print("Fitness", best_fit)
        print("Identified Transfer function: ", tf)
    else:
        # batch size
        batch_size_s = float(args["batchSize"]) # in seoconds
        batch_size = int(batch_size_s/dt)  # in number of points

        if full_data_len < batch_size:
            print("Not enough data points. Get longer logs")
            return

        # find how many batches are available in full data
        n_batches = int(full_data_len/batch_size)
        print("Found {} batches".format(n_batches))


        ################## SYS ID ##################
        best_fit = fit = None
        best_A, best_B, best_C, best_D = None, None, None, None
        best_t, best_u, best_y = None, None, None
        best_zero = np.inf
        subspace_p_increment = 50
        n_intervals = (600 - subspace_p_increment)/subspace_p_increment
        for batch_i in range(n_batches):
            print("Processing batch# {}".format(batch_i))

            start_i = batch_i * batch_size
            end_i = start_i + batch_size
            data.set_param(start_ind=start_i, end_ind=end_i)
            full_data_len, t, u, y, dt = data.get_data()
            best_p = 0
            for i in range(int(n_intervals)):
                p = (i+1)*subspace_p_increment
                try: 
                    sysid = PX4SYSID(t, u, y, use_subspace =True, subspace_method='N4SID', Ts=dt, u_name='input', y_name='ouput', subspace_p=p, plot = False)
                    A,B,C,D,_, fit, zeros = sysid.get_data()
                except:
                    print("Bad batch...... skipping")
                    break

                if (best_fit is None or fit > best_fit) and min(zeros) < 0.0 : # best_zero:
                    best_A, best_B, best_C, best_D = A, B, C, D
                    best_t, best_u, best_y = t, u, y
                    best_fit = fit
                    best_zero = min(zeros)
                    best_p = p

        print("Best fit = {}".format(best_fit))

        sysid = PX4SYSID(best_t, best_u, best_y, use_subspace =True, subspace_method='N4SID', Ts=dt, u_name='best_input', y_name='best_ouput', subspace_p=best_p, plot = True)
        _,_,_,_,tf, _, _ = sysid.get_data()
        print("Identified Transfer function: ", tf)

    if best_fit < 0.75 or np.isnan(best_fit):
        print(" [SYSID] WARNING Poor fitting %s. Try different data window with more dynamics!" %best_fit)
        print("Not proceeding with control tuning.")
        return
    
    ################## PID design ##################
    print(" --------------- Finding optimal gains using Genetic Optimization ---------------")
    ga = GeneticOptimization(best_A, best_B, best_C, best_D, num_pop = 25, num_gen = 10, w1=0.7, w2=0.3, dt=dt)
    q,r = ga.run_ga()

    print("optimal {} rate PID gains {}".format(args["axis"], ga.get_gains()))

    pid = PX4PIDDesign(best_A, best_B, best_C, best_D)
    pid.set_weights(q,r)
    pid.plotStepResponse()  
    

def str2bool(v):
    """
    To be passed to argparse for parsing True/False commandline arguments
    """
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.') 

if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()

    ap.add_argument("-f", "--file", required=True,
        help="Path to ulog file.")
    ap.add_argument("-sd", "--showDataOnly", type=str2bool, required=False,
        help=" Show data only with no SYS ID or PID design. ", default=False)
    ap.add_argument("-a", "--axis", required=False,
        help=" 'roll', 'pitch', 'yaw' ", default="pitch")
    ap.add_argument("-as", "--autoSelect", type=str2bool, required=False,
        help=" Automatic selection of data batch for system identificaion ", default=True)
    ap.add_argument("-bs", "--batchSize", required=False,
        help=" Batch length in seconds ", default=5.0)
    ap.add_argument("-s", "--start", required=False,
        help=" Start time in seconds, if auto batch selection is false", default=1.0)
    ap.add_argument("-e", "--end", required=False,
        help=" End time in seconds, if auto batch selection is false", default=-1.0)
    ap.add_argument("-dt", "--samplingTime", required=False,
        help=" Data sampling time in seconds", default=0.004)

    args = vars(ap.parse_args())


    main(args)
