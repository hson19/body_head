import numpy as np
from numpy import dot,eye
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from filterpy.kalman import ExtendedKalmanFilter
from scipy.spatial.transform import Rotation as R
import pandas as pd
import scipy.linalg as linalg
from copy import deepcopy
from matplotlib.colors import ListedColormap


class KalmanFilterPos:
    """Kalman filter for position class"""
    def __init__(self,deltatime,x0):
        # self.deltatime = deltatime
        self.kalman_filter = KalmanFilter(dim_x=6, dim_z=3)
        self.kalman_filter.F = np.array([[1,deltatime,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,deltatime,0,0],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,deltatime],
                            [0,0,0,0,0,1]])
        self.kalman_filter.B = np.array([[0.5*deltatime**2,0,0],
                          [deltatime,0,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0]])
        self.kalman_filter.H = np.array([[1,0,0,0,0,0],
                          [0,0,1,0,0,0],
                          [0,0,0,0,1,0]])
        # self.kalman_filter.H = np.eye(6)
        self.kalman_filter.Q = np.eye(6)*0.1
        self.kalman_filter.R = np.eye(3)*10e-1 # To change ?? 
        self.kalman_filter.P = np.eye(6)*0.1
        self.kalman_filter.x = x0
        self.result = []
        self.result.append(x0)
    def kalman_update(self,z=None):
        if z is not None:
            self.kalman_filter.update(z)
        self.result.append(self.kalman_filter.x)
        return None

    def kalman_predict(self,x=None,u=None):
        if x is not None:
            self.kalman_filter.x = x
        if u is not None:
            self.kalman_filter.predict(u)
        else:
            self.kalman_filter.predict()
    
    def update_pred_matrix(self,deltatime):
        self.kalman_filter.F=np.array([[1,deltatime,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,deltatime,0,0],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,deltatime],
                            [0,0,0,0,0,1]])
        self.kalman_filter.B = np.array([[0.5*deltatime**2,0,0],
                          [deltatime,0,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0]])
        return None
class KalmanFilterQuat:
    def __init__(self,Omega,dt,x0):
        self.kalman_filter = KalmanFilter(dim_x=4, dim_z=4)
        Wx, Wy, Wz = Omega
        Omega = np.array([
            [0, -Wx, -Wy, -Wz],
            [Wx, 0, -Wz, Wy],
            [Wy, Wz, 0, -Wx],
            [Wz, -Wy, Wx, 0]
        ])
        self.F=np.eye(4) + 0.5 * dt * Omega
        self.kalman_filter.F = np.eye(4)
        self.kalman_filter.H = np.eye(4)
        self.kalman_filter.Q = np.eye(4)*0.1
        self.kalman_filter.R = np.eye(4)*0.1
        self.kalman_filter.P = np.eye(4)*0.1
        self.kalman_filter.x = x0
        self.result = []
        self.result.append(x0)   

    def kalman_update(self,z=None):
        if z is not None:
            self.kalman_filter.update(z)
            self.kalman_filter.x = unit(self.kalman_filter.x)
        self.result.append(self.kalman_filter.x)
        return None 
    def kalman_predict(self,x=None,u=None):
        if x is not None:
            self.kalman_filter.x = x
        if u is not None:
            self.kalman_filter.predict(u)
        else:
            self.kalman_filter.predict()
        self.kalman_filter.x = unit(self.kalman_filter.x)
    def update_pred_matrix(self,dt,Gyro):
        Wx, Wy, Wz = Gyro
        Omega = np.array([
            [0, -Wx, -Wy, -Wz],
            [Wx, 0, -Wz, Wy],
            [Wy, Wz, 0, -Wx],
            [Wz, -Wy, Wx, 0]
        ])        
        self.F=np.eye(4) + 0.5 * dt * Omega

class DataIterable():
    def __init__(self,file,card):
        self.data = ReadData(file,card)
        self.i = 0
        self.card = card
    def __iter__(self):
        return self
    def __next__(self):
        if self.i < len(self.data):
            self.i += 1
            return self.data[self.i-1]
        else:
            raise StopIteration
    def __str__(self):
        print(self.card)
        return None
    

class ModelKalmanFilter(ExtendedKalmanFilter):
    def __init__(self,dim_x,dim_z,dim_u=0):
        super().__init__(dim_x,dim_z,dim_u)
    def predict(self, u=0):
        """Predict next state (prior) using the Kalman filter state propagation
        equations.

        Parameters
        ----------

        u : np.array
            Optional control vector. If non-zero, it is multiplied by B
            to create the control input into the system.
        """

        self.P = dot(self.F, self.P).dot(self.F.T) + self.Q

        # save prior
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)
        
    def update(self, z, H,nToID, R=None,residual=np.subtract):
        """ Performs the update innovation of the extended Kalman filter.

        Parameters
        ----------

        z : np.array
            measurement for this step.
            If `None`, posterior is not computed

        H : Matrix H
        R : np.array, scalar, or None
            Optionally provide R to override the measurement noise for this
            one call, otherwise  self.R will be used.

        args : tuple, optional, default (,)
            arguments to be passed into HJacobian after the required state
            variable. for robot localization you might need to pass in
            information about the map and time of day, so you might have
            `args=(map_data, time)`, where the signature of HCacobian will
            be `def HJacobian(x, map, t)`

        hx_args : tuple, optional, default (,)
            arguments to be passed into Hx function after the required state
            variable.

        residual : function (z, z2), optional
            Optional function that computes the residual (difference) between
            the two measurement vectors. If you do not provide this, then the
            built in minus operator will be used. You will normally want to use
            the built in unless your residual computation is nonlinear (for
            example, if they are angles)
        """

        if z is None:
            self.z = np.array([[None]*self.dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return
        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = eye(self.dim_z) * R

        if np.isscalar(z) and self.dim_z == 1:
            z = np.asarray([z], float)

        PHT = dot(self.P, H.T)
        self.S = dot(H, PHT) + R
        self.SI = linalg.inv(self.S)
        self.K = PHT.dot(self.SI)

        hx = H.dot(self.x)
        self.y = residual(z, hx)
        self.x = self.x + dot(self.K, self.y)
        self.x[nToID['hq0']:nToID['hq3']+1]=unit(self.x[nToID['hq0']:nToID['hq3']+1])
        self.x[nToID['bq0']:nToID['bq3']+1]=unit(self.x[nToID['bq0']:nToID['bq3']+1])
        self.x[nToID['aq0']:nToID['aq3']+1]=unit(self.x[nToID['aq0']:nToID['aq3']+1])
        self.x[nToID['fq0']:nToID['fq3']+1]=unit(self.x[nToID['fq0']:nToID['fq3']+1])

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        I_KH = self._I - dot(self.K, H)
        self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)

        # set to None to force recompute
        self._log_likelihood = None
        self._likelihood = None
        self._mahalanobis = None

        # save measurement and posterior state
        self.z = deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()


def get_quat_pred_matrix(dt,gyro):
    """ gyro is a 3 element array of gyro measurements in radians per second
    dt is a time intreval in seconds 
    Returns the quaternions prediction matrix for a global to local quaternion
    """
    wx, wy, wz = gyro
    omega = np.array([
        [0, -wx, -wy, -wz],
        [wx, 0, -wz, wy],
        [wy, wz, 0, -wx],
        [wz, -wy, wx, 0]
    ])
    return np.eye(4) + 0.5 * dt * omega
def get_position_pred_matrix(dt,state_dict,ntoid):
    """
    dt is a time intreval in seconds
    Returns the position prediction matrix for a global to local position
    TODO : Acceleration should be in the local ??
    """
    """
    rotation of acceleration is defined by:
    a_north = a_x*2*(q0^2+q1^2-0.5)-a_y*2(q1*q2+q0*q3)-a_z*2(q1*q3-q0*q2)
    a_up = a_x*2(q1*q2+q0*q3)+a_y*2(q0^2+q2^2-0.5)+a_z*2(q2*q3-q0*q1)
    a_east = a_x*2(q1*q3-q0*q2)+a_y*2(q2*q3+q0*q1)+a_z*2(q0^2+q3^2-0.5)
    """
    q0,q1,q2,q3 = state_dict['hq0'],state_dict['hq1'],state_dict['hq2'],state_dict['hq3']
    ax,ay,az = state_dict['h_ax'],state_dict['h_ay'],state_dict['h_az']
    da_northdq0 = 4*q0*ax - 2*q3*ay - 2*q2*az
    da_northdq1 = 4*q1*ax + 2*q2*ay + 2*q3*az
    da_northdq2 = 2*q1*ay + 2*q0*az
    da_northdq3 = -2*q0*ay + 2*q1*az
    da_northdax = 2*q0**2 + 2*q1**2 - 1
    da_northday = 2*q1*q2 - 2*q0*q3
    da_northdaz = 2*q1*q3 + 2*q0*q2

    da_updq0 = 2*q3*ax + 4*q0*ay - 2*q1*az
    da_updq1 = 2*q2*ax - 2*q0*az
    da_updq2 = 2*q1*ax + 4*q2*ay + 2*q3*az
    da_updq3 = 2*q0*ax + 2*q2*az
    da_updax = 2*q1*q2 + 2*q0*q3
    da_upday = 2*(q0**2 + q2**2 - 1)
    da_updaz = 2*q2*q3 - 2*q0*q1

    da_eastdq0 = -2*q2*ax + 2*q1*ay + 4*q0*az
    da_eastdq1 = 2*q3*ax + 2*q0*ay 
    da_eastdq2 = -2*q0*ax + 2*q3*ay 
    da_easddq3 = 2*q1*ax + 2*q2*ay + 4*q3*az
    da_eastdax = 2*q1*q3 - 2*q0*q2
    da_eastday = 2*q2*q3 + 2*q0*q1
    da_eastdaz = 2*q0**2 + 2*q3**2 - 1

    p_north = np.zeros(len(state_dict.keys()))
    p_north[ntoid['hpx']]=1
    p_north[ntoid['h_vx']]=dt

    p_north[ntoid['hq0']]=da_northdq0*dt
    p_north[ntoid['hq1']]=da_northdq1*dt
    p_north[ntoid['hq2']]=da_northdq2*dt
    p_north[ntoid['hq3']]=da_northdq3*dt
    p_north[ntoid['h_ax']]=da_northdax*dt
    p_north[ntoid['h_ay']]=da_northday*dt
    p_north[ntoid['h_az']]=da_northdaz*dt

    p_up = np.zeros(len(state_dict.keys()))
    p_up[ntoid['hpy']]=1
    p_up[ntoid['h_vy']]=dt

    p_up[ntoid['hq0']]=da_updq0*dt
    p_up[ntoid['hq1']]=da_updq1*dt
    p_up[ntoid['hq2']]=da_updq2*dt
    p_up[ntoid['hq3']]=da_updq3*dt
    p_up[ntoid['h_ax']]=da_updax*dt
    p_up[ntoid['h_ay']]=da_upday*dt
    p_up[ntoid['h_az']]=da_updaz*dt

    p_east = np.zeros(len(state_dict.keys()))
    p_east[ntoid['hpz']]=1
    p_east[ntoid['h_vz']]=dt

    p_east[ntoid['hq0']]=da_eastdq0*dt
    p_east[ntoid['hq1']]=da_eastdq1*dt
    p_east[ntoid['hq2']]=da_eastdq2*dt
    p_east[ntoid['hq3']]=da_easddq3*dt
    p_east[ntoid['h_ax']]=da_eastdax*dt
    p_east[ntoid['h_ay']]=da_eastday*dt
    p_east[ntoid['h_az']]=da_eastdaz*dt

    v_north = np.zeros(len(state_dict.keys()))
    v_north[ntoid['h_vx']]=1
    v_north[ntoid['h_ax']]=dt

    v_up = np.zeros(len(state_dict.keys()))
    v_up[ntoid['h_vz']]=1
    v_up[ntoid['h_ay']]=dt

    v_east = np.zeros(len(state_dict.keys()))
    v_east[ntoid['h_vz']]=1
    v_east[ntoid['h_az']]=dt


    return (p_north,p_up,p_east,v_north,v_up,v_east)
def get_jacobian_position_matrix(dt,state,ntoid):
    """dt is a time intreval in seconds
    state is list representing the state of the system
    ntoid is a dictionary mapping the name of the state to its index in the 
    state list
    returns the jacobian matrix of the sensor position
    """
    n=len(state.keys())
    # Xbody
    # x= 1- 0.6*hq0 - 0.6*hq1 (Derivative) 
    body_x_position=np.zeros(n)
    body_x_position[ntoid['hq0']]+=-0.6*state['hq0']
    body_x_position[ntoid['hq1']]+=-0.6*state['hq1']
    body_x_position[ntoid['hpx']]+=1

    # Ybody
    # dy/dX = 1 -0.3 hq3 + 0.3 hq2
    body_y_position=np.zeros(n)
    body_y_position[ntoid['hpy']]+=1
    body_y_position[ntoid['hq0']]=-0.3*state['hq3']
    body_y_position[ntoid['hq1']]=-0.3*state['hq2']
    body_y_position[ntoid['hq2']]=-0.3*state['hq1']
    body_y_position[ntoid['hq3']]=-0.3*state['hq0']

    # Zbody
    #  dz/dX = 1 + 0.3 hq2 -0.3 hq3 + 0.3 hq0 - 0.3 hq1
    body_z_position=np.zeros(n)
    body_z_position[ntoid['hpz']]+=1
    body_z_position[ntoid['hq0']]=0.3*state['hq2']
    body_z_position[ntoid['hq1']]=-0.3*state['hq3']
    body_z_position[ntoid['hq2']]=0.3*state['hq0']
    body_z_position[ntoid['hq3']]=-0.3*state['hq1']

    # X arm
    # x= 1 +0.3 hq3 -0.3 hq2 - 0.3 hq1 + 0.3 hq0
    arm_x_position=np.zeros(n)
    arm_x_position[ntoid['hpx']]+=1
    arm_x_position[ntoid['hq0']]=0.3*state['hq3']
    arm_x_position[ntoid['hq1']]=-0.3*state['hq2']
    arm_x_position[ntoid['hq2']]=-0.3*state['hq1']
    arm_x_position[ntoid['hq3']]=0.3*state['hq0']
    
    # x = -0.6 aq0 - 0.6 aq2
    arm_x_position[ntoid['aq0']]=-0.6*state['aq0']
    arm_x_position[ntoid['aq1']]=-0.6*state['aq1']

    # Y arm
    # y= 1 -0.6 hq0 - 0.6 hq2
    arm_y_position=np.zeros(n)
    arm_y_position[ntoid['hpy']]+=1
    arm_y_position[ntoid['hq0']]=-0.6*state['hq0']
    arm_y_position[ntoid['hq2']]=-0.6*state['hq2']

    # y= -0.3 aq3 - 0.3 aq2 -0.3 aq1 - 0.3 aq0
    arm_y_position[ntoid['aq0']]=-0.3*state['aq3']
    arm_y_position[ntoid['aq1']]=-0.3*state['aq2']
    arm_y_position[ntoid['aq2']]=-0.3*state['aq1']
    arm_y_position[ntoid['aq3']]=-0.3*state['aq0']

    # Z arm
    # z= 1 -0.6 hq1 - 0.6 q0Heead - 0.6 hq3 - 0.6 hq2
    arm_z_position=np.zeros(n)
    arm_z_position[ntoid['hpz']]+=1
    arm_z_position[ntoid['hq0']]=-0.3*state['hq1']
    arm_z_position[ntoid['hq1']]=-0.3*state['hq0']
    arm_z_position[ntoid['hq2']]=-0.3*state['hq3']
    arm_z_position[ntoid['hq3']]=-0.3*state['hq2']

    # z= 0.3 aq2 - 0.3 aq3 + 0.3 aq0 - 0.3 aq1
    arm_z_position[ntoid['aq0']]=0.3*state['aq2']
    arm_z_position[ntoid['aq1']]=-0.3*state['aq3']
    arm_z_position[ntoid['aq2']]=0.3*state['aq0']
    arm_z_position[ntoid['aq3']]=-0.3*state['aq1']

    forearm_x_position=arm_x_position.copy()
    forearm_x_position[ntoid['aq0']]-=0.6*state['aq0']
    forearm_x_position[ntoid['aq1']]-=0.6*state['aq1']

    forearm_x_position[ntoid['fq0']]-=0.6*state['fq0']
    forearm_x_position[ntoid['fq1']]-=0.6*state['fq1']

    forearm_y_position=arm_y_position.copy()
    forearm_y_position[ntoid['aq0']]-=0.3*state['aq3']
    forearm_y_position[ntoid['aq1']]-=0.3*state['aq2']
    forearm_y_position[ntoid['aq2']]-=0.3*state['aq1']
    forearm_y_position[ntoid['aq3']]-=0.3*state['aq0']

    forearm_y_position[ntoid['fq0']]-=0.3*state['fq3']
    forearm_y_position[ntoid['fq1']]-=0.3*state['fq2']
    forearm_y_position[ntoid['fq2']]-=0.3*state['fq1']
    forearm_y_position[ntoid['fq3']]-=0.3*state['fq0']


    forearm_z_position=arm_z_position.copy()
    forearm_z_position[ntoid['aq0']]=0.3*state['aq2']
    forearm_z_position[ntoid['aq1']]=-0.3*state['aq3']
    forearm_z_position[ntoid['aq2']]=0.3*state['aq0']
    forearm_z_position[ntoid['aq3']]=-0.3*state['aq1']

    forearm_z_position[ntoid['fq0']]=0.3*state['fq2']
    forearm_z_position[ntoid['fq1']]=-0.3*state['fq3']
    forearm_z_position[ntoid['fq2']]=0.3*state['fq0']
    forearm_z_position[ntoid['fq3']]=-0.3*state['fq1']
    return (body_x_position,body_y_position,body_z_position,arm_x_position,
            arm_y_position,arm_z_position,forearm_x_position,forearm_y_position,forearm_z_position)
def jacobian_state_transition_matrix(state,dt):
    """state is the current state
    dt is the time interval
    returns the jacobian matrix of the state transition
    """
    acc=[0,0,0]
    state_dict,ntoid,idton = state_to_state_dict(state)
    matrix=np.zeros((len(state),len(state)))
    (p_north,p_up,p_east,v_north,v_up,v_east)=get_position_pred_matrix(dt,state_dict,ntoid)
    matrix[ntoid['hpx']]=p_north
    matrix[ntoid['hpy']]=p_up
    matrix[ntoid['hpz']]=p_east
    matrix[ntoid['h_vx']]=v_north
    matrix[ntoid['h_vy']]=v_up
    matrix[ntoid['h_vz']]=v_east

    body_x_position,body_y_position,body_z_position,arm_x_position,arm_y_position,arm_y_position,forearm_x_position,forearm_y_position,forearm_z_position=get_jacobian_position_matrix(dt,state_dict,ntoid)
    matrix[ntoid['hq0']:ntoid['hq3']+1,ntoid['hq0']:ntoid['hq3']+1]=get_quat_pred_matrix(dt,[state_dict["h_gx"],state_dict["h_gy"],state_dict["h_gz"]])
    matrix[ntoid['h_gx']:ntoid['h_gz']+1,ntoid['h_gx']:ntoid['h_gz']+1]=np.eye(3)

    matrix[ntoid['bq0']:ntoid['bq3']+1,ntoid['bq0']:ntoid['bq3']+1]=get_quat_pred_matrix(dt,[state_dict["gBpx"],state_dict["gBpy"],state_dict["gBpz"]])
    matrix[ntoid['bpx']]=body_x_position
    matrix[ntoid['bpy']]=body_y_position
    matrix[ntoid['bpz']]=body_z_position
    matrix[ntoid['gBpx']:ntoid['gBpz']+1,ntoid['gBpx']:ntoid['gBpz']+1]=np.eye(3)

    matrix[ntoid['aq0']:ntoid['aq3']+1,ntoid['aq0']:ntoid['aq3']+1]=get_quat_pred_matrix(dt,[state_dict["gApx"],state_dict["gApy"],state_dict["gApz"]])
    matrix[ntoid['apx']]=arm_x_position
    matrix[ntoid['apy']]=arm_y_position
    matrix[ntoid['apz']]=arm_y_position
    matrix[ntoid['gApx']:ntoid['gApz']+1,ntoid['gApx']:ntoid['gApz']+1]=np.eye(3)

    matrix[ntoid['fq0']:ntoid['fq3']+1,ntoid['fq0']:ntoid['fq3']+1]=get_quat_pred_matrix(
                                dt,[state_dict["gApx"],state_dict["gApy"],state_dict["gApz"]])
    matrix[ntoid['fpx']]=forearm_x_position
    matrix[ntoid['fpy']]=forearm_y_position
    matrix[ntoid['fpz']]=forearm_z_position
    matrix[ntoid['gFpx']:ntoid['gFpz']+1,ntoid['gFpx']:ntoid['gFpz']+1]=np.eye(3)
    print_matrix(matrix,idton)
    cmap = ListedColormap(['white', 'black'])

    # Create a binary mask of the data
    # binary_data = (matrix != 0).astype(int)    
    # plt.imshow(binary_data,cmap=cmap)
    # plt.show()
    return matrix
def kalman_filtering(file):
    """File is the file containing the data
    returns the results of the kalman filtering
    """
    data_head,data_body,data_arm,data_forearm=(DataIterable(file,"head"),
                                               DataIterable(file,"body"),
                                               DataIterable(file,"arm"),
                                               DataIterable(file,"forearm"))
    state_dict,ntoid,idton=init_state(data_head.data,data_body.data,
                                      data_arm.data,data_forearm.data)
    sensor=min([data_head,data_body,data_arm,data_forearm],
               key=lambda x: x.data['time'][x.i] if x.data is not None else np.inf)
    last_time=sensor.data['time'][sensor.i]
    current_time=last_time
    sensor.i+=1
    next_sensor=min(data_head,data_body,data_arm,data_forearm,
                    key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
    dt=(next_sensor.data['time'][next_sensor.i]-last_time)/1000

    state=[state_dict[idton[i]] for i in range(len(idton))]
    filter=ModelKalmanFilter(dim_x=len(state),dim_z=9)
    filter.x=state
    results=[]

    JF=jacobian_state_transition_matrix(state,dt)
    # JF = np.eye(len(state))*0.001
    len_head=len(data_head.data) if data_head.data is not None else 0
    len_body=len(data_body.data) if data_body.data is not None else 0
    len_arm=len(data_arm.data) if data_arm.data is not None else 0
    len_forearm=len(data_forearm.data) if data_forearm.data is not None else 0
    while (data_head.i<len_head or data_body.i<len_body 
           or data_arm.i<len_arm or data_forearm.i<len_forearm):
        sensor=min(data_head,data_body,data_arm,data_forearm,
                   key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
        current_time=sensor.data['time'][sensor.i]
        dt= (current_time-last_time)/1000

        JF=jacobian_state_transition_matrix(state,dt)
        # JF = np.eye(len(state))*0.1

        filter.F=JF
        state_dict,ntoid,idton=state_to_state_dict(filter.x)
        filter.x=state_prediction(state_dict,dt,ntoid)
        filter.predict()

        if sensor.card=="head":
            # print("Head")
            qn= [1,0,0,0]
            accl = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],
                    sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],
                   sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,accl,mag))  # quaternion from local to global
            z=np.append(z,[accl[0],accl[1],accl[2]])
            z=np.append(z,[sensor.data['gx'][sensor.i],
                           sensor.data['gy'][sensor.i],
                           sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((10,len(state)))
            state_to_measurement_matrix[0:4,ntoid['hq0']:ntoid['hq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:7,ntoid['h_ax']:ntoid['h_az']+1]=np.eye(3)
            state_to_measurement_matrix[7:,ntoid['h_gx']:ntoid['h_gz']+1]=np.eye(3)
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(10)*0.001)
        elif sensor.card=="body":
            qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],
                   sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],
                   sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],
                           sensor.data['gy'][sensor.i],
                           sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['bq0']:ntoid['bq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:,ntoid['gBpx']:ntoid['gBpz']+1]=np.eye(3)
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(7)*0.001)
        elif sensor.card=="arm":
            qn=[1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],
                   sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],
                   sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],
                           sensor.data['gy'][sensor.i],
                           sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['aq0']:ntoid['aq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:,ntoid['gApx']:ntoid['gApz']+1]=np.eye(3)
            print("Arm quaternion before update",
                  filter.x[ntoid['aq0']:ntoid['aq3']+1],"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            filter.update(z,state_to_measurement_matrix,
                          ntoid,R=np.eye(7)*0.001)
            print("Arm quaternion after update",filter.x[ntoid['aq0']:ntoid['aq3']+1])
            print(" Measurement",z)

        elif sensor.card=="forearm":
            qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],
                   sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],
                   sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag))
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['fq0']:ntoid['fq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:,ntoid['gFpx']:ntoid['gFpz']+1]=np.eye(3)
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(7)*0.001)
        sensor.i+=1
        results.append(filter.x)
        last_time=current_time
    return results,ntoid
def state_prediction(state_dict,dt,ntoid):
    """state_dict is a dictionary containing the state of the system
    dt is the time interval
    ntoid is a dictionary mapping the name of the state to its index in the state list
    returns the predicted state of the system
    """
    state_head = [state_dict['hpx'],state_dict["h_vx"],state_dict['hpy'],
                  state_dict['h_vy'],state_dict['hpz'],state_dict['h_vz']]
    quat_head= [state_dict['hq0'],state_dict['hq1'],state_dict['hq2'],
                state_dict['hq3']]
    conj_quat_head=quat_conjugate(quat_head)
    kalman_filter_quat_head=KalmanFilterQuat([state_dict['h_gx'],
                                              state_dict['h_gy'],state_dict['h_gz']],dt,conj_quat_head)
    kalman_filter_pos_head=KalmanFilterPos(dt,state_head)
    
    accl_head=[state_dict['h_ax'],state_dict['h_ay'],state_dict['h_az']]
    accg_head= quat_transformation(quat_head,accl_head)
    accg_head[1]-=9.81 # remove gravity
    kalman_filter_quat_head.update_pred_matrix(dt,[state_dict['h_gx'],
                                                   state_dict['h_gy'],state_dict['h_gz']])
    kalman_filter_pos_head.kalman_predict(u=accg_head)
    kalman_filter_quat_head.kalman_predict()
    kalman_filter_pos_head.kalman_update()
    kalman_filter_quat_head.kalman_update()

    state_head = kalman_filter_pos_head.result[-1]
    new_pos_head= state_head.reshape(3,2)[:,0]
    new_v_head = state_head.reshape(3,2)[:,1]
    new_quat_head = quat_conjugate(kalman_filter_quat_head.result[-1])
    
    conj_quat_body= quat_conjugate([state_dict['bq0'],state_dict['bq1'],
                                    state_dict['bq2'],state_dict['bq3']])
    kalman_filter_quat_body=KalmanFilterQuat([state_dict['gBpx'],
                                              state_dict['gBpy'],
                                              state_dict['gBpz']],
                                              dt,conj_quat_body)
    new_pos_body = new_pos_head + quat_transformation(quat_head,[-0.3,0,0])
    kalman_filter_quat_body.update_pred_matrix(dt,[state_dict['gBpx'],
                                                   state_dict['gBpy'],
                                                   state_dict['gBpz']])
    kalman_filter_quat_body.kalman_predict()
    kalman_filter_quat_body.kalman_update()
    new_quat_body = quat_conjugate(kalman_filter_quat_body.result[-1])

    quat_arm = [state_dict['aq0'],state_dict['aq1'],state_dict['aq2'],state_dict['aq3']]
    pos_arm = (new_pos_head + quat_transformation(quat_head,[0,-0.25,0])
               + quat_transformation(quat_arm,[-0.15,0,0]))

    conj_quat_arm = quat_conjugate(quat_arm)
    print("Arm quaternion before predict",quat_arm)
    kalman_filter_quat_arm=KalmanFilterQuat([state_dict['gApx'],
                                             state_dict['gApy'],
                                             state_dict['gApz']],
                                             dt,conj_quat_arm)
    kalman_filter_quat_arm.update_pred_matrix(dt,[state_dict['gApx'],
                                                  state_dict['gApy'],
                                                  state_dict['gApz']])
    kalman_filter_quat_arm.kalman_predict()
    kalman_filter_quat_arm.kalman_update()

    new_quat_arm = quat_conjugate(kalman_filter_quat_arm.result[-1])
    print("Arm quaternion after predict",new_quat_arm)
    quat_forearm = [state_dict['fq0'],state_dict['fq1'],
                    state_dict['fq2'],state_dict['fq3']]
    pos_forearm = (pos_arm + quat_transformation(quat_arm,[-0.15,0,0]) 
    + quat_transformation(quat_forearm,[-0.15,0,0]))

    conj_quat_forearm=quat_conjugate(quat_forearm) # Global to Local
    kalman_filter_quat_forearm=KalmanFilterQuat([state_dict['gFpx'],
                                                 state_dict['gFpy'],
                                                 state_dict['gFpz']],
                                                 dt,conj_quat_forearm)
    kalman_filter_quat_forearm.update_pred_matrix(dt,[state_dict['gFpx'],
                                                      state_dict['gFpy'],
                                                      state_dict['gFpz']])
    kalman_filter_quat_forearm.kalman_predict()
    kalman_filter_quat_forearm.kalman_update()
    new_quat_forearm = quat_conjugate(kalman_filter_quat_forearm.result[-1])


    state={'hq0':new_quat_head[0],'hq1':new_quat_head[1],'hq2':new_quat_head[2],
           'hq3':new_quat_head[3],'hpx':new_pos_head[0],'hpy':new_pos_head[1],
           'hpz':new_pos_head[2],'h_vx':new_v_head[0],'h_vy':new_v_head[1],
           'h_vz':new_v_head[2],'h_ax':state_dict['h_ax'],
           'h_ay':state_dict['h_ay'],'h_az':state_dict['h_az'],
           'h_gx':state_dict['h_gx'],'h_gy':state_dict['h_gy'],
           'h_gz':state_dict['h_gz'],'bq0':new_quat_body[0],
           'bq1':new_quat_body[1],'bq2':new_quat_body[2],
           'bq3':new_quat_body[3],'bpx':new_pos_body[0],'bpy':new_pos_body[1],
           'bpz':new_pos_body[2],'gBpx':state_dict['gBpx'],
           'gBpy':state_dict['gBpy'],'gBpz':state_dict['gBpz'],
           'aq0':new_quat_arm[0],'aq1':new_quat_arm[1],'aq2':new_quat_arm[2],
           'aq3':new_quat_arm[3],'apx':pos_arm[0],'apy':pos_arm[1],
           'apz':pos_arm[2],'gApx':state_dict['gApx'],
           'gApy':state_dict['gApy'],'gApz':state_dict['gApz'],
           'fq0':new_quat_forearm[0],'fq1':new_quat_forearm[1],
           'fq2':new_quat_forearm[2],'fq3':new_quat_forearm[3],
           'fpx':pos_forearm[0],'fpy':pos_forearm[1],
           'fpz':pos_forearm[2],'gFpx':state_dict['gFpx'],
           'gFpy':state_dict['gFpy'],'gFpz':state_dict['gFpz']}
    new_state=np.zeros(len(state_dict.keys()))
    for key in state_dict.keys():
        new_state[ntoid[key]]=state[key]
    return new_state

def init_state(data_head,data_body,data_arm,data_forearm):
    """data_head is the data from the upper chest sensor
    data_body is the data from the abdomen sensor
    data_arm is the data from the arm sensor
    data_forearm is the data from the forearm sensor
    """
    state_dict,ntoid,idtoname=state_to_state_dict(np.zeros(46))
    quat_head = quat_conjugate(quat_observation([1, 0, 0, 0],
                                    [data_head['ax'][0], data_head['ay'][0],
                                    data_head['az'][0]], [data_head['mx'][0],
                                    data_head['my'][0],data_head['mz'][0]],mua=1))
    state_dict['hq0'],state_dict['hq1'],state_dict['hq2'],state_dict['hq3'] = quat_head
    if data_body is not None:
        quatBody = quat_conjugate(quat_observation([1, 0, 0, 0],[data_body['ax'][0],
                                        data_body['ay'][0],data_body['az'][0]],
                                        [data_body['mx'][0],data_body['my'][0],
                                         data_body['mz'][0]],mua=1))
    else:
        quatBody = quat_head
    state_dict['bq0'],state_dict['bq1'],state_dict['bq2'],state_dict['bq3'] = quatBody
    state_dict['bpx'],state_dict['bpy'],state_dict['bpz'] = [0,0,0] 
    + quat_transformation(quat_head,[-0.3,0,0])
    if data_arm is not None:
        quat_arm = quat_conjugate(quat_observation([1, 0, 0, 0],
                                    [data_arm['ax'][0], data_arm['ay'][0],data_arm['az'][0]],
                                    [data_arm['mx'][0],data_arm['my'][0],data_arm['mz'][0]],mua=1))
    else:
        quat_arm = quat_head
    state_dict['aq0'],state_dict['aq1'],state_dict['aq2'],state_dict['aq3'] = quat_arm
    pos_arm = [0,0,0] + quat_transformation(quat_head,[0,-0.15,0]) 
    + quat_transformation(quat_arm,[-0.15,0,0])
    state_dict['apx'],state_dict['apy'],state_dict['apz'] = pos_arm
    if data_forearm is not None:
        quat_forearm = quat_conjugate(quat_observation([1, 0, 0, 0],
                            [data_forearm['ax'][0],data_forearm['ay'][0],data_forearm['az'][0]],
                            [data_forearm['mx'][0],data_forearm['my'][0],data_forearm['mz'][0]],mua=1))
    else:
        quat_forearm = quat_head
    state_dict['fq0'],state_dict['fq1'],state_dict['fq2'],state_dict['fq3'] = quat_forearm
    state_dict['fpx'],state_dict['fpy'],state_dict['fpz'] = pos_arm 
    + quat_transformation(quat_arm,[-0.15,0,0]) + quat_transformation(quat_forearm,[-0.15,0,0])

    return state_dict,ntoid,idtoname
######################################################################################################
# Utilitlies function
######################################################################################################

def quat_observation(qn, acc, mag,mua=1):
    """
    qn: quaternion from local to global
    acc: Acceleration in local frame
    mag: Magnetic field in local frame
    mua : uncertainty in the gravity vector
    returns the observed quaternion from global to local
    """
    # TODO: Find appropriate value for MUA
    q1, q2, q3, q4 = qn
    quatgtol = np.array([q1, -q2, -q3, -q4])
    vg_hat = unit(quat_transformation(quatgtol, [0, -1,0 ]))

    vg = unit(-np.array(acc))
    na = unit(np.cross(vg_hat, vg))
    d_omega = np.arccos(np.dot(vg_hat, vg))
    
    q2_ae, q3_ae, q4_ae = np.sin(mua * d_omega / 2)* na
    q1_ae = np.cos(mua * d_omega / 2)
    q_a = unit(quat_product([q1_ae, q2_ae, q3_ae, q4_ae], quatgtol))
    q1_a, q2_a, q3_a, q4_a = q_a
    q_a_inv = np.array([q1_a, -q2_a, -q3_a, -q4_a])

    v_north_l = quat_transformation(q_a, [1,0, 0])
    v_m_xyz = quat_transformation(q_a_inv, mag)
    Bx, _, Bz = v_m_xyz
    v_m_xz = [Bx / np.sqrt(Bx**2 + Bz**2), 0, Bz / np.sqrt(Bx**2 + Bz**2)]

    v_m_xz_l = quat_transformation(q_a, v_m_xz)
    d_omeg_m = np.arccos(np.dot(v_north_l, v_m_xz_l))
    q2_me, q3_me, q4_me = np.sin(d_omeg_m / 2)*np.cross(v_north_l, v_m_xz_l)
    q1_me = np.cos(d_omeg_m / 2)
    q_me = [q1_me, q2_me, q3_me, q4_me]
    q_m = unit(quat_product( q_me,q_a))
    return unit(q_m)

def print_matrix(matrix,idton):
    """
    matrix is the matrix to print
    idton is a dictionary mapping the index to the name of the state

    """
    f= open("matrix.txt","w+")
    for i,row in enumerate(matrix):
        f.write(idton[i]+" ")
        for j, elem in enumerate(row):
            
            if elem!=0:
                f.write("\Fraction{d "+idton[i]+"}{d "+idton[j]+"}= "+str(elem)+" ")
        f.write("\n")
    f.close()
def state_to_state_dict(state):
    """state is the state of the system
    returns a dictionary mapping the name of the state to its value
    """
    names=['hq0','hq1','hq2','hq3','hpx','hpy','hpz','h_vx','h_vy','h_vz','h_ax','h_ay','h_az','h_gx','h_gy','h_gz']
    state_names_body=['bq0','bq1','bq2','bq3','bpx','bpy','bpz','gBpx','gBpy','gBpz']
    state_names_arm=['aq0','aq1','aq2','aq3','apx','apy','apz','gApx','gApy','gApz']
    state_names_forearm=['fq0','fq1','fq2','fq3','fpx','fpy','fpz','gFpx','gFpy','gFpz']
    names.extend(state_names_body)
    names.extend(state_names_arm)
    names.extend(state_names_forearm)
    state_dict={names[i]:state[i] for i in range(len(names))}
    ntoid={names[i]:i for i in range(len(names))}
    idtoname={i:names[i] for i in range(len(names))}
    return state_dict,ntoid,idtoname
def ReadData(file,card):
    """file is the path to the file
    card is the card of the sensor
    returns the sensor data form the card or none if the file does not exist"""
    # Read the data from the file
    col_names=['it','time','ax','ay','az','gx','gy','gz','mx','my','mz']
    try:
        data = pd.read_csv(file+"/nav3_body_head@"+card+".csv",names=col_names,header=None)
    except:
        return None
    return data
def quat_transformation(q, p):
    """ q is the quaternion, p is the point to transform
    returns the transformed point
    """
    quat_p=np.array([0,p[0],p[1],p[2]])
    q_inv = quat_conjugate(q)
    q_p = quat_product(q,quat_product(quat_p,q_inv))
    return q_p[1:]
def quat_conjugate(quat):
    """quat is the quaternion
    returns the conjugate of the quaternion
    """
    q1, q2, q3, q4 = quat
    return np.array([q1, -q2, -q3, -q4])
def unit(x):
    """x is the vector
    returns the unit vector"""
    return x/(np.linalg.norm(x))
def quat_product(q1, q2):
    """q1 and q2 are the quaternions
    returns the product of the quaternions"""
    q11, q12, q13, q14 = q1
    q21, q22, q23, q24 = q2
    return np.array([q11*q21 - q12*q22 - q13*q23 - q14*q24,
                        q11*q22 + q12*q21 + q13*q24 - q14*q23,
                        q11*q23 - q12*q24 + q13*q21 + q14*q22,
                        q11*q24 + q12*q23 - q13*q22 + q14*q21])

def averageDeltat(file,card):
    data=ReadData(file,card)
    mean=np.mean(np.diff(data['time']))/1000
    return mean
def averageDeltatE2(file,card):
    data=pd.read_csv(file+"/e2_body_head@"+card+".csv")
    #Select the second column
    data=data.iloc[:,1]
    #Calculate the difference between the time
    data=np.diff(data)
    #Calculate the mean
    mean=np.mean(data)/1000
    return mean
def averageDeltatE4(file,card):
    data=pd.read_csv(file+"/e4_body_head@"+card+".csv")
    #Select the second column
    data=data.iloc[:,1]
    #Calculate the difference between the time
    data=np.diff(data)
    #Calculate the mean
    mean=np.mean(data)/1000
    return mean
# Dt=averageDeltatE4("./measures","head")
# print("DT of experiment",Dt)
# print("DT of data",averageDeltat("./measures","head"))
def updateandmeasuretiem(file,card):
    data=pd.read_csv(file+"/e4_body_head@"+card+".csv")
    #Select the second column
    TimeData=data.iloc[:,1]
    Dtupdate=data.iloc[:,2]
    Dtmeasure=data.iloc[:,3]
    Dtupdate= np.mean(Dtupdate)/1000
    Dtmeasure= np.mean(Dtmeasure)/1000
    Dt= np.mean(np.diff(TimeData))/1000
    return Dtupdate,Dtmeasure,Dt
print("Update and measure time",updateandmeasuretiem("./measures","head"))
# file="measures"
# results,ntoid=kalman_filtering(file)
# # write the results in a file, in /measures/ folder
# f= open(file+"/result.csv","w+")
# for result in results:
#     for elem in result:
#         f.write(str(elem)+",")
#     f.write("\n")
# f.close()

# from Visualisation import live_animation
# live_animation(file+"/result.csv",ntoid,DEBUG=True,body=False,head=True,arm=False,forearm=False)

# file="measures"
# name=['it','t0','hq0', 'hq1', 'hq2', 'hq3', 'hpx', 'hpy', 'hpz', 'h_vx', 'h_vy', 'h_vz',
#                     'h_ax', 'h_ay', 'h_az', 'h_gx', 'h_gy', 'h_gz', 'bq0', 'bq1', 'bq2', 'bq3',
#                     'bpx', 'bpy', 'bpz', 'gBpx', 'gBpy', 'gBpz', 'aq0', 'aq1', 'aq2', 'aq3',
#                     'apx', 'apy', 'apz', 'gApx', 'gApy', 'gApz', 'fq0', 'fq1', 'fq2', 'fq3',
#                     'fpx', 'fpy', 'fpz', 'gFpx', 'gFpy', 'gFpz','time']
# ntoid={name[i]:i for i in range(len(name))}
# from python.Visualisation import live_animation
# live_animation(file+"/e4_body_head@head.csv",ntoid,DEBUG=True,body=True,head=True,arm=True,forearm=True)
# data = pd.read_csv('measures/e4_body_head@head.csv',header=None)

# data=data.iloc[-1] 
# line=[float(element.replace('[','').replace(']','').strip('\n')) if isinstance(element, str) else element for element in data]
# state_dict={}
# for id,elem in enumerate(line):
#     print(name[id],elem)
#     state_dict[name[id]]=elem
# print('state_dict',state_dict)
# print("norm quat :",np.linalg.norm([state_dict['hq0'],state_dict['hq1'],state_dict['hq2'],state_dict['hq3']]))
