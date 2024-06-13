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
        self.x[nToID['Hq0']:nToID['Hq3']+1]=unit(self.x[nToID['Hq0']:nToID['Hq3']+1])
        self.x[nToID['Bq0']:nToID['Bq3']+1]=unit(self.x[nToID['Bq0']:nToID['Bq3']+1])
        self.x[nToID['Aq0']:nToID['Aq3']+1]=unit(self.x[nToID['Aq0']:nToID['Aq3']+1])
        self.x[nToID['Fq0']:nToID['Fq3']+1]=unit(self.x[nToID['Fq0']:nToID['Fq3']+1])

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
def get_position_pred_matrix(dt):
    """
    dt is a time intreval in seconds
    Returns the position prediction matrix for a global to local position
    TODO : Acceleration should be in the local ??
    """
    return np.array([[1,0,0,dt,0,0,(dt**2)/2,0,0],
                    [0,1,0,0,dt,0,0,(dt**2)/2,0],
                    [0,0,1,0,0,dt,0,0,(dt**2)/2],
                    [0,0,0,1,0,0,dt,0,0],
                    [0,0,0,0,1,0,0,dt,0],
                    [0,0,0,0,0,1,0,0,dt],
                    [0,0,0,0,0,0,1,0,0],
                    [0,0,0,0,0,0,0,1,0],
                    [0,0,0,0,0,0,0,0,1]])
def get_body_position_matrix(dt,state,ntoid):
    """dt is a time intreval in seconds
    state is list representing the state of the system
    ntoid is a dictionary mapping the name of the state to its index in the state list
    returns the jacobian matrix of the sensor position
    """
    n=len(state.keys())
    # Xbody
    # x= 1- 0.6*Hq0 - 0.6*Hq1 (Derivative) 
    body_x_position=np.zeros(n)
    body_x_position[ntoid['Hq0']]+=-0.6*state['Hq0']
    body_x_position[ntoid['Hq1']]+=-0.6*state['Hq1']
    body_x_position[ntoid['Hpx']]+=1

    # Ybody
    # dy/dX = 1 -0.3 Hq3 + 0.3 Hq2
    body_y_position=np.zeros(n)
    body_y_position[ntoid['Hpy']]+=1
    body_y_position[ntoid['Hq0']]=-0.3*state['Hq3']
    body_y_position[ntoid['Hq1']]=-0.3*state['Hq2']
    body_y_position[ntoid['Hq2']]=-0.3*state['Hq1']
    body_y_position[ntoid['Hq3']]=-0.3*state['Hq0']

    # Zbody
    #  dz/dX = 1 + 0.3 Hq2 -0.3 Hq3 + 0.3 Hq0 - 0.3 Hq1
    body_z_position=np.zeros(n)
    body_z_position[ntoid['Hpz']]+=1
    body_z_position[ntoid['Hq0']]=0.3*state['Hq2']
    body_z_position[ntoid['Hq1']]=-0.3*state['Hq3']
    body_z_position[ntoid['Hq2']]=0.3*state['Hq0']
    body_z_position[ntoid['Hq3']]=-0.3*state['Hq1']

    # X arm
    # x= 1 +0.3 Hq3 -0.3 Hq2 - 0.3 Hq1 + 0.3 Hq0
    arm_x_position=np.zeros(n)
    arm_x_position[ntoid['Hpx']]+=1
    arm_x_position[ntoid['Hq0']]=0.3*state['Hq3']
    arm_x_position[ntoid['Hq1']]=-0.3*state['Hq2']
    arm_x_position[ntoid['Hq2']]=-0.3*state['Hq1']
    arm_x_position[ntoid['Hq3']]=0.3*state['Hq0']
    
    # x = -0.6 Aq0 - 0.6 Aq2
    arm_x_position[ntoid['Aq0']]=-0.6*state['Aq0']
    arm_x_position[ntoid['Aq1']]=-0.6*state['Aq1']

    # Y arm
    # y= 1 -0.6 Hq0 - 0.6 Hq2
    arm_y_position=np.zeros(n)
    arm_y_position[ntoid['Hpy']]+=1
    arm_y_position[ntoid['Hq0']]=-0.6*state['Hq0']
    arm_y_position[ntoid['Hq2']]=-0.6*state['Hq2']

    # y= -0.3 Aq3 - 0.3 Aq2 -0.3 Aq1 - 0.3 Aq0
    arm_y_position[ntoid['Aq0']]=-0.3*state['Aq3']
    arm_y_position[ntoid['Aq1']]=-0.3*state['Aq2']
    arm_y_position[ntoid['Aq2']]=-0.3*state['Aq1']
    arm_y_position[ntoid['Aq3']]=-0.3*state['Aq0']

    # Z arm
    # z= 1 -0.6 Hq1 - 0.6 q0Heead - 0.6 Hq3 - 0.6 Hq2
    arm_z_position=np.zeros(n)
    arm_z_position[ntoid['Hpz']]+=1
    arm_z_position[ntoid['Hq0']]=-0.3*state['Hq1']
    arm_z_position[ntoid['Hq1']]=-0.3*state['Hq0']
    arm_z_position[ntoid['Hq2']]=-0.3*state['Hq3']
    arm_z_position[ntoid['Hq3']]=-0.3*state['Hq2']

    # z= 0.3 Aq2 - 0.3 Aq3 + 0.3 Aq0 - 0.3 Aq1
    arm_z_position[ntoid['Aq0']]=0.3*state['Aq2']
    arm_z_position[ntoid['Aq1']]=-0.3*state['Aq3']
    arm_z_position[ntoid['Aq2']]=0.3*state['Aq0']
    arm_z_position[ntoid['Aq3']]=-0.3*state['Aq1']

    forearm_x_position=arm_x_position.copy()
    forearm_x_position[ntoid['Aq0']]-=0.6*state['Aq0']
    forearm_x_position[ntoid['Aq1']]-=0.6*state['Aq1']

    forearm_x_position[ntoid['Fq0']]-=0.6*state['Fq0']
    forearm_x_position[ntoid['Fq1']]-=0.6*state['Fq1']

    forearm_y_position=arm_y_position.copy()
    forearm_y_position[ntoid['Aq0']]-=0.3*state['Aq3']
    forearm_y_position[ntoid['Aq1']]-=0.3*state['Aq2']
    forearm_y_position[ntoid['Aq2']]-=0.3*state['Aq1']
    forearm_y_position[ntoid['Aq3']]-=0.3*state['Aq0']

    forearm_y_position[ntoid['Fq0']]-=0.3*state['Fq3']
    forearm_y_position[ntoid['Fq1']]-=0.3*state['Fq2']
    forearm_y_position[ntoid['Fq2']]-=0.3*state['Fq1']
    forearm_y_position[ntoid['Fq3']]-=0.3*state['Fq0']


    forearm_z_position=arm_z_position.copy()
    forearm_z_position[ntoid['Aq0']]=0.3*state['Aq2']
    forearm_z_position[ntoid['Aq1']]=-0.3*state['Aq3']
    forearm_z_position[ntoid['Aq2']]=0.3*state['Aq0']
    forearm_z_position[ntoid['Aq3']]=-0.3*state['Aq1']

    forearm_z_position[ntoid['Fq0']]=0.3*state['Fq2']
    forearm_z_position[ntoid['Fq1']]=-0.3*state['Fq3']
    forearm_z_position[ntoid['Fq2']]=0.3*state['Fq0']
    forearm_z_position[ntoid['Fq3']]=-0.3*state['Fq1']
    return body_x_position,body_y_position,body_z_position,arm_x_position,arm_y_position,arm_z_position,forearm_x_position,forearm_y_position,forearm_z_position



def jacobian_state_transition_matrix(state,dt):
    """state is the current state
    dt is the time interval
    returns the jacobian matrix of the state transition
    """
    state_dict,ntoid,idton = state_to_state_dict(state)
    matrix=np.zeros((len(state),len(state)))
    dt=0.01
    body_x_position,body_y_position,body_z_position,arm_x_position,arm_y_position,arm_y_position,forearm_x_position,forearm_y_position,forearm_z_position=get_body_position_matrix(dt,state_dict,ntoid)
    matrix[ntoid['Hq0']:ntoid['Hq3']+1,ntoid['Hq0']:ntoid['Hq3']+1]=get_quat_pred_matrix(dt,[state_dict["gHpx"],state_dict["gHpy"],state_dict["gHpz"]])
    matrix[ntoid['Hpx']:ntoid['aHpz']+1,ntoid['Hpx']:ntoid['aHpz']+1]=get_position_pred_matrix(dt)
    matrix[ntoid['gHpx']:ntoid['gHpz']+1,ntoid['gHpx']:ntoid['gHpz']+1]=np.eye(3)

    matrix[ntoid['Bq0']:ntoid['Bq3']+1,ntoid['Bq0']:ntoid['Bq3']+1]=get_quat_pred_matrix(dt,[state_dict["gBpx"],state_dict["gBpy"],state_dict["gBpz"]])
    matrix[ntoid['Bpx']]=body_x_position
    matrix[ntoid['Bpy']]=body_y_position
    matrix[ntoid['Bpz']]=body_z_position
    matrix[ntoid['gBpx']:ntoid['gBpz']+1,ntoid['gBpx']:ntoid['gBpz']+1]=np.eye(3)

    matrix[ntoid['Aq0']:ntoid['Aq3']+1,ntoid['Aq0']:ntoid['Aq3']+1]=get_quat_pred_matrix(dt,[state_dict["gApx"],state_dict["gApy"],state_dict["gApz"]])
    matrix[ntoid['Apx']]=arm_x_position
    matrix[ntoid['Apy']]=arm_y_position
    matrix[ntoid['Apz']]=arm_y_position
    matrix[ntoid['gApx']:ntoid['gApz']+1,ntoid['gApx']:ntoid['gApz']+1]=np.eye(3)

    matrix[ntoid['Fq0']:ntoid['Fq3']+1,ntoid['Fq0']:ntoid['Fq3']+1]=get_quat_pred_matrix(dt,[state_dict["gApx"],state_dict["gApy"],state_dict["gApz"]])
    matrix[ntoid['Fpx']]=forearm_x_position
    matrix[ntoid['Fpy']]=forearm_y_position
    matrix[ntoid['Fpz']]=forearm_z_position
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
    data_head,data_body,data_arm,data_forearm=DataIterable(file,"head"),DataIterable(file,"body"),DataIterable(file,"arm"),DataIterable(file,"forearm")
    state_dict,ntoid,idton=init_state(data_head.data,data_body.data,data_arm.data,data_forearm.data)
    sensor=min([data_head,data_body,data_arm,data_forearm],key=lambda x: x.data['time'][x.i] if x.data is not None else np.inf)
    last_time=sensor.data['time'][sensor.i]
    current_time=last_time
    sensor.i+=1
    next_sensor=min(data_head,data_body,data_arm,data_forearm,key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
    dt=(next_sensor.data['time'][next_sensor.i]-last_time)/1000

    state=[state_dict[idton[i]] for i in range(len(idton))]
    filter=ModelKalmanFilter(dim_x=len(state),dim_z=9)
    filter.x=state
    results=[]

    # JF=jacobian_state_transition_matrix(state,dt)
    JF = np.eye(len(state))*0.001
    len_head=len(data_head.data) if data_head.data is not None else 0
    len_body=len(data_body.data) if data_body.data is not None else 0
    len_arm=len(data_arm.data) if data_arm.data is not None else 0
    len_forearm=len(data_forearm.data) if data_forearm.data is not None else 0
    while data_head.i<len_head or data_body.i<len_body or data_arm.i<len_arm or data_forearm.i<len_forearm:
        sensor=min(data_head,data_body,data_arm,data_forearm,key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
        current_time=sensor.data['time'][sensor.i]
        dt= (current_time-last_time)/1000

        # JF=jacobian_state_transition_matrix(state,dt)
        JF = np.eye(len(state))*0.1

        filter.F=JF
        state_dict,ntoid,idton=state_to_state_dict(filter.x)
        filter.x=state_prediction(state_dict,dt,ntoid)
        filter.predict()

        if sensor.card=="head":
            # print("Head")
            qn= [1,0,0,0]
            accl = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,accl,mag))  # quaternion from local to global
            z=np.append(z,[accl[0],accl[1],accl[2]])
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((10,len(state)))
            state_to_measurement_matrix[0:4,ntoid['Hq0']:ntoid['Hq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:7,ntoid['aHpx']:ntoid['aHpz']+1]=np.eye(3)
            state_to_measurement_matrix[7:,ntoid['gHpx']:ntoid['gHpz']+1]=np.eye(3)
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(10)*0.001)
        elif sensor.card=="body":
            qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['Bq0']:ntoid['Bq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:,ntoid['gBpx']:ntoid['gBpz']+1]=np.eye(3)
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(7)*0.001)


        elif sensor.card=="arm":
            qn=[1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['Aq0']:ntoid['Aq3']+1]=np.eye(4)
            state_to_measurement_matrix[4:,ntoid['gApx']:ntoid['gApz']+1]=np.eye(3)
            print("Arm quaternion before update",filter.x[ntoid['Aq0']:ntoid['Aq3']+1],"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            filter.update(z,state_to_measurement_matrix,ntoid,R=np.eye(7)*0.001)
            print("Arm quaternion after update",filter.x[ntoid['Aq0']:ntoid['Aq3']+1])
            print(" Measurement",z)

        elif sensor.card=="forearm":
            qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(quat_observation(qn,acc,mag))
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            state_to_measurement_matrix = np.zeros((7,len(state)))
            state_to_measurement_matrix[0:4,ntoid['Fq0']:ntoid['Fq3']+1]=np.eye(4)
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
    state_head = [state_dict['Hpx'],state_dict["vHpx"],state_dict['Hpy'],state_dict['vHpy'],state_dict['Hpz'],state_dict['vHpz']]
    quat_head= [state_dict['Hq0'],state_dict['Hq1'],state_dict['Hq2'],state_dict['Hq3']]
    conj_quat_head=quat_conjugate(quat_head)
    kalman_filter_quat_head=KalmanFilterQuat([state_dict['gHpx'],state_dict['gHpy'],state_dict['gHpz']],dt,conj_quat_head)
    kalman_filter_pos_head=KalmanFilterPos(dt,state_head)
    
    accl_head=[state_dict['aHpx'],state_dict['aHpy'],state_dict['aHpz']]
    accg_head= quat_transformation(quat_head,accl_head)
    accg_head[1]-=9.81 # remove gravity
    kalman_filter_quat_head.update_pred_matrix(dt,[state_dict['gHpx'],state_dict['gHpy'],state_dict['gHpz']])
    kalman_filter_pos_head.kalman_predict(u=accg_head)
    kalman_filter_quat_head.kalman_predict()
    kalman_filter_pos_head.kalman_update()
    kalman_filter_quat_head.kalman_update()

    state_head = kalman_filter_pos_head.result[-1]
    new_pos_head= state_head.reshape(3,2)[:,0]
    new_v_head = state_head.reshape(3,2)[:,1]
    new_quat_head = quat_conjugate(kalman_filter_quat_head.result[-1])
    
    conj_quat_body= quat_conjugate([state_dict['Bq0'],state_dict['Bq1'],state_dict['Bq2'],state_dict['Bq3']])
    kalman_filter_quat_body=KalmanFilterQuat([state_dict['gBpx'],state_dict['gBpy'],state_dict['gBpz']],dt,conj_quat_body)
    new_pos_body = new_pos_head + quat_transformation(quat_head,[-0.3,0,0])
    kalman_filter_quat_body.update_pred_matrix(dt,[state_dict['gBpx'],state_dict['gBpy'],state_dict['gBpz']])
    kalman_filter_quat_body.kalman_predict()
    kalman_filter_quat_body.kalman_update()
    new_quat_body = quat_conjugate(kalman_filter_quat_body.result[-1])

    quat_arm = [state_dict['Aq0'],state_dict['Aq1'],state_dict['Aq2'],state_dict['Aq3']]
    pos_arm = new_pos_head + quat_transformation(quat_head,[0,-0.25,0]) + quat_transformation(quat_arm,[-0.15,0,0])


    conj_quat_arm = quat_conjugate(quat_arm)
    print("Arm quaternion before predict",quat_arm)
    kalman_filter_quat_arm=KalmanFilterQuat([state_dict['gApx'],state_dict['gApy'],state_dict['gApz']],dt,conj_quat_arm)
    kalman_filter_quat_arm.update_pred_matrix(dt,[state_dict['gApx'],state_dict['gApy'],state_dict['gApz']])
    kalman_filter_quat_arm.kalman_predict()
    kalman_filter_quat_arm.kalman_update()

    new_quat_arm = quat_conjugate(kalman_filter_quat_arm.result[-1])
    print("Arm quaternion after predict",new_quat_arm)
    quat_forearm = [state_dict['Fq0'],state_dict['Fq1'],state_dict['Fq2'],state_dict['Fq3']]
    pos_forearm = pos_arm + quat_transformation(quat_arm,[-0.15,0,0]) + quat_transformation(quat_forearm,[-0.15,0,0])

    conj_quat_forearm=quat_conjugate(quat_forearm) # Global to Local
    kalman_filter_quat_forearm=KalmanFilterQuat([state_dict['gFpx'],state_dict['gFpy'],state_dict['gFpz']],dt,conj_quat_forearm)
    kalman_filter_quat_forearm.update_pred_matrix(dt,[state_dict['gFpx'],state_dict['gFpy'],state_dict['gFpz']])
    kalman_filter_quat_forearm.kalman_predict()
    kalman_filter_quat_forearm.kalman_update()
    new_quat_forearm = quat_conjugate(kalman_filter_quat_forearm.result[-1])


    state={'Hq0':new_quat_head[0],'Hq1':new_quat_head[1],'Hq2':new_quat_head[2],'Hq3':new_quat_head[3],'Hpx':new_pos_head[0],'Hpy':new_pos_head[1],'Hpz':new_pos_head[2],'vHpx':new_v_head[0],'vHpy':new_v_head[1],
           'vHpz':new_v_head[2],'aHpx':state_dict['aHpx'],'aHpy':state_dict['aHpy'],'aHpz':state_dict['aHpz'],'gHpx':state_dict['gHpx'],'gHpy':state_dict['gHpy'],'gHpz':state_dict['gHpz'],
           'Bq0':new_quat_body[0],'Bq1':new_quat_body[1],'Bq2':new_quat_body[2],'Bq3':new_quat_body[3],'Bpx':new_pos_body[0],'Bpy':new_pos_body[1],'Bpz':new_pos_body[2],'gBpx':state_dict['gBpx'],'gBpy':state_dict['gBpy'],
           'gBpz':state_dict['gBpz'],'Aq0':new_quat_arm[0],'Aq1':new_quat_arm[1],'Aq2':new_quat_arm[2],'Aq3':new_quat_arm[3],'Apx':pos_arm[0],'Apy':pos_arm[1],'Apz':pos_arm[2],'gApx':state_dict['gApx'],'gApy':state_dict['gApy'],
            'gApz':state_dict['gApz'],'Fq0':new_quat_forearm[0],'Fq1':new_quat_forearm[1],'Fq2':new_quat_forearm[2],'Fq3':new_quat_forearm[3],'Fpx':pos_forearm[0],'Fpy':pos_forearm[1],'Fpz':pos_forearm[2],
            'gFpx':state_dict['gFpx'],'gFpy':state_dict['gFpy'],'gFpz':state_dict['gFpz']}
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
    quat_head = quat_conjugate(quat_observation([1, 0, 0, 0],[data_head['ax'][0],data_head['ay'][0],data_head['az'][0]], [data_head['mx'][0],data_head['my'][0],data_head['mz'][0]],mua=1))
    state_dict['Hq0'],state_dict['Hq1'],state_dict['Hq2'],state_dict['Hq3'] = quat_head
    if data_body is not None:
        quatBody = quat_conjugate(quat_observation([1, 0, 0, 0],[data_body['ax'][0],data_body['ay'][0],data_body['az'][0]], [data_body['mx'][0],data_body['my'][0],data_body['mz'][0]],mua=1))
    else:
        quatBody = quat_head
    state_dict['Bq0'],state_dict['Bq1'],state_dict['Bq2'],state_dict['Bq3'] = quatBody
    state_dict['Bpx'],state_dict['Bpy'],state_dict['Bpz'] = [0,0,0] + quat_transformation(quat_head,[-0.3,0,0])
    if data_arm is not None:
        quat_arm = quat_conjugate(quat_observation([1, 0, 0, 0],[data_arm['ax'][0],data_arm['ay'][0],data_arm['az'][0]], [data_arm['mx'][0],data_arm['my'][0],data_arm['mz'][0]],mua=1))
    else:
        quat_arm = quat_head
    state_dict['Aq0'],state_dict['Aq1'],state_dict['Aq2'],state_dict['Aq3'] = quat_arm
    pos_arm = [0,0,0] + quat_transformation(quat_head,[0,-0.15,0]) + quat_transformation(quat_arm,[-0.15,0,0])
    state_dict['Apx'],state_dict['Apy'],state_dict['Apz'] = pos_arm
    if data_forearm is not None:
        quat_forearm = quat_conjugate(quat_observation([1, 0, 0, 0],[data_forearm['ax'][0],data_forearm['ay'][0],data_forearm['az'][0]], [data_forearm['mx'][0],data_forearm['my'][0],data_forearm['mz'][0]],mua=1))
    else:
        quat_forearm = quat_head
    state_dict['Fq0'],state_dict['Fq1'],state_dict['Fq2'],state_dict['Fq3'] = quat_forearm
    state_dict['Fpx'],state_dict['Fpy'],state_dict['Fpz'] = pos_arm + quat_transformation(quat_arm,[-0.15,0,0]) + quat_transformation(quat_forearm,[-0.15,0,0])

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
    names=['Hq0','Hq1','Hq2','Hq3','Hpx','Hpy','Hpz','vHpx','vHpy','vHpz','aHpx','aHpy','aHpz','gHpx','gHpy','gHpz']
    state_names_body=['Bq0','Bq1','Bq2','Bq3','Bpx','Bpy','Bpz','gBpx','gBpy','gBpz']
    state_names_arm=['Aq0','Aq1','Aq2','Aq3','Apx','Apy','Apz','gApx','gApy','gApz']
    state_names_forearm=['Fq0','Fq1','Fq2','Fq3','Fpx','Fpy','Fpz','gFpx','gFpy','gFpz']
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
    mean=np.mean(data)
    return mean
file="measures"

results,ntoid=kalman_filtering(file)
# write the results in a file, in /measures/ folder
f= open(file+"/result.csv","w+")
for result in results:
    for elem in result:
        f.write(str(elem)+",")
    f.write("\n")
f.close()

from Visualisation import LiveAnimeation
LiveAnimeation(file+"/result.csv",ntoid,DEBUG=True,Body=True,Head=True,Arm=True,Forearm=True)

