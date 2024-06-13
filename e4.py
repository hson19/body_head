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
# Creation of a Kalman filter for position and velocity
class KalmanFilterPos:
    def __init__(self,deltatime,x0):
        # self.deltatime = deltatime
        self.KalmanFilter = KalmanFilter(dim_x=6, dim_z=3)
        self.KalmanFilter.F = np.array([[1,deltatime,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,deltatime,0,0],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,deltatime],
                            [0,0,0,0,0,1]])
        self.KalmanFilter.B = np.array([[0.5*deltatime**2,0,0],
                          [deltatime,0,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0]])
        self.KalmanFilter.H = np.array([[1,0,0,0,0,0],
                          [0,0,1,0,0,0],
                          [0,0,0,0,1,0]])
        # self.KalmanFilter.H = np.eye(6)
        self.KalmanFilter.Q = np.eye(6)*0.1
        self.KalmanFilter.R = np.eye(3)*10e-1 # To change ?? 
        self.KalmanFilter.P = np.eye(6)*0.1
        self.KalmanFilter.x = x0
        self.result = []
        self.result.append(x0)
    def kalman_update(self,z=None):
        if z is not None:
            self.KalmanFilter.update(z)
        self.result.append(self.KalmanFilter.x)
        return None

    def kalman_predict(self,x=None,u=None):
        if x is not None:
            self.KalmanFilter.x = x
        if u is not None:
            self.KalmanFilter.predict(u)
        else:
            self.KalmanFilter.predict()
    
    def update_pred_matrix(self,deltatime):
        self.KalmanFilter.F=np.array([[1,deltatime,0,0,0,0],
                            [0,1,0,0,0,0],
                            [0,0,1,deltatime,0,0],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,deltatime],
                            [0,0,0,0,0,1]])
        self.KalmanFilter.B = np.array([[0.5*deltatime**2,0,0],
                          [deltatime,0,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0],
                          [0,0.5*deltatime**2,0],
                          [0,deltatime,0]])
        return None
class KalmanFilterQuat:
    def __init__(self,Omega,Dt,x0):
        self.KalmanFilter = KalmanFilter(dim_x=4, dim_z=4)
        Wx, Wy, Wz = Omega
        Omega = np.array([
            [0, -Wx, -Wy, -Wz],
            [Wx, 0, -Wz, Wy],
            [Wy, Wz, 0, -Wx],
            [Wz, -Wy, Wx, 0]
        ])
        self.F=np.eye(4) + 0.5 * Dt * Omega
        self.KalmanFilter.F = np.eye(4)
        self.KalmanFilter.H = np.eye(4)
        self.KalmanFilter.Q = np.eye(4)*0.1
        self.KalmanFilter.R = np.eye(4)*0.1
        self.KalmanFilter.P = np.eye(4)*0.1
        self.KalmanFilter.x = x0
        self.result = []
        self.result.append(x0)   

    def kalman_update(self,z=None):
        if z is not None:
            self.KalmanFilter.update(z)
            self.KalmanFilter.x = unit(self.KalmanFilter.x)
        self.result.append(self.KalmanFilter.x)
        return None 
    def kalman_predict(self,x=None,u=None):
        if x is not None:
            self.KalmanFilter.x = x
        if u is not None:
            self.KalmanFilter.predict(u)
        else:
            self.KalmanFilter.predict()
        self.KalmanFilter.x = unit(self.KalmanFilter.x)
    def update_pred_matrix(self,Dt,Gyro):
        Wx, Wy, Wz = Gyro
        Omega = np.array([
            [0, -Wx, -Wy, -Wz],
            [Wx, 0, -Wz, Wy],
            [Wy, Wz, 0, -Wx],
            [Wz, -Wy, Wx, 0]
        ])        
        self.F=np.eye(4) + 0.5 * Dt * Omega

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
        """
        Predict next state (prior) using the Kalman filter state propagation
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

def getQuatPredMatrix(Dt,Gyro):
    Wx, Wy, Wz = Gyro
    Omega = np.array([
        [0, -Wx, -Wy, -Wz],
        [Wx, 0, -Wz, Wy],
        [Wy, Wz, 0, -Wx],
        [Wz, -Wy, Wx, 0]
    ])
    return np.eye(4) + 0.5 * Dt * Omega
def getPositionPredMatrix(Dt):
    return np.array([[1,0,0,Dt,0,0,(Dt**2)/2,0,0],
                    [0,1,0,0,Dt,0,0,(Dt**2)/2,0],
                    [0,0,1,0,0,Dt,0,0,(Dt**2)/2],
                    [0,0,0,1,0,0,Dt,0,0],
                    [0,0,0,0,1,0,0,Dt,0],
                    [0,0,0,0,0,1,0,0,Dt],
                    [0,0,0,0,0,0,1,0,0],
                    [0,0,0,0,0,0,0,1,0],
                    [0,0,0,0,0,0,0,0,1]])
def getBodyPositionMatrix(Dt,state,NtoId):
    n=len(state.keys())
    # Xbody
    # x= 1- 0.6*Hq0 - 0.6*Hq1 (Derivative) 
    Bpx=np.zeros(n)
    Bpx[NtoId['Hq0']]+=-0.6*state['Hq0']
    Bpx[NtoId['Hq1']]+=-0.6*state['Hq1']
    Bpx[NtoId['Hpx']]+=1

    # Ybody
    # dy/dX = 1 -0.3 Hq3 + 0.3 Hq2
    Bpy=np.zeros(n)
    Bpy[NtoId['Hpy']]+=1
    Bpy[NtoId['Hq0']]=-0.3*state['Hq3']
    Bpy[NtoId['Hq1']]=-0.3*state['Hq2']
    Bpy[NtoId['Hq2']]=-0.3*state['Hq1']
    Bpy[NtoId['Hq3']]=-0.3*state['Hq0']

    # Zbody
    #  dz/dX = 1 + 0.3 Hq2 -0.3 Hq3 + 0.3 Hq0 - 0.3 Hq1
    Bpz=np.zeros(n)
    Bpz[NtoId['Hpz']]+=1
    Bpz[NtoId['Hq0']]=0.3*state['Hq2']
    Bpz[NtoId['Hq1']]=-0.3*state['Hq3']
    Bpz[NtoId['Hq2']]=0.3*state['Hq0']
    Bpz[NtoId['Hq3']]=-0.3*state['Hq1']

    # X arm
    # x= 1 +0.3 Hq3 -0.3 Hq2 - 0.3 Hq1 + 0.3 Hq0
    Apx=np.zeros(n)
    Apx[NtoId['Hpx']]+=1
    Apx[NtoId['Hq0']]=0.3*state['Hq3']
    Apx[NtoId['Hq1']]=-0.3*state['Hq2']
    Apx[NtoId['Hq2']]=-0.3*state['Hq1']
    Apx[NtoId['Hq3']]=0.3*state['Hq0']
    
    # x = -0.6 Aq0 - 0.6 Aq2
    Apx[NtoId['Aq0']]=-0.6*state['Aq0']
    Apx[NtoId['Aq1']]=-0.6*state['Aq1']

    # Y arm
    # y= 1 -0.6 Hq0 - 0.6 Hq2
    Apy=np.zeros(n)
    Apy[NtoId['Hpy']]+=1
    Apy[NtoId['Hq0']]=-0.6*state['Hq0']
    Apy[NtoId['Hq2']]=-0.6*state['Hq2']

    # y= -0.3 Aq3 - 0.3 Aq2 -0.3 Aq1 - 0.3 Aq0
    Apy[NtoId['Aq0']]=-0.3*state['Aq3']
    Apy[NtoId['Aq1']]=-0.3*state['Aq2']
    Apy[NtoId['Aq2']]=-0.3*state['Aq1']
    Apy[NtoId['Aq3']]=-0.3*state['Aq0']

    # Z arm
    # z= 1 -0.6 Hq1 - 0.6 q0Heead - 0.6 Hq3 - 0.6 Hq2
    Apz=np.zeros(n)
    Apz[NtoId['Hpz']]+=1
    Apz[NtoId['Hq0']]=-0.3*state['Hq1']
    Apz[NtoId['Hq1']]=-0.3*state['Hq0']
    Apz[NtoId['Hq2']]=-0.3*state['Hq3']
    Apz[NtoId['Hq3']]=-0.3*state['Hq2']

    # z= 0.3 Aq2 - 0.3 Aq3 + 0.3 Aq0 - 0.3 Aq1
    Apz[NtoId['Aq0']]=0.3*state['Aq2']
    Apz[NtoId['Aq1']]=-0.3*state['Aq3']
    Apz[NtoId['Aq2']]=0.3*state['Aq0']
    Apz[NtoId['Aq3']]=-0.3*state['Aq1']

    Fpx=Apx.copy()
    Fpx[NtoId['Aq0']]-=0.6*state['Aq0']
    Fpx[NtoId['Aq1']]-=0.6*state['Aq1']

    Fpx[NtoId['Fq0']]-=0.6*state['Fq0']
    Fpx[NtoId['Fq1']]-=0.6*state['Fq1']

    Fpy=Apy.copy()
    Fpy[NtoId['Aq0']]-=0.3*state['Aq3']
    Fpy[NtoId['Aq1']]-=0.3*state['Aq2']
    Fpy[NtoId['Aq2']]-=0.3*state['Aq1']
    Fpy[NtoId['Aq3']]-=0.3*state['Aq0']

    Fpy[NtoId['Fq0']]-=0.3*state['Fq3']
    Fpy[NtoId['Fq1']]-=0.3*state['Fq2']
    Fpy[NtoId['Fq2']]-=0.3*state['Fq1']
    Fpy[NtoId['Fq3']]-=0.3*state['Fq0']


    Fpz=Apz.copy()
    Fpz[NtoId['Aq0']]=0.3*state['Aq2']
    Fpz[NtoId['Aq1']]=-0.3*state['Aq3']
    Fpz[NtoId['Aq2']]=0.3*state['Aq0']
    Fpz[NtoId['Aq3']]=-0.3*state['Aq1']

    Fpz[NtoId['Fq0']]=0.3*state['Fq2']
    Fpz[NtoId['Fq1']]=-0.3*state['Fq3']
    Fpz[NtoId['Fq2']]=0.3*state['Fq0']
    Fpz[NtoId['Fq3']]=-0.3*state['Fq1']
    return Bpx,Bpy,Bpz,Apx,Apy,Apz,Fpx,Fpy,Fpz



def JacobianF(state,Dt):
    DicState,NtoID,IDtoN = StateToDicState(state)
    matrix=np.zeros((len(state),len(state)))
    Dt=0.01
    Bpx,Bpy,Bpz,Apx,Apy,Apz,Fpx,Fpy,Fpz=getBodyPositionMatrix(Dt,DicState,NtoID)
    matrix[NtoID['Hq0']:NtoID['Hq3']+1,NtoID['Hq0']:NtoID['Hq3']+1]=getQuatPredMatrix(Dt,[DicState["gHpx"],DicState["gHpy"],DicState["gHpz"]])
    matrix[NtoID['Hpx']:NtoID['aHpz']+1,NtoID['Hpx']:NtoID['aHpz']+1]=getPositionPredMatrix(Dt)
    matrix[NtoID['gHpx']:NtoID['gHpz']+1,NtoID['gHpx']:NtoID['gHpz']+1]=np.eye(3)

    matrix[NtoID['Bq0']:NtoID['Bq3']+1,NtoID['Bq0']:NtoID['Bq3']+1]=getQuatPredMatrix(Dt,[DicState["gBpx"],DicState["gBpy"],DicState["gBpz"]])
    matrix[NtoID['Bpx']]=Bpx
    matrix[NtoID['Bpy']]=Bpy
    matrix[NtoID['Bpz']]=Bpz
    matrix[NtoID['gBpx']:NtoID['gBpz']+1,NtoID['gBpx']:NtoID['gBpz']+1]=np.eye(3)

    matrix[NtoID['Aq0']:NtoID['Aq3']+1,NtoID['Aq0']:NtoID['Aq3']+1]=getQuatPredMatrix(Dt,[DicState["gApx"],DicState["gApy"],DicState["gApz"]])
    matrix[NtoID['Apx']]=Apx
    matrix[NtoID['Apy']]=Apy
    matrix[NtoID['Apz']]=Apz
    matrix[NtoID['gApx']:NtoID['gApz']+1,NtoID['gApx']:NtoID['gApz']+1]=np.eye(3)

    matrix[NtoID['Fq0']:NtoID['Fq3']+1,NtoID['Fq0']:NtoID['Fq3']+1]=getQuatPredMatrix(Dt,[DicState["gApx"],DicState["gApy"],DicState["gApz"]])
    matrix[NtoID['Fpx']]=Fpx
    matrix[NtoID['Fpy']]=Fpy
    matrix[NtoID['Fpz']]=Fpz
    matrix[NtoID['gFpx']:NtoID['gFpz']+1,NtoID['gFpx']:NtoID['gFpz']+1]=np.eye(3)
    Printmatrix(matrix,IDtoN)
    cmap = ListedColormap(['white', 'black'])

    # Create a binary mask of the data
    # binary_data = (matrix != 0).astype(int)    
    # plt.imshow(binary_data,cmap=cmap)
    # plt.show()
    return matrix
def PredictionFunction(file):
    DataHead,DataBody,DataArm,DataForearm=DataIterable(file,"head"),DataIterable(file,"body"),DataIterable(file,"arm"),DataIterable(file,"forearm")
    DicState,NtoID,IDToN=InitState(DataHead.data,DataBody.data,DataArm.data,DataForearm.data)
    sensor=min([DataHead,DataBody,DataArm,DataForearm],key=lambda x: x.data['time'][x.i] if x.data is not None else np.inf)
    LastTime=sensor.data['time'][sensor.i]
    CurrentTime=LastTime
    sensor.i+=1
    NextSensor=min(DataHead,DataBody,DataArm,DataForearm,key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
    Dt=(NextSensor.data['time'][NextSensor.i]-LastTime)/1000

    state=[DicState[IDToN[i]] for i in range(len(IDToN))]
    filter=ModelKalmanFilter(dim_x=len(state),dim_z=9)
    filter.x=state
    results=[]

    # JF=JacobianF(state,Dt)
    JF = np.eye(len(state))*0.001
    lenHead=len(DataHead.data) if DataHead.data is not None else 0
    lenBody=len(DataBody.data) if DataBody.data is not None else 0
    lenArm=len(DataArm.data) if DataArm.data is not None else 0
    lenForearm=len(DataForearm.data) if DataForearm.data is not None else 0
    while DataHead.i<lenHead or DataBody.i<lenBody or DataArm.i<lenArm or DataForearm.i<lenForearm:
        sensor=min(DataHead,DataBody,DataArm,DataForearm,key=lambda x: x.data['time'][x.i] if x.data is not None and x.i < len(x.data) else np.inf)
        CurrentTime=sensor.data['time'][sensor.i]
        Dt= (CurrentTime-LastTime)/1000

        # JF=JacobianF(state,Dt)
        JF = np.eye(len(state))*0.1

        filter.F=JF
        DicState,NtoID,IDToN=StateToDicState(filter.x)
        filter.x=StatePrediction(DicState,Dt,NtoID)
        filter.predict()

        if sensor.card=="head":
            # print("Head")
            Qn= [1,0,0,0]
            accL = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(observationModel(Qn,accL,mag)) # quaternion from local to global
            z=np.append(z,[accL[0],accL[1],accL[2]])
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            H = np.zeros((10,len(state)))
            H[0:4,NtoID['Hq0']:NtoID['Hq3']+1]=np.eye(4)
            H[4:7,NtoID['aHpx']:NtoID['aHpz']+1]=np.eye(3)
            H[7:,NtoID['gHpx']:NtoID['gHpz']+1]=np.eye(3)
            filter.update(z,H,NtoID,R=np.eye(10)*0.001)
        elif sensor.card=="body":
            Qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(observationModel(Qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            H = np.zeros((7,len(state)))
            H[0:4,NtoID['Bq0']:NtoID['Bq3']+1]=np.eye(4)
            H[4:,NtoID['gBpx']:NtoID['gBpz']+1]=np.eye(3)
            filter.update(z,H,NtoID,R=np.eye(7)*0.001)


        elif sensor.card=="arm":
            Qn=[1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(observationModel(Qn,acc,mag)) # Local to Global
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            H = np.zeros((7,len(state)))
            H[0:4,NtoID['Aq0']:NtoID['Aq3']+1]=np.eye(4)
            H[4:,NtoID['gApx']:NtoID['gApz']+1]=np.eye(3)
            print("Arm quaternion before update",filter.x[NtoID['Aq0']:NtoID['Aq3']+1],"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            filter.update(z,H,NtoID,R=np.eye(7)*0.001)
            print("Arm quaternion after update",filter.x[NtoID['Aq0']:NtoID['Aq3']+1])
            print(" Measurement",z)

        elif sensor.card=="forearm":
            Qn= [1,0,0,0]
            acc = [sensor.data['ax'][sensor.i],sensor.data['ay'][sensor.i],sensor.data['az'][sensor.i]]
            mag = [sensor.data['mx'][sensor.i],sensor.data['my'][sensor.i],sensor.data['mz'][sensor.i]]
            z=quat_conjugate(observationModel(Qn,acc,mag))
            z=np.append(z,[sensor.data['gx'][sensor.i],sensor.data['gy'][sensor.i],sensor.data['gz'][sensor.i]])
            H = np.zeros((7,len(state)))
            H[0:4,NtoID['Fq0']:NtoID['Fq3']+1]=np.eye(4)
            H[4:,NtoID['gFpx']:NtoID['gFpz']+1]=np.eye(3)
            filter.update(z,H,NtoID,R=np.eye(7)*0.001)
        sensor.i+=1
        results.append(filter.x)
        LastTime=CurrentTime
    return results,NtoID
def StatePrediction(DicState,Dt,NtoID):
    StateHead = [DicState['Hpx'],DicState["vHpx"],DicState['Hpy'],DicState['vHpy'],DicState['Hpz'],DicState['vHpz']]
    quatHead= [DicState['Hq0'],DicState['Hq1'],DicState['Hq2'],DicState['Hq3']]
    ConjQuatHead=quat_conjugate(quatHead)
    KalmanFilterQuatHead=KalmanFilterQuat([DicState['gHpx'],DicState['gHpy'],DicState['gHpz']],Dt,ConjQuatHead)
    KalmanFilterPosHead=KalmanFilterPos(Dt,StateHead)
    
    accHeadL=[DicState['aHpx'],DicState['aHpy'],DicState['aHpz']]
    accHeadG= quat_transformation(quatHead,accHeadL)
    accHeadG[1]-=9.81 # remove gravity
    KalmanFilterQuatHead.update_pred_matrix(Dt,[DicState['gHpx'],DicState['gHpy'],DicState['gHpz']])
    KalmanFilterPosHead.kalman_predict(u=accHeadG)
    KalmanFilterQuatHead.kalman_predict()
    KalmanFilterPosHead.kalman_update()
    KalmanFilterQuatHead.kalman_update()

    StateHead = KalmanFilterPosHead.result[-1]
    NewPosHead= StateHead.reshape(3,2)[:,0]
    NewVHead = StateHead.reshape(3,2)[:,1]
    NewquatHead = quat_conjugate(KalmanFilterQuatHead.result[-1])
    
    conjugateQuatBody= quat_conjugate([DicState['Bq0'],DicState['Bq1'],DicState['Bq2'],DicState['Bq3']])
    KalmanFilterQuatBody=KalmanFilterQuat([DicState['gBpx'],DicState['gBpy'],DicState['gBpz']],Dt,conjugateQuatBody)
    NewPosBody = NewPosHead + quat_transformation(quatHead,[-0.3,0,0])
    KalmanFilterQuatBody.update_pred_matrix(Dt,[DicState['gBpx'],DicState['gBpy'],DicState['gBpz']])
    KalmanFilterQuatBody.kalman_predict()
    KalmanFilterQuatBody.kalman_update()
    NewquatBody = quat_conjugate(KalmanFilterQuatBody.result[-1])

    quatArm = [DicState['Aq0'],DicState['Aq1'],DicState['Aq2'],DicState['Aq3']]
    PosArm = NewPosHead + quat_transformation(quatHead,[0,-0.25,0]) + quat_transformation(quatArm,[-0.15,0,0])


    conjugateQuatArm = quat_conjugate(quatArm)
    print("Arm quaternion before predict",quatArm)
    KalmanFilterQuatArm=KalmanFilterQuat([DicState['gApx'],DicState['gApy'],DicState['gApz']],Dt,conjugateQuatArm)
    KalmanFilterQuatArm.update_pred_matrix(Dt,[DicState['gApx'],DicState['gApy'],DicState['gApz']])
    KalmanFilterQuatArm.kalman_predict()
    KalmanFilterQuatArm.kalman_update()

    NewquatArm = quat_conjugate(KalmanFilterQuatArm.result[-1])
    print("Arm quaternion after predict",NewquatArm)
    quatForearm = [DicState['Fq0'],DicState['Fq1'],DicState['Fq2'],DicState['Fq3']]
    PosForearm = PosArm + quat_transformation(quatArm,[-0.15,0,0]) + quat_transformation(quatForearm,[-0.15,0,0])

    conjugateQuatForearm=quat_conjugate(quatForearm) # Global to Local
    KalmanFilterQuatForearm=KalmanFilterQuat([DicState['gFpx'],DicState['gFpy'],DicState['gFpz']],Dt,conjugateQuatForearm)
    KalmanFilterQuatForearm.update_pred_matrix(Dt,[DicState['gFpx'],DicState['gFpy'],DicState['gFpz']])
    KalmanFilterQuatForearm.kalman_predict()
    KalmanFilterQuatForearm.kalman_update()
    NewquatForearm = quat_conjugate(KalmanFilterQuatForearm.result[-1])


    State={'Hq0':NewquatHead[0],'Hq1':NewquatHead[1],'Hq2':NewquatHead[2],'Hq3':NewquatHead[3],'Hpx':NewPosHead[0],'Hpy':NewPosHead[1],'Hpz':NewPosHead[2],'vHpx':NewVHead[0],'vHpy':NewVHead[1],
           'vHpz':NewVHead[2],'aHpx':DicState['aHpx'],'aHpy':DicState['aHpy'],'aHpz':DicState['aHpz'],'gHpx':DicState['gHpx'],'gHpy':DicState['gHpy'],'gHpz':DicState['gHpz'],
           'Bq0':NewquatBody[0],'Bq1':NewquatBody[1],'Bq2':NewquatBody[2],'Bq3':NewquatBody[3],'Bpx':NewPosBody[0],'Bpy':NewPosBody[1],'Bpz':NewPosBody[2],'gBpx':DicState['gBpx'],'gBpy':DicState['gBpy'],
           'gBpz':DicState['gBpz'],'Aq0':NewquatArm[0],'Aq1':NewquatArm[1],'Aq2':NewquatArm[2],'Aq3':NewquatArm[3],'Apx':PosArm[0],'Apy':PosArm[1],'Apz':PosArm[2],'gApx':DicState['gApx'],'gApy':DicState['gApy'],
            'gApz':DicState['gApz'],'Fq0':NewquatForearm[0],'Fq1':NewquatForearm[1],'Fq2':NewquatForearm[2],'Fq3':NewquatForearm[3],'Fpx':PosForearm[0],'Fpy':PosForearm[1],'Fpz':PosForearm[2],
            'gFpx':DicState['gFpx'],'gFpy':DicState['gFpy'],'gFpz':DicState['gFpz']}
    NewState=np.zeros(len(DicState.keys()))
    for key in DicState.keys():
        NewState[NtoID[key]]=State[key]
    return NewState

def InitState(HeadData,BodyData,ArmData,ForearmData):
    DicState,NToID,IDToName=StateToDicState(np.zeros(46))
    quatHead = quat_conjugate(observationModel([1, 0, 0, 0],[HeadData['ax'][0],HeadData['ay'][0],HeadData['az'][0]], [HeadData['mx'][0],HeadData['my'][0],HeadData['mz'][0]],Mua=1))
    DicState['Hq0'],DicState['Hq1'],DicState['Hq2'],DicState['Hq3'] = quatHead
    if BodyData is not None:
        quatBody = quat_conjugate(observationModel([1, 0, 0, 0],[BodyData['ax'][0],BodyData['ay'][0],BodyData['az'][0]], [BodyData['mx'][0],BodyData['my'][0],BodyData['mz'][0]],Mua=1))
    else:
        quatBody = quatHead
    DicState['Bq0'],DicState['Bq1'],DicState['Bq2'],DicState['Bq3'] = quatBody
    DicState['Bpx'],DicState['Bpy'],DicState['Bpz'] = [0,0,0] + quat_transformation(quatHead,[-0.3,0,0])
    if ArmData is not None:
        quatArm = quat_conjugate(observationModel([1, 0, 0, 0],[ArmData['ax'][0],ArmData['ay'][0],ArmData['az'][0]], [ArmData['mx'][0],ArmData['my'][0],ArmData['mz'][0]],Mua=1))
    else:
        quatArm = quatHead
    DicState['Aq0'],DicState['Aq1'],DicState['Aq2'],DicState['Aq3'] = quatArm
    ArmPos = [0,0,0] + quat_transformation(quatHead,[0,-0.15,0]) + quat_transformation(quatArm,[-0.15,0,0])
    DicState['Apx'],DicState['Apy'],DicState['Apz'] = ArmPos
    if ForearmData is not None:
        quatForearm = quat_conjugate(observationModel([1, 0, 0, 0],[ForearmData['ax'][0],ForearmData['ay'][0],ForearmData['az'][0]], [ForearmData['mx'][0],ForearmData['my'][0],ForearmData['mz'][0]],Mua=1))
    else:
        quatForearm = quatHead
    DicState['Fq0'],DicState['Fq1'],DicState['Fq2'],DicState['Fq3'] = quatForearm
    DicState['Fpx'],DicState['Fpy'],DicState['Fpz'] = ArmPos + quat_transformation(quatArm,[-0.15,0,0]) + quat_transformation(quatForearm,[-0.15,0,0])

    return DicState,NToID,IDToName
######################################################################################################
# Utilitlies function
######################################################################################################
#Takes L TO G quaternion Local Acceleration and local Magnetic field
#Returns Global to Local quaternion
def observationModel(Qn, Acc, Mag,Mua=1):
    # TODO: Find appropriate value for MUA
    Q1, Q2, Q3, Q4 = Qn
    QGtoL = np.array([Q1, -Q2, -Q3, -Q4])
    VgHat = unit(quat_transformation(QGtoL, [0, -1,0 ]))

    Vg = unit(-np.array(Acc))
    Na = unit(np.cross(VgHat, Vg))
    Domega = np.arccos(np.dot(VgHat, Vg))
    
    Qae2, Qae3, Qae4 = np.sin(Mua * Domega / 2)* Na
    Qae1 = np.cos(Mua * Domega / 2)
    Qa = unit(quat_product([Qae1, Qae2, Qae3, Qae4], QGtoL))
    Qa1, Qa2, Qa3, Qa4 = Qa
    Qainv = np.array([Qa1, -Qa2, -Qa3, -Qa4])

    VnorthL = quat_transformation(Qa, [1,0, 0])
    Vmxyz = quat_transformation(Qainv, Mag)
    Bx, _, Bz = Vmxyz
    Vmxz = [Bx / np.sqrt(Bx**2 + Bz**2), 0, Bz / np.sqrt(Bx**2 + Bz**2)]

    VmxzL = quat_transformation(Qa, Vmxz)
    Domegam = np.arccos(np.dot(VnorthL, VmxzL))
    Qme2, Qme3, Qme4 = np.sin(Domegam / 2)*np.cross(VnorthL, VmxzL)
    Qme1 = np.cos(Domegam / 2)
    Qme = [Qme1, Qme2, Qme3, Qme4]
    Qm = unit(quat_product( Qme,Qa))
    return unit(Qm)

def Printmatrix(matrix,IDToN):
    f= open("matrix.txt","w+")
    for i,row in enumerate(matrix):
        f.write(IDToN[i]+" ")
        for j, elem in enumerate(row):
            
            if elem!=0:
                f.write("\Fraction{d "+IDToN[i]+"}{d "+IDToN[j]+"}= "+str(elem)+" ")
        f.write("\n")
    f.close()
def StateToDicState(state):
    names=['Hq0','Hq1','Hq2','Hq3','Hpx','Hpy','Hpz','vHpx','vHpy','vHpz','aHpx','aHpy','aHpz','gHpx','gHpy','gHpz']
    BodyNames=['Bq0','Bq1','Bq2','Bq3','Bpx','Bpy','Bpz','gBpx','gBpy','gBpz']
    ArmNames=['Aq0','Aq1','Aq2','Aq3','Apx','Apy','Apz','gApx','gApy','gApz']
    ForearmNames=['Fq0','Fq1','Fq2','Fq3','Fpx','Fpy','Fpz','gFpx','gFpy','gFpz']
    names.extend(BodyNames)
    names.extend(ArmNames)
    names.extend(ForearmNames)
    DicState={names[i]:state[i] for i in range(len(names))}
    NToID={names[i]:i for i in range(len(names))}
    IDToName={i:names[i] for i in range(len(names))}
    return DicState,NToID,IDToName
def ReadData(file,card):
    # Read the data from the file
    col_names=['it','time','ax','ay','az','gx','gy','gz','mx','my','mz']
    try:
        data = pd.read_csv(file+"/nav3_body_head@"+card+".csv",names=col_names,header=None)
    except:
        return None
    return data
def quat_transformation(q, p):
    # q is the quaternion, p is the point to transform
    quat_p=np.array([0,p[0],p[1],p[2]])
    q_inv = quat_conjugate(q)
    q_p = quat_product(q,quat_product(quat_p,q_inv))
    return q_p[1:]
def quat_conjugate(quat):
    q1, q2, q3, q4 = quat
    return np.array([q1, -q2, -q3, -q4])
def unit(x):
    return x/(np.linalg.norm(x))
def quat_product(q1, q2):
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

results,NToID=PredictionFunction(file)
# write the results in a file, in /measures/ folder
f= open(file+"/result.csv","w+")
for result in results:
    for elem in result:
        f.write(str(elem)+",")
    f.write("\n")
f.close()

from Visualisation import LiveAnimeation
LiveAnimeation(file+"/result.csv",NToID,DEBUG=True,Body=True,Head=True,Arm=True,Forearm=True)

