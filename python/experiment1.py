# This code is for the first experiment.
#  A subject with 2 card walks 2 meters in a straight line and than stops.
# We use the code to detect how much a second card improve the measurement.

import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from scipy.spatial.transform import Rotation as R
import pandas as pd

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

    def kalman_update(self,z):
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

#Use the position and veloticy of body and head
def bodyHeadConstraint(KalmanFilterObjectHead,KalmanFilterObjectBody,HeadOr,BodyOr):
    # Transformation of positions
    ConstraintHeadPos = quat_transformation(quat_conjugate(BodyOr), [0.3, 0, 0])
    ConstraintHeadPos += KalmanFilterObjectBody.KalmanFilter.x.reshape(3,2)[:,0]
    ConstraintBodyPos = quat_transformation(quat_conjugate(HeadOr), [-0.3, 0, 0])
    ConstraintBodyPos += KalmanFilterObjectHead.KalmanFilter.x.reshape(3,2)[:,0]
    
    VelocityHead = KalmanFilterObjectHead.KalmanFilter.x.reshape(3,2)[:,1]
    VelocityBody = KalmanFilterObjectBody.KalmanFilter.x.reshape(3,2)[:,1]
    measurementHead=np.array([ConstraintHeadPos[0],VelocityBody[0],ConstraintHeadPos[1],VelocityBody[1],ConstraintHeadPos[2],VelocityBody[2]])
    measurementBody=np.array([ConstraintBodyPos[0],VelocityHead[0],ConstraintBodyPos[1],VelocityHead[1],ConstraintBodyPos[2],VelocityHead[2]])
    # Kalman filter update for Head
    KalmanFilterObjectHead.kalman_update(measurementHead)
    # Kalman filter update for Body
    KalmanFilterObjectBody.kalman_update(measurementBody)
    
    return 
#Use only the position of the head and body
def bodyHeadConstraint2(KalmanFilterObjectHead,KalmanFilterObjectBody,HeadOr,BodyOr,Predicted):
    VelocityHead = KalmanFilterObjectHead.KalmanFilter.x.reshape(3,2)[:,1]
    VelocityBody = KalmanFilterObjectBody.KalmanFilter.x.reshape(3,2)[:,1]
    ConstraintHeadPos = quat_transformation(quat_conjugate(BodyOr), [0.3, 0, 0])
    ConstraintHeadPos += KalmanFilterObjectBody.KalmanFilter.x.reshape(3,2)[:,0]
    ConstraintBodyPos= quat_transformation(quat_conjugate(HeadOr), [-0.3, 0, 0])
    ConstraintBodyPos += KalmanFilterObjectHead.KalmanFilter.x.reshape(3,2)[:,0]
                   
    if Predicted=="head":
        measurementBody=np.array([ConstraintBodyPos[0],ConstraintBodyPos[1],ConstraintBodyPos[2]])
        KalmanFilterObjectBody.kalman_update(measurementBody)
        print("body position just updated")
        KalmanFilterObjectHead.kalman_update() #Just record the position if no measurement
    elif Predicted=="body":
        measurementHead=np.array([ConstraintHeadPos[0],ConstraintHeadPos[1],ConstraintHeadPos[2]])
        KalmanFilterObjectHead.kalman_update(measurementHead)
        KalmanFilterObjectBody.kalman_update() #Just record the position if no measurement
    else:
        print("Error, Predicted should be head or body")
     
    return
# Reading the data
def ReadData(card):
    # Read the data from the file
    col_names=['it','time','ax','ay','az','gx','gy','gz','mx','my','mz']
    data = pd.read_csv("measures2m/nav3_body_head@"+card+".csv",names=col_names,header=None)
    return data

def kalmanQuaternion(Gyro, Dt,Quat,KalmanFilterObject):
    Wx, Wy, Wz = Gyro
    Omega = np.array([
        [0, -Wx, -Wy, -Wz],
        [Wx, 0, -Wz, Wy],
        [Wy, Wz, 0, -Wx],
        [Wz, -Wy, Wx, 0]
    ])
    KalmanFilterObject.F=np.eye(4) + 0.5 * Dt * Omega
    KalmanFilterObject.predict()
    Xorp = KalmanFilterObject.x
    ConjQuat = quat_conjugate(Quat)
    if np.dot(ConjQuat, Xorp) > 0:
        Qc = Quat
        KalmanFilterObject.update(Qc)
    else:
        Qc = quat_conjugate(-ConjQuat)
        KalmanFilterObject.update(Qc)

    return None

# Returns Global to Local quaternion
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
def runExperimentConstraint():
    HeadData,BodyData = ReadData("head"),ReadData("body")
    
    itHeadmax,itBodymax=HeadData['it'].max(),BodyData['it'].max()
    mus=[1]
    PosDic={}
    VelocityDic={}
    for mu in mus:
        itHead,itBody = 0,0
        KalmanFilterPosHead = KalmanFilterPos(0.01, np.array([0, 0, 0, 0, 0, 0]))
        KalmanFilterPosBody = KalmanFilterPos(0.01, np.array([-0.3, 0, 0,0, 0, 0]))

        quatHead = observationModel([1, 0, 0, 0],[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]], [HeadData['mx'][itHead],HeadData['my'][itHead],HeadData['mz'][itHead]],Mua=1)
        quatBody = observationModel([1, 0, 0, 0],[BodyData['ax'][itBody],BodyData['ay'][itBody],BodyData['az'][itBody]], [BodyData['mx'][itBody],BodyData['my'][itBody],BodyData['mz'][itBody]],Mua=1)

        KalmanFilterQuatHead = KalmanFilterQuat([HeadData['gx'][itHead],HeadData['gy'][itHead],HeadData['gz'][itHead]],0.01, quatHead)
        KalmanFilterQuatBody = KalmanFilterQuat([BodyData['gx'][itBody],BodyData['gy'][itBody],BodyData['gz'][itBody]],0.01,quatBody)

        itHead+=1;itBody+=1
        lastTime= max(HeadData['time'][0],BodyData['time'][0])
        StartTime=lastTime
        Times=[0]
        while(itHead< itHeadmax and itBody < itBodymax):
            # checks which data is the most recent updating the most recent data
            if HeadData['time'][itHead] < BodyData['time'][itBody]:
                KalmanPos=KalmanFilterPosHead
                KalmanQuat=KalmanFilterQuatHead
                acc=[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]]
                Dt=(HeadData['time'][itHead]-lastTime)/1000
                Omega = [HeadData['gx'][itHead], HeadData['gy'][itHead], HeadData['gz'][itHead]]
                CurrentTime=HeadData['time'][itHead]
                Predicted="head"
            else:
                KalmanPos=KalmanFilterPosBody
                KalmanQuat=KalmanFilterQuatBody
                acc=[BodyData['ax'][itBody],BodyData['ay'][itBody],BodyData['az'][itBody]]
                Dt=(BodyData['time'][itBody]-lastTime)/1000
                Omega = [BodyData['gx'][itBody], BodyData['gy'][itBody], BodyData['gz'][itBody]]
                CurrentTime=BodyData['time'][itBody]
                Predicted="body"

            # Kalman filter for position
            KalmanPos.update_pred_matrix(Dt)
            TransformedAcc=quat_transformation(quat_conjugate(KalmanQuat.KalmanFilter.x),acc)
            TransformedAcc += np.array([0,-9.81,0]) # Adding gravity in the Up direction
            KalmanPos.kalman_predict(u=TransformedAcc)
            # bodyHeadConstraint(KalmanFilterPosHead,KalmanFilterPosBody,KalmanFilterQuatHead.KalmanFilter.x,KalmanFilterQuatBody.KalmanFilter.x)
            bodyHeadConstraint2(KalmanFilterPosHead,KalmanFilterPosBody,KalmanFilterQuatHead.KalmanFilter.x,KalmanFilterQuatBody.KalmanFilter.x,Predicted)
            
            # Kalman filter for quaternion
            KalmanQuat.update_pred_matrix(Dt,Omega)
        
            KalmanQuat.kalman_predict()
            ObservationQuat=observationModel(KalmanQuat.KalmanFilter.x,[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]], [HeadData['mx'][itHead],HeadData['my'][itHead],HeadData['mz'][itHead]],Mua=1)
            KalmanQuat.kalman_update(ObservationQuat)

            if HeadData['time'][itHead] < BodyData['time'][itBody]:
                itHead+=1
            else:
                itBody+=1
            Times.append(abs(CurrentTime-StartTime)/1000)
            lastTime=CurrentTime
    
        # PosHead and Body are (n,6) arrays
        PosHead = np.array(KalmanFilterPosHead.result)
        PosHead=PosHead.reshape(PosHead.shape[0],6)

        PosBody = np.array(KalmanFilterPosBody.result)
        PosBody=PosBody.reshape(PosBody.shape[0],6)
        # QuatHead and Body are (n,4) arrays
        QuatHead = np.array(KalmanFilterQuatHead.result)

        xHead, yHead, zHead = PosHead[:,0], PosHead[:,2], PosHead[:,4]
        xBody, yBody, zBody = PosBody[:,0], PosBody[:,2], PosBody[:,4]
        PosDic[mu]=[[xHead,yHead,zHead],[xBody,yBody,zBody]]
        VelocityDic[mu]=[[PosHead[:,1],PosHead[:,3],PosHead[:,5]],[PosBody[:,1],PosBody[:,3],PosBody[:,5]]]

    fig, axs = plt.subplots(2, 1)
    axs[0].axhline(y=2, color='r', linestyle='--',label='2m mark')
    axs[1].axhline(y=0, color='r', linestyle='--',label='0m mark')
    for mu in mus:
        xHead, yHead, zHead = PosDic[mu][0]
        xBody, yBody, zBody = PosDic[mu][1]
    #Take the norm of position on x and z axis
        axs[0].plot(Times, np.sqrt(xHead**2 + zHead**2), label='Upper Chest')
        axs[0].plot(Times, np.sqrt(xBody**2 + zBody**2), label='Abdomen')
        # Plot a red line at the 2m mark
        
        axs[0].set_xlabel('Time [s]')
        axs[0].set_ylabel('Distance [m]')
        #Take the position on y axis
        axs[1].plot(Times, yHead)# label='Upper Chest')
        axs[1].plot(Times, yBody)# label='Abdomen')
        # plot a red line at 0m mark
        
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('Up axis [m]')
    axs[0].legend()
    axs[1].legend()
    fig.tight_layout()    
    fig.savefig("figures/PositionConstraint.pdf")
    plt.show()    
    
    vxHead, vyHead, vzHead = VelocityDic[mu][0]
    vxBody, vyBody, vzBody = VelocityDic[mu][1]
    fig, axs = plt.subplots(2, 1)
    for mu in mus:
        # plot the norm of the velocity on the x and z axis
        axs[0].plot(Times, np.sqrt(vxHead**2 + vzHead**2), label='Upper chest')
        axs[0].plot(Times, np.sqrt(vxBody**2 + vzBody**2), label='Abdomen')
        axs[0].set_xlabel('Time [s]')
        axs[0].set_ylabel('Velocity [m/s]')
        
        # plot the velocity on the y axis
        axs[1].plot(Times, vyHead, label='Upper chest, mu='+str(mu))
        axs[1].plot(Times, vyBody, label='Abdomen, mu='+str(mu))
        axs[1].set_title('Velocity of the head on the UP axis')
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('Velocity [m/s]')
    # axs[0].legend()
    # axs[1].legend()    
    fig.tight_layout()    
    fig.savefig("figures/VelocityConstraint.pdf")
    plt.show()       
    return KalmanFilterPosHead.result,KalmanFilterPosBody.result,KalmanFilterQuatHead.result,KalmanFilterQuatBody.result,Times
def runExperimentWithoutConstraint():
    ReferenceData=ReadReferenceData()
    mus=np.linspace(0.1,1,5)
    DicResults={}
    for mu in mus:
        HeadData= ReadData("head")
        itHead = 0
        itHeadmax=HeadData['it'].shape[0]
        KalmanFilterPosHead = KalmanFilterPos(0.01, np.array([0, 0, 0, 0, 0, 0]))
        quatHead = observationModel([1, 0, 0, 0],[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]], [HeadData['mx'][itHead],HeadData['my'][itHead],HeadData['mz'][itHead]],Mua=mu)

        KalmanFilterQuatHead = KalmanFilterQuat([HeadData['gx'][itHead],HeadData['gy'][itHead],HeadData['gz'][itHead]],0.01, quatHead)
        itHead+=1
        while(itHead< itHeadmax):
            # checks which data is the most recent updating the most recent data
            acc=[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]]
            Dt=abs(HeadData['time'][itHead]-HeadData['time'][itHead-1])/1000
            Omega = [HeadData['gx'][itHead], HeadData['gy'][itHead], HeadData['gz'][itHead]]

            # Kalman filter for position
            KalmanFilterPosHead.update_pred_matrix(Dt)
            TransformedAcc=quat_transformation(quat_conjugate(KalmanFilterQuatHead.KalmanFilter.x),acc)
            TransformedAcc = np.array(TransformedAcc) - np.array([0,9.81,0]) # Adding gravity in the Up direction
            KalmanFilterPosHead.kalman_predict(u=TransformedAcc)
            KalmanFilterPosHead.kalman_update() #Just record the position if no measurement

            # Kalman filter for quaternion
            KalmanFilterQuatHead.update_pred_matrix(Dt,Omega)
            KalmanFilterQuatHead.kalman_predict()
            ObservationQuat=observationModel(KalmanFilterQuatHead.KalmanFilter.x,acc, [HeadData['mx'][itHead-1],HeadData['my'][itHead-1],HeadData['mz'][itHead-1]],Mua=1)
            KalmanFilterQuatHead.kalman_update(ObservationQuat)
            itHead+=1
        DicResults[mu]=KalmanFilterPosHead.result

        
    fig, axs = plt.subplots(2, 1) 
    axs[0].axhline(y=2, color='r', linestyle='--',label='2m mark')
    axs[1].axhline(y=0, color='r', linestyle='--',label='0m mark')
    for mu in mus:
        TimeHead = (HeadData['time']-HeadData['time'][0])/1000
        # PosHead and Body are (n,6) arrays
        PosHead = np.array(DicResults[mu])
        PosHead=PosHead.reshape(PosHead.shape[0],6)
        # QuatHead and Body are (n,4) arrays
        QuatHead = np.array(KalmanFilterQuatHead.result)

        xHead, yHead, zHead = PosHead[:,0], PosHead[:,2], PosHead[:,4]

        
        #Take the norm of position on x and z axis
        axs[0].plot(TimeHead, np.sqrt(xHead**2 + zHead**2), label='mu='+str(mu))
        # Plot a red line at the 2m mark
        
        axs[0].set_xlabel('Time [s]')
        axs[0].set_ylabel('Distance [m]')

        #Take the position on y axis
        axs[1].plot(TimeHead, yHead)# label='mu='+str(mu))
        # plot a red line at 0m mark
        
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('Up [m]')
    axs[0].legend()    
    axs[1].legend()
    fig.tight_layout()    
    fig.savefig("figures/PositionWithoutConstraint.pdf")
    plt.show()  
    fig, axs = plt.subplots(2, 1)  
    for mu in mus:
        PosHead = np.array(DicResults[mu])
        vxHead, vyHead, vzHead = PosHead[:,1], PosHead[:,3], PosHead[:,5]
        
        # plot the norm of the velocity on the x and z axis
        axs[0].plot(TimeHead, np.sqrt(vxHead**2 + vzHead**2), label='mu='+str(mu))
        axs[0].set_xlabel('Time [s]')
        axs[0].set_ylabel('Velocity [m/s]')
        
        # plot the velocity on the y axis
        axs[1].plot(TimeHead, vyHead, label='mu='+str(mu))
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('Velocity Up [m/s]')
    # axs[0].legend()
    # axs[1].legend() 
    fig.tight_layout()
    fig.savefig("figures/VelocityWithoutConstraint.pdf")
    plt.show()
    return KalmanFilterPosHead.result,None,KalmanFilterQuatHead.result,None,TimeHead,None


#Doesn't use any constraint nor the quaternino orientation
# THis imply that the x direction is the gravity direction
def runExperimentWithoutConstraintWithoutQuat():
    HeadData= ReadData("head")
    itHead= 0
    itHeadmax=HeadData['it'].max()
    KalmanFilterPosHead = KalmanFilterPos(0.01, np.array([0, 0, 0, 0, 0, 0]))
    StartTime=HeadData['time'][0]
    LastTime=StartTime
    while(itHead< itHeadmax-1):
        # checks which data is the most recent updating the most recent data
        acc=[HeadData['ax'][itHead],HeadData['ay'][itHead],HeadData['az'][itHead]]
        Dt=abs(HeadData['time'][itHead+1]-HeadData['time'][itHead])/1000
        # Kalman filter for position
        KalmanFilterPosHead.update_pred_matrix(Dt)
        TransformedAcc = np.array(acc) - np.array([9.81,0,0])
        # print("TransformedAcc",TransformedAcc)
        KalmanFilterPosHead.kalman_predict(u=TransformedAcc)
        KalmanFilterPosHead.kalman_update()
        itHead+=1

    TimeHead=(HeadData['time']-StartTime)/1000


    #Ploting the results
    # PosHead is (n,6) arrays
    PosHead=np.array(KalmanFilterPosHead.result)
    PosHead=PosHead.reshape(PosHead.shape[0],6)

    xHead, yHead, zHead = PosHead[:,0], PosHead[:,2], PosHead[:,4]

    fig, axs = plt.subplots(2, 1)
    #Take the norm of position on x and z axis
    axs[0].plot(TimeHead, np.sqrt(yHead**2 + zHead**2), label='Upper Chest')
    # Plot a red line at the 2m mark
    axs[0].axhline(y=2, color='r', linestyle='--',label='2m mark')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Distance [m]')
    axs[0].legend()
    #Take the position on y axis
    axs[1].plot(TimeHead, xHead)# label='Upper Chest')
    # plot a red line at 0m mark
    axs[1].axhline(y=0, color='r', linestyle='--',label='0m mark')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Up [m]')
    axs[1].legend()
    fig.tight_layout()
    fig.savefig("figures/PositionWithoutConstraintWithoutQuat.pdf")
    plt.show()   

    vxHead, vyHead, vzHead = PosHead[:,1], PosHead[:,3], PosHead[:,5]
    fig, axs = plt.subplots(2, 1)
    # plot the norm of the velocity on the x and z axis
    axs[0].plot(TimeHead, np.sqrt(vyHead**2 + vzHead**2), label='Upper Chest')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Velocity [m/s]')
    
    # plot the velocity on the y axis
    axs[1].plot(TimeHead, vxHead, label='Upper Chest')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Velocity Up [m/s]')

    # axs[0].legend()
    # axs[1].legend()
    fig.tight_layout()
    fig.savefig("figures/VelocityWithoutConstraintWithoutQuat.pdf")
    plt.show() 
    return KalmanFilterPosHead.result,None,None,None,TimeHead,None

# compare the results
def compareResults(PosHead,PosBody,QuatHead,QuatBody,TimeHead,TimeBody):
    #loading the reference data
    col_names=['it','time0','time1']
    Head_names=['Hq0','Hq1','Hq2','Hq3','HQp0','HQp1','HQp2','HQp3','HQo0','HQo1','HQo2','HQo3','Hpx','HVx','Hax','Hpy','Hvy','Hay','Hpz','Hvz','Haz']
    Body_names=['Bq0','Bq1','Bq2','Bq3','BQp0','BQp1','BQp2','BQp3','BQo0','BQo1','BQo2','BQo3','Bpx','BVx','Bax','Bpy','Bvy','Bay','Bpz','Bvz','Baz']
    Arm_names=['Aq0','Aq1','Aq2','Aq3','AQp0','AQp1','AQp2','AQp3','AQo0','AQo1','AQo2','AQo3','Apx','AVx','Aax','Apy','Avy','Aay','Apz','Avz','Aaz']
    ForeArm_names=['Fq0','Fq1','Fq2','Fq3','FQp0','FQp1','FQp2','FQp3','FQo0','FQo1','FQo2','FQo3','Fpx','FVx','Fax','Fpy','Fvy','Fay','Fpz','Fvz','Faz']
    col_names.extend(Head_names)
    col_names.extend(Body_names)
    col_names.extend(Arm_names)
    col_names.extend(ForeArm_names)
    NToID={name:ID for ID,name in enumerate(col_names)}
    data = pd.read_csv("measures/e2_body_head@head.csv",names=col_names,header=None)
    data=data.applymap(lambda x: float(x.replace('[','').replace(']','').strip('\n')) if isinstance(x, str) else x)
    readData=ReadData("head")
    #checking if acceleration are the same 
    PosHead=np.array(PosHead)
    N=PosHead.shape[0]
    #Reshaping so there is no last dimension of size 1
    PosHead=PosHead.reshape(N,6)
    #select only first elments of dim 0
    PosHead=PosHead[:,4]

    plt.plot(PosHead[:44],data["Hpx"])
    plt.show()
def showResults(PosHead,PosBody,QuatHead,QuatBody,TimeHead,TimeBody):
      # PosHead and Body are (n,6) arrays
    PosHead = np.array(PosHead)
    PosHead=PosHead.reshape(PosHead.shape[0],6)
    PosBody = np.array(PosBody)
    PosBody=PosBody.reshape(PosBody.shape[0],6)
    # QuatHead and Body are (n,4) arrays
    QuatHead = np.array(QuatHead)
    QuatBody = np.array(QuatBody)
    xHead, yHead, zHead = PosHead[:,0], PosHead[:,2], PosHead[:,4]
    xBody, yBody, zBody = PosBody[:,0], PosBody[:,2], PosBody[:,4]

    fig, axs = plt.subplots(2, 1)
    #Take the norm of position on x and z axis
    axs[0].plot(TimeHead, np.sqrt(yHead**2 + zHead**2), label='Head')
    # Plot a red line at the 2m mark
    axs[0].axhline(y=2, color='r', linestyle='--')
    axs[0].set_title('Position of the head')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Distance (m)')
    axs[0].legend()
    #Take the position on y axis
    axs[1].plot(TimeHead, xHead, label='Head')
    # plot a red line at 0m mark
    axs[1].axhline(y=0, color='r', linestyle='--')
    axs[1].set_title('Position of the head')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Distance (m)')
    axs[1].legend()
    plt.show()



#########################################################################################
# Utility functions
#########################################################################################
# Quaternion and matrix operations
def quat_transformation(q, p):
    # q is the quaternion, p is the point to transform
    quat_p=np.array([0,p[0],p[1],p[2]])
    q_inv = quat_conjugate(q)
    q_p = quat_product(q,quat_product(quat_p,q_inv))
    return q_p[1:]
# Define a helper function for quaternion to DCM conversion
def q2dcm(q):
    rot = R.from_quat(q)
    return rot.as_matrix()

def ReadReferenceData():
     #loading the reference data
    col_names=['it','time0','time1']
    Head_names=['Hq0','Hq1','Hq2','Hq3','HQp0','HQp1','HQp2','HQp3','HQo0','HQo1','HQo2','HQo3','Hpx','HVx','Hax','Hpy','Hvy','Hay','Hpz','Hvz','Haz']
    Body_names=['Bq0','Bq1','Bq2','Bq3','BQp0','BQp1','BQp2','BQp3','BQo0','BQo1','BQo2','BQo3','Bpx','BVx','Bax','Bpy','Bvy','Bay','Bpz','Bvz','Baz']
    Arm_names=['Aq0','Aq1','Aq2','Aq3','AQp0','AQp1','AQp2','AQp3','AQo0','AQo1','AQo2','AQo3','Apx','AVx','Aax','Apy','Avy','Aay','Apz','Avz','Aaz']
    ForeArm_names=['Fq0','Fq1','Fq2','Fq3','FQp0','FQp1','FQp2','FQp3','FQo0','FQo1','FQo2','FQo3','Fpx','FVx','Fax','Fpy','Fvy','Fay','Fpz','Fvz','Faz']
    acc=['ax','ay','az']
    mag=['mx','my','mz']
    col_names.extend(Head_names)
    col_names.extend(Body_names)
    col_names.extend(Arm_names)
    col_names.extend(ForeArm_names)
    col_names.extend(acc)
    col_names.extend(mag)
    data = pd.read_csv("measures2m/e2_body_head@head.csv",names=col_names,header=None)
    data=data.applymap(lambda x: float(x.replace('[','').replace(']','').strip('\n')) if isinstance(x, str) else x)  
    return data 
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
def getAverageDeltat():
    HeadData= ReadData("head")
    itHead = 0
    itHeadmax=HeadData['it'].max()
    LastTime=HeadData['time'][0]
    Deltat=[]
    while(itHead< itHeadmax):
        Deltat.append(abs(HeadData['time'][itHead]-LastTime)/1000)
        LastTime=HeadData['time'][itHead]
        itHead+=1
    return np.mean(Deltat)
runExperimentWithoutConstraintWithoutQuat()
PosHead,PosBody,QuatHead,QuatBody,TimeHead,TimeBody=runExperimentWithoutConstraint()
# runExperimentConstraint()
# compareResults(PosHead,PosBody,QuatHead,QuatBody,TimeHead,TimeBody)
# showResults(PosHead,PosBody,QuatHead,QuatBody,TimeHead,TimeBody)

print(getAverageDeltat())