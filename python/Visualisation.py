import time
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation
from scipy.spatial import Delaunay
import pandas as pd

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d.axes3d import Axes3D

ID = 0
live = True
ATTITUDEONLY = True

# reads the good data and does a frame
def animate(frame,ax,card):
    time = []
    quaternion = []
    position = []
    global ID
    global live
    data = pd.read_csv("measures/e2_body_head@"+card+".csv",names=['it','time0','time1','q0','q1','q2','q3','px','Vx','ax','p1','vy','ay','p2','vz','az'],header=None)
    
    if live:
        data=data.iloc[-1]
    else:
        data=data.iloc[ID]
        ID+=1
    line=[element.replace('[','').replace(']','').strip('\n') if isinstance(element, str) else element for element in data]
    time.append(float(line[2]))
    quaternion.append([float(line[3]), float(line[4]), float(line[5]), float(line[6])])
    position.append([float(line[7]), float(line[10]), float(line[13])])
    
    ax.clear()  # Clear the previous plot
    # position = position_data[frame]
    position = [0,0,0]

    

    # object_artist._offsets3d = (x, y, z)
    ax.set_xlim(-2 , 3)
    ax.set_ylim(-2 , 3)
    ax.set_zlim(-2, 3)  

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return 

def animate_debug_quaternion(frame,ax,card,ntoid,body=True,head=True,arm=True,forearm=True):
    time = []

    global ID
    global live

    
    data = pd.read_csv(card,header=None)
    
    if live:
        data=data.iloc[-1]
    else:
        data=data.iloc[ID]
        ID+=1  
    line=[float(element.replace('[','').replace(']','').strip('\n')) if isinstance(element, str) else element for element in data]
    state_dict={name:line[ntoid[name]] for name in ntoid.keys()}
    time.append(float(line[2]))
    ax.clear() 
    head_position = []
    head_position.extend([state_dict['hpx'],state_dict['hpy'],state_dict['hpz']])
    if body:
        body_quat,body_predicted_quat,body_observed_quat,body_position = [],[],[],[]
        body_quat.append([state_dict['bq0'], state_dict['bq1'], state_dict['bq2'], state_dict['bq3']])
        body_position.extend([state_dict['bpx'], state_dict['bpy'], state_dict['bpz']])
        if ATTITUDEONLY:
            body_position = [body_position[0]-head_position[0],body_position[1]-head_position[1],body_position[2]-head_position[2]]
        rotate_and_plot(ax,body_quat,body_position,color='blue')
        if 'BQp0' in state_dict.keys():
            body_predicted_quat.append([state_dict['BQp0'], state_dict['BQp1'], state_dict['BQp2'], state_dict['BQp3']])
            rotate_and_plot(ax,body_predicted_quat,body_position,color='blue')
        if 'BQo0' in state_dict.keys():
            body_observed_quat.append([state_dict['BQo0'], state_dict['BQo1'], state_dict['BQo2'], state_dict['BQo3']])
            rotate_and_plot(ax,body_observed_quat,body_position,color='blue')
        
    if head:
        head_quat,head_predicted_quat,head_observed_quat = [],[],[]
        head_quat.append([state_dict['hq0'],state_dict['hq1'],state_dict['hq2'],state_dict['hq3']])

        if ATTITUDEONLY:
            rotate_and_plot(ax,head_quat,[0,0,0],color='black',dashed=True) # Black
        # if 'HQp0' in state_dict.keys():
        #     head_predicted_quat.append([state_dict['HQp0'],state_dict['HQp1'],state_dict['HQp2'],state_dict['HQp3']])
        #     rotate_and_plot(ax,head_predicted_quat,head_position,color='black')
        # if 'HQo0' in state_dict.keys():
        #     head_observed_quat.append([state_dict['HQo0'],state_dict['HQo1'],state_dict['HQo2'],state_dict['HQo3']])
        #     rotate_and_plot(ax,head_observed_quat,head_position,color='black')
    if arm:
        arm_quat,arm_predicted_quat,arm_observed_quat,arm_position = [],[],[],[]
        arm_quat.append([state_dict['aq0'],state_dict['aq1'],state_dict['aq2'],state_dict['aq3']])

        arm_position.extend([state_dict['apx'],state_dict['apy'],state_dict['apz']])
        if ATTITUDEONLY:
            arm_position = [arm_position[0]-head_position[0],arm_position[1]-head_position[1],arm_position[2]-head_position[2]]
        rotate_and_plot(ax,arm_quat,arm_position,color='red')
        if 'AQp0' in state_dict.keys():
            arm_predicted_quat.append([state_dict['AQp0'],state_dict['AQp1'],state_dict['AQp2'],state_dict['AQp3']])
            rotate_and_plot(ax,arm_predicted_quat,arm_position,color='red')
        if 'AQo0' in state_dict.keys():
            arm_observed_quat.append([state_dict['AQo0'],state_dict['AQo1'],state_dict['AQo2'],state_dict['AQo3']])
            rotate_and_plot(ax,arm_observed_quat,arm_position,color='red')

    if forearm:
        forearm_quat,forearm_predicted_quat,forearm_observed_quat,forearm_position = [],[],[],[]
        forearm_quat.append([state_dict['fq0'],state_dict['fq1'],state_dict['fq2'],state_dict['fq3']])
        forearm_position.extend([state_dict['fpx'],state_dict['fpy'],state_dict['fpz']])
        if ATTITUDEONLY:
            forearm_position = [forearm_position[0]-head_position[0],forearm_position[1]-head_position[1],forearm_position[2]-head_position[2]]
        rotate_and_plot(ax,forearm_quat,forearm_position,color='green')
        if 'FQp0' in state_dict.keys():
            forearm_predicted_quat.append([state_dict['FQp0'],state_dict['FQp1'],state_dict['FQp2'],state_dict['FQp3']])
            rotate_and_plot(ax,forearm_predicted_quat,forearm_position,color='green')
        if 'FQo0' in state_dict.keys():
            forearm_observed_quat.append([state_dict['FQo0'],state_dict['FQo1'],state_dict['FQo2'],state_dict['FQo3']])
            rotate_and_plot(ax,forearm_observed_quat,forearm_position,color='green')        

    if body and head:
        head_shoulder_vector = quat_transformation(head_quat[0],np.array([0,-0.25,0]))
        head_body_vector = quat_transformation(head_quat[0],np.array([-0.3,0,0]))
        if ATTITUDEONLY:
            head_position = [0,0,0]
        shoulder_position=head_position+head_shoulder_vector
        #Draw a line from Head to Shoulder
        ax.plot([head_position[0],head_position[0]+head_shoulder_vector[0]],
                [head_position[1],head_position[1]+head_shoulder_vector[1]],
                [head_position[2],head_position[2]+head_shoulder_vector[2]],color='black')
        #Draw a line from Head to Body
        ax.plot([head_position[0],head_position[0]+head_body_vector[0]],
                [head_position[1],head_position[1]+head_body_vector[1]],
                [head_position[2],head_position[2]+head_body_vector[2]],color='black')

        shoulder_arm_vector = quat_transformation(arm_quat[0],np.array([-0.15,0,0]))
        #Draw a line from arm to Shoulder
        ax.plot([shoulder_position[0],shoulder_position[0]+shoulder_arm_vector[0]],
                [shoulder_position[1],shoulder_position[1]+shoulder_arm_vector[1]],
                [shoulder_position[2],shoulder_position[2]+shoulder_arm_vector[2]],color='red')

        elbow_position=shoulder_position+2*shoulder_arm_vector
        ax.plot([shoulder_position[0],elbow_position[0]],
                [shoulder_position[1],elbow_position[1]],
                [shoulder_position[2],elbow_position[2]],color='red')

        elbow_forearm_vector = quat_transformation(forearm_quat[0],np.array([-0.15,0,0]))
        #Draw a line from Forearm to Elbow
        ax.plot([elbow_position[0],elbow_position[0]+elbow_forearm_vector[0]],
                [elbow_position[1],elbow_position[1]+elbow_forearm_vector[1]],
                [elbow_position[2],elbow_position[2]+elbow_forearm_vector[2]],color='green')

    
    ax.set_xlim(-1 , 1)
    ax.set_ylim(-1 , 1)
    ax.set_zlim(-1, 1)

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return 

def rotate_and_plot(ax,quaternion,position,color=False,dashed=False):
    # take the conjugate of the quaternion
    quaternion[0] = [quaternion[0][0], quaternion[0][1], quaternion[0][2], quaternion[0][3]]
    draw_arrow(ax,position,quaternion[0],dashed=dashed,color=color)

def live_animation(card,ntoid,DEBUG=False,body=True,head=True,arm=True,forearm=True):
    plt.rcParams["figure.figsize"] = [7.50, 3.50]
    plt.rcParams["figure.autolayout"] = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')
    ax.set_title('Object Visualization')
    
    initial_quaternion = [1.0, 0.0, 0.0, 0.0]
    initial_position = [0,0,0]
    
    rotate_and_plot(ax,[initial_quaternion],initial_position,color='black')
    # Set fixed axis limits
    ax.set_xlim(-2 , 3)
    ax.set_ylim(-2 , 3)
    ax.set_zlim(-2, 3)
    # Animate the plot
    if DEBUG:
        ani = FuncAnimation(fig, animate_debug_quaternion, fargs=(ax,card,ntoid,body,head,arm,forearm), interval=40, blit=True,cache_frame_data=False)
    else:
  
        ani = FuncAnimation(fig, animate, fargs=(ax,card), interval=10, blit=True,cache_frame_data=False)

    plt.show()

def get_average_time():
    data = pd.read_csv("measures/e3_body_head@head.csv",names=['it','time0','time1','q0','q1','q2','q3','px','Vx','ax','p1','vy','ay','p2','vz','az'],header=None)
    sum=0
    for i in range(0,len(data)-1):
        time=data.iloc[i+1][2]-data.iloc[i][2]
        sum+=time
    return sum/len(data)

class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)
        
    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs) 
def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''

    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)


setattr(Axes3D, 'arrow3D', _arrow3D) 

def arrow_to_global(quaternion,position,scale=1):
    transformed_arrowx  = quat_transformation( quaternion,np.array([1*scale,0,0])) + position
    transformed_arrowy = quat_transformation( quaternion,np.array([0,1*scale,0]),) + position
    transformed_arrowz = quat_transformation(quaternion,np.array([0,0,1*scale])) + position
    return transformed_arrowx,transformed_arrowy,transformed_arrowz


def draw_arrow(ax,position,quaternion,dashed=False,arrowstyle=False,color=False,scale=0.10):
    transformed_arrowx ,transformed_arrowy,transformed_arrowz  = arrow_to_global(quaternion,position,scale=scale)
    if color != False:
        # ax.arrow3D(position[0],position[1],position[2],transformed_arrowx[0],transformed_arrowx[1],transformed_arrowx[2],mutation_scale=10*scale,fc=color)
        ax.plot([position[0],transformed_arrowx[0]],[position[1],transformed_arrowx[1]],[position[2],transformed_arrowx[2]],color='blue')
        ax.plot([position[0],transformed_arrowy[0]],[position[1],transformed_arrowy[1]],[position[2],transformed_arrowy[2]],color='green')
        ax.plot([position[0],transformed_arrowz[0]],[position[1],transformed_arrowz[1]],[position[2],transformed_arrowz[2]],color='red')
        # ax.arrow3D(position[0],position[1],position[2],transformed_arrowy[0],transformed_arrowy[1],transformed_arrowy[2],mutation_scale=10*scale,fc=color)
        # ax.arrow3D(position[0],position[1],position[2],transformed_arrowz[0],transformed_arrowz[1],transformed_arrowz[2],mutation_scale=10*scale,fc=color)
        return
    if dashed:
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowx[0],transformed_arrowx[1],transformed_arrowx[2],mutation_scale=10*scale,fc='blue',linestyle='dashed')
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowy[0],transformed_arrowy[1],transformed_arrowy[2],mutation_scale=10*scale,fc='green',linestyle='dashed')
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowz[0],transformed_arrowz[1],transformed_arrowz[2],mutation_scale=10*scale,fc='red',linestyle='dashed')
    elif arrowstyle:
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowx[0],transformed_arrowx[1],transformed_arrowx[2],mutation_scale=10*scale,fc='blue',arrowstyle='->')
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowy[0],transformed_arrowy[1],transformed_arrowy[2],mutation_scale=10*scale,fc='green',arrowstyle='->')
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowz[0],transformed_arrowz[1],transformed_arrowz[2],mutation_scale=10*scale,fc='red',arrowstyle='->')
    else:
        ax.arrow3D(position[0],position[1],position[2],position[0]+transformed_arrowx[0],position[1]+transformed_arrowx[1],position[2]+transformed_arrowx[2],mutation_scale=10*scale,fc='blue',arrowstyle='->')
        ax.arrow3D(position[0],position[1],position[2],position[0]+transformed_arrowy[0],position[1]+transformed_arrowy[1],position[2]+transformed_arrowy[2],mutation_scale=10*scale,fc='green',arrowstyle='->')
        ax.arrow3D(position[0],position[1],position[2],position[0]+transformed_arrowz[0],position[1]+transformed_arrowz[1],position[2]+transformed_arrowz[2],mutation_scale=10*scale,fc='red',arrowstyle='->')

def quat_transformation(quat, vector):
    p_quat = np.array([0, vector[0], vector[1], vector[2]])
    q_conj = np.array([quat[0], -quat[1], -quat[2], -quat[3]])
    p_quat_conj = quat_product(quat_product(quat,p_quat), q_conj)
    return p_quat_conj[1:]
def quat_product(q1, q2):
    q11, q12, q13, q14 = q1
    q21, q22, q23, q24 = q2
    return np.array([q11*q21 - q12*q22 - q13*q23 - q14*q24,
                        q11*q22 + q12*q21 + q13*q24 - q14*q23,
                        q11*q23 - q12*q24 + q13*q21 + q14*q22,
                        q11*q24 + q12*q23 - q13*q22 + q14*q21])

# col_names=['it','time0','time1']
# Head_names=['hq0','hq1','hq2','hq3','HQp0','HQp1','HQp2','HQp3','HQo0','HQo1','HQo2','HQo3','hpx','HVx','Hax','hpy','Hvy','Hay','hpz','Hvz','Haz']
# Body_names=['bq0','bq1','bq2','bq3','BQp0','BQp1','BQp2','BQp3','BQo0','BQo1','BQo2','BQo3','bpx','BVx','Bax','bpy','Bvy','Bay','bpz','Bvz','Baz']
# arm_names=['aq0','aq1','aq2','aq3','AQp0','AQp1','AQp2','AQp3','AQo0','AQo1','AQo2','AQo3','apx','AVx','Aax','apy','Avy','Aay','apz','Avz','Aaz']
# Forearm_names=['fq0','fq1','fq2','fq3','FQp0','FQp1','FQp2','FQp3','FQo0','FQo1','FQo2','FQo3','fpx','FVx','Fax','fpy','Fvy','Fay','fpz','Fvz','Faz']
# # DATA=["ax","ay","az","mx","my","mz"]
# col_names.extend(Head_names)
# col_names.extend(Body_names)
# col_names.extend(arm_names)
# col_names.extend(Forearm_names)
# # col_names.extend(DATA)
# ntoid={name:ID for ID,name in enumerate(col_names)}
# live_animation("measures/e2_body_head@head.csv",ntoid,DEBUG=True,Body=True,Head=True,arm=True,forearm=True)
# # get_average_time()