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


def quaternion_to_rotation_matrix(quaternion):
    q = np.array(quaternion)
    q /= np.linalg.norm(q)
    # rotation_matrix = np.array([
    #     [1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
    #     [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
    #     [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]
    # ])
    rotation_matix = np.array([
        [2*(q[0]**2+q[1]**2)-1, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2])],
        [2*(q[1]*q[2]+q[0]*q[3]), 2*(q[0]**2+q[2]**2)-1, 2*(q[2]*q[3]-q[0]*q[1])],
        [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]), 2*(q[0]**2+q[3]**2)-1]
    ])
    return rotation_matix

# reads the good data and does a frame
def animate(frame,ax,object_artist,card):
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

    
    rotation_matrix = quaternion_to_rotation_matrix(quaternion[0])
    transformed_vertices= []
    for vertex in object_vertices:
        transformed_vertices.append(np.matmul(np.array(rotation_matrix), np.array(vertex)) + position)
    transformed_vertices = np.array(transformed_vertices)
    x = transformed_vertices[:,0]
    y = transformed_vertices[:,1]
    z = transformed_vertices[:,2]
    # object_artist._offsets3d = (x, y, z)
    tri = Delaunay(transformed_vertices[:, :2])
    ax.plot_trisurf(x,y,z, triangles=tri.simplices,color='b')
    ax.set_xlim(-2 , 3)
    ax.set_ylim(-2 , 3)
    ax.set_zlim(-2, 3)  

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return object_artist,

def animate_debug_quaternion(frame,ax,object_artist,card,Body=True,Head=True,Arm=True,Forearm=True):
    time = []

    global ID
    global live
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
    data = pd.read_csv("measures/e2_body_head@"+card+".csv",names=col_names,header=None)
    
    if live:
        data=data.iloc[-1]
    else:
        data=data.iloc[ID]
        ID+=1  
    line=[element.replace('[','').replace(']','').strip('\n') if isinstance(element, str) else element for element in data]

    time.append(float(line[2]))
    ax.clear() 
    Headposition = []
    Headposition.extend([float(line[NToID['Hpx']]),float(line[NToID['Hpy']]),float(line[NToID['Hpz']])])
    if Body:
        print("body is printed")
        quaternion,quaternionPredicted,quaternionObserved,position = [],[],[],[]
        quaternion.append([float(line[NToID['Bq0']]), float(line[NToID['Bq1']]), float(line[NToID['Bq2']]), float(line[NToID['Bq3']])])
        quaternionPredicted.append([float(line[NToID['BQp0']]), float(line[NToID['BQp1']]), float(line[NToID['BQp2']]), float(line[NToID['BQp3']])])
        quaternionObserved.append([float(line[NToID['BQo0']]), float(line[NToID['BQo1']]), float(line[NToID['BQo2']]), float(line[NToID['BQo3']])])
        position.extend([float(line[NToID['Bpx']]), float(line[NToID['Bpy']]), float(line[NToID['Bpz']])])
        position = [position[0]-Headposition[0],position[1]-Headposition[1],position[2]-Headposition[2]]
        RotateAndPrint(ax,quaternion,quaternionPredicted,quaternionObserved,position,color='blue')
    if Head:
        Headquaternion,HeadquaternionPredicted,HeadquaternionObserved = [],[],[]
        Headquaternion.append([float(line[NToID['Hq0']]),float(line[NToID['Hq1']]),float(line[NToID['Hq2']]),float(line[NToID['Hq3']])])
        HeadquaternionPredicted.append([float(line[NToID['HQp0']]),float(line[NToID['HQp1']]),float(line[NToID['HQp2']]),float(line[NToID['HQp3']])])
        HeadquaternionObserved.append([float(line[NToID['HQo0']]),float(line[NToID['HQo1']]),float(line[NToID['HQo2']]),float(line[NToID['HQo3']])])
        if AttitudeOnly:
            RotateAndPrint(ax,Headquaternion,HeadquaternionPredicted,HeadquaternionObserved,[0,0,0],color='black')
        else:
            RotateAndPrint(ax,Headquaternion,HeadquaternionPredicted,HeadquaternionObserved,Headposition)#color='black')
    if Arm:
        Armquaternion,ArmquaternionPredicted,ArmquaternionObserved,Armposition = [],[],[],[]
        Armquaternion.append([float(line[NToID['Aq0']]),float(line[NToID['Aq1']]),float(line[NToID['Aq2']]),float(line[NToID['Aq3']])])
        ArmquaternionPredicted.append([float(line[NToID['AQp0']]),float(line[NToID['AQp1']]),float(line[NToID['AQp2']]),float(line[NToID['AQp3']])])
        ArmquaternionObserved.append([float(line[NToID['AQo0']]),float(line[NToID['AQo1']]),float(line[NToID['AQo2']]),float(line[NToID['AQo3']])])
        Armposition.extend([float(line[NToID['Apx']]),float(line[NToID['Apy']]),float(line[NToID['Apz']])])
        print("ArmQuaternion",Armquaternion)
        if AttitudeOnly:
            Armposition = [Armposition[0]-Headposition[0],Armposition[1]-Headposition[1],Armposition[2]-Headposition[2]]
        RotateAndPrint(ax,Armquaternion,ArmquaternionPredicted,ArmquaternionObserved,Armposition,color='red')

    if Forearm:
        Forearmquaternion,ForearmquaternionPredicted,ForearmquaternionObserved,Forearmposition = [],[],[],[]
        Forearmquaternion.append([float(line[NToID['Fq0']]),float(line[NToID['Fq1']]),float(line[NToID['Fq2']]),float(line[NToID['Fq3']])])
        ForearmquaternionPredicted.append([float(line[NToID['FQp0']]),float(line[NToID['FQp1']]),float(line[NToID['FQp2']]),float(line[NToID['FQp3']])])
        ForearmquaternionObserved.append([float(line[NToID['FQo0']]),float(line[NToID['FQo1']]),float(line[NToID['FQo2']]),float(line[NToID['FQo3']])])
        Forearmposition.extend([float(line[NToID['Fpx']]),float(line[NToID['Fpy']]),float(line[NToID['Fpz']])])
        if AttitudeOnly:
            Forearmposition = [Forearmposition[0]-Headposition[0],Forearmposition[1]-Headposition[1],Forearmposition[2]-Headposition[2]]
        RotateAndPrint(ax,Forearmquaternion,ForearmquaternionPredicted,ForearmquaternionObserved,Forearmposition,color='green')

    ax.set_xlim(-2 , 3)
    ax.set_ylim(-2 , 3)
    ax.set_zlim(-2, 3)

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return object_artist,

def RotateAndPrint(ax,quaternion,quaternionPredicted,quaternionObserved,position,color=False):
    # take the conjugate of the quaternion
    quaternion[0] = [quaternion[0][0], quaternion[0][1], quaternion[0][2], quaternion[0][3]]
    quaternionPredicted[0] = [quaternionPredicted[0][0], quaternionPredicted[0][1], quaternionPredicted[0][2], quaternionPredicted[0][3]]
    quaternionObserved[0] = [quaternionObserved[0][0], quaternionObserved[0][1], quaternionObserved[0][2], quaternionObserved[0][3]] 

    rotation_matrix=quaternion_to_rotation_matrix(quaternion[0])
    rotation_matrixPredicted=quaternion_to_rotation_matrix(quaternionPredicted[0])
    rotation_matrixObserved=quaternion_to_rotation_matrix(quaternionObserved[0])

    transformed_vertices= []
    transformed_verticesPredicted= []
    transformed_verticesObserved= []
    for vertex in object_vertices:
        transformed_vertices.append(np.matmul(np.array(rotation_matrix), np.array(vertex)) + position)
        transformed_verticesPredicted.append(np.matmul(np.array(rotation_matrixPredicted), np.array(vertex)) + position)
        transformed_verticesObserved.append(np.matmul(np.array(rotation_matrixObserved), np.array(vertex)) + position)
    transformed_vertices = np.array(transformed_vertices)
    transformed_verticesPredicted = np.array(transformed_verticesPredicted)
    transformed_verticesObserved = np.array(transformed_verticesObserved)

    # drawArrow(ax,position,rotation_matrix)
    drawArrow(ax,position,rotation_matrixPredicted,color=color,scale=2)
    # drawArrow(ax,position,rotation_matrixObserved,dashed=True,scale=2)
    #drawArrow(ax,position,rotation_matrix,arrowstyle=True)
def LiveAnimeation(DEBUG=False,card='head',Body=True,Head=True,Arm=True,Forearm=True):
    plt.rcParams["figure.figsize"] = [7.50, 3.50]
    plt.rcParams["figure.autolayout"] = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')
    ax.set_title('Object Visualization')
    
    scale_factor = 1000  # Scale factor for object vertices
    #Define object vertices (for visualization)
    global object_vertices
    object_vertices = np.array([
        [-0.00675, 0,-0.00375],
        [0.00675, 0,-0.00375],
        [0.00675, 0,0.00375],
        [-0.00675, 0,0.00375],
        [-0.00675, 0,-0.00375],
        [-0.00675, 0.002,-0.00375],
        [0.00675,0.002, -0.00375],
        [0.00675, 0.002,0.00375],
        [-0.00675, 0.002,0.00375],
        [-0.00675, 0.002,-0.00375],
    ])*scale_factor

    # Initial plot of object
    initial_quaternion = [1.0, 0.0, 0.0, 0.0]
    initial_position = [0,0,0]
    
    rotation_matrix = quaternion_to_rotation_matrix(initial_quaternion)
    transformed_vertices = np.dot(object_vertices, rotation_matrix.T) + initial_position

    transformed_vertices = np.array(transformed_vertices)
    x = transformed_vertices[:,0]
    y = transformed_vertices[:,1]
    z = transformed_vertices[:,2]
    
    object_artist = ax.plot_trisurf(x,y,z, color='r')
    
    # Set fixed axis limits
    ax.set_xlim(-2 , 3)
    ax.set_ylim(-2 , 3)
    ax.set_zlim(-2, 3)
    # Animate the plot
    if DEBUG:
        ani = FuncAnimation(fig, animate_debug_quaternion, fargs=(ax,object_artist,card,Body,Head,Arm,Forearm), interval=10, blit=True,cache_frame_data=False)
    else:
  
        ani = FuncAnimation(fig, animate, fargs=(ax,object_artist,card), interval=10, blit=True,cache_frame_data=False)

    plt.show()

def getAverageTime():
    data = pd.read_csv("measures/e2_body_head@head.csv",names=['it','time0','time1','q0','q1','q2','q3','px','Vx','ax','p1','vy','ay','p2','vz','az'],header=None)
    sum=0
    for i in range(0,len(data)-1):
        time=data.iloc[i+1][2]-data.iloc[i][2]
        sum+=time
    print(sum/len(data))
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

def transformInitSpate(rotation_matrix,position):
    transformed_arrowx  = np.matmul(np.array(rotation_matrix), np.array([1,0,0])) + position
    transformed_arrowy = np.matmul(np.array(rotation_matrix), np.array([0,1,0])) + position
    transformed_arrowz = np.matmul(np.array(rotation_matrix), np.array([0,0,1])) + position
    return transformed_arrowx,transformed_arrowy,transformed_arrowz

def transformArrow(x,y,z,rotation_matrix,position):
    transformed_arrow  = np.matmul(np.array(rotation_matrix), np.array([x,y,z])) + position 
    return transformed_arrow
def drawArrow(ax,position,rotation_matrix,dashed=False,arrowstyle=False,color=False,scale=1):
    transformed_arrowx ,transformed_arrowy,transformed_arrowz  = transformInitSpate(rotation_matrix,position)
    if color != False:
        print("color is printed",color)
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowx[0],transformed_arrowx[1],transformed_arrowx[2],mutation_scale=10*scale,fc=color)
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowy[0],transformed_arrowy[1],transformed_arrowy[2],mutation_scale=10*scale,fc=color)
        ax.arrow3D(position[0],position[1],position[2],transformed_arrowz[0],transformed_arrowz[1],transformed_arrowz[2],mutation_scale=10*scale,fc=color)
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

ID = 0
live = False
AttitudeOnly = True
LiveAnimeation(DEBUG=True,card='head',Body=True,Head=True,Arm=True,Forearm=True)
# getAverageTime()
