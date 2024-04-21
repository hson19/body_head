import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation
from scipy.spatial import Delaunay
import pandas as pd




def quaternion_to_rotation_matrix(quaternion):
    q = np.array(quaternion)
    q /= np.linalg.norm(q)
    rotation_matrix = np.array([
        [1 - 2*q[2]**2 - 2*q[3]**2, 2*q[1]*q[2] - 2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
        [2*q[1]*q[2] + 2*q[0]*q[3], 1 - 2*q[1]**2 - 2*q[3]**2, 2*q[2]*q[3] - 2*q[0]*q[1]],
        [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1], 1 - 2*q[1]**2 - 2*q[2]**2]
    ])
    return rotation_matrix

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
    ax.set_xlim(-20 , 30)
    ax.set_ylim(-20 , 30)
    ax.set_zlim(-20, 30)  

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return object_artist,

def animate_debug_quaternion(frame,ax,object_artist,card):
    time = []
    quaternion = []
    quaternionPredicted = []
    quaternionObserved = []
    position = []
    global ID
    global live
    data = pd.read_csv("measures/e2_body_head@"+card+".csv",names=['it','time0','time1','q0','q1','q2','q3','Qp0','Qp1','Qp2','Qp3','Qo0','Qo1','Qo2','Qo3','px','Vx','ax','p1','vy','ay','p2','vz','az'],header=None)
    if live:
        data=data.iloc[-1]
    else:
        data=data.iloc[ID]
        ID+=1  
    line=[element.replace('[','').replace(']','').strip('\n') if isinstance(element, str) else element for element in data]
    time.append(float(line[2]))
    quaternion.append([float(line[3]), float(line[4]), float(line[5]), float(line[6])])
    quaternionPredicted.append([float(line[7]), float(line[8]), float(line[9]), float(line[10])])
    quaternionObserved.append([float(line[11]), float(line[12]), float(line[13]), float(line[14])])
    position.append([float(line[15]), float(line[18]), float(line[21])])

    ax.clear()  # Clear the previous plot
    # position = position_data[frame]
    position = [0,0,0]

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

    x = transformed_vertices[:,0]
    y = transformed_vertices[:,1]
    z = transformed_vertices[:,2]
    xPredicted = transformed_verticesPredicted[:,0]
    yPredicted = transformed_verticesPredicted[:,1]
    zPredicted = transformed_verticesPredicted[:,2]
    xObserved = transformed_verticesObserved[:,0]
    yObserved = transformed_verticesObserved[:,1]
    zObserved = transformed_verticesObserved[:,2]


    tri = Delaunay(transformed_verticesPredicted[:, :2])
    ax.plot_trisurf(xPredicted,yPredicted,zPredicted, triangles=tri.simplices,color='r',label='Predicted')

    tri = Delaunay(transformed_verticesObserved[:, :2])
    ax.plot_trisurf(xObserved,yObserved,zObserved, triangles=tri.simplices,color='b',label='Observed')
    
    # tri = Delaunay(transformed_vertices[:, :2])
    # ax.plot_trisurf(x,y,z, triangles=tri.simplices,color='g',label='Kalman Filtered')

    plt.legend()

    ax.set_xlim(-20 , 30)
    ax.set_ylim(-20 , 30)
    ax.set_zlim(-20, 30)

    ax.set_xlabel('North')
    ax.set_ylabel('UP')
    ax.set_zlabel('East')

    return object_artist,


def LiveAnimeation(DEBUG=False,card='body'):
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
        ani = FuncAnimation(fig, animate_debug_quaternion, fargs=(ax,object_artist,card), interval=10, blit=True,cache_frame_data=False)
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

ID = 0
live = True
LiveAnimeation(DEBUG=True,card='head')
# getAverageTime()
