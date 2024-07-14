import numpy as np
from filterpy.kalman import KalmanFilter
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import ParameterGrid
import pandas as pd

# Define the objective function to minimize
def objective_function(params, P, DtBody,Qpos, DtHead,QHead, measurements, true_value):
    # Unpack the parameters
    process_noise = params['process_noise']
    measurement_noise = params['measurement_noise']

    # Define the Kalman Filter
    Body,Head = KalmanFilterBodyHeadModel( P, DtBody,Qpos, DtHead,QHead)


    # Predict and update using Kalman Filter
    filtered_values = []
    for measurement in measurements:
        Body.x = measurement
        xBody,pBody=Body.predict()
        xHead,pHead=Head.predict()
        Body.update(measurement)
    
    # Calculate mean squared error between filtered values and true values
    mse = mean_squared_error(true_value, filtered_values)
    
    return mse

# Define the parameter grid
param_grid = {'process_noise': np.linspace(0.01, 10, 10),
              'measurement_noise': np.linspace(0.01, 10, 10)}

# Generate all possible combinations of parameters
grid = ParameterGrid(param_grid)

# Perform grid search
best_mse = float('inf')
best_params = None
# for params in grid:
#     # mse = objective_function(params)
#     if mse < best_mse:
#         best_mse = mse
#         best_params = params

print("Best Parameters (Process Noise, Measurement Noise):", best_params)


# Def Body Head Model
# One kalman filters with just prediction step
# One with update step with measurement given by first prediciton

def KalmanFilterBodyHeadModel( P, DtBody,Qpos, DtHead,QHead):
    Body=KalmanFilter(dim_x=9, dim_z=6)
    Head=KalmanFilter(dim_x=9, dim_z=6)
    # Body
    Body.F = np.array([
                [1,DtBody,(DtBody*DtBody)/2,0,0,0,0,0,0],
                [0,1,DtBody,0,0,0,0,0,0], 
                [0,0,1,0,0,0,0,0,0], 
                [0,0,0,1,DtBody,(DtBody*DtBody)/2,0,0,0], 
                [0,0,0,0,1,DtBody,0,0,0], 
                [0,0,0,0,0,1,0,0,0], 
                [0,0,0,0,0,0,1,DtBody,(DtBody*DtBody)/2], 
                [0,0,0,0,0,0,0,1,DtBody], 
                [0,0,0,0,0,0,0,0,1] 
            ])
    Body.Q = Qpos  # Process noise covariance the parameters are to be tuned
    Body.P = P

    # Head
    Head.F = np.array([
                [1,DtHead,(DtHead*DtHead)/2,0,0,0,0,0,0],
                [0,1,DtHead,0,0,0,0,0,0], 
                [0,0,1,0,0,0,0,0,0], 
                [0,0,0,1,DtHead,(DtHead*DtHead)/2,0,0,0], 
                [0,0,0,0,1,DtHead,0,0,0], 
                [0,0,0,0,0,1,0,0,0], 
                [0,0,0,0,0,0,1,DtHead,(DtHead*DtHead)/2], 
                [0,0,0,0,0,0,0,1,DtHead], 
                [0,0,0,0,0,0,0,0,1]         
    ])
    Head.Q = QHead  # Process noise covariance the parameters are to be tuned
    Head.P = P
    Head.H =[
                [1,0,0,0,0,0,0,0,0 ],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,0,0,0,1,0,0], 
                [0,0,1,0,0,0,0,0,0],
                [0,0,0,0,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,1] ]

    return  Body, Head

def TuningE1():
    # Body,Head= KalmanFilterBodyHeadModel( P, DtBody,Qpos, DtHead,QHead)
    colnames=['id','time1','time2','xBody','vxBody','axBody','yBody','vyBody','ayBody','zBody','vzBody','azBody','xHead','vxHead','axHead','yHead','vyHead','ayHead','zHead','vzHead','azHead']
    e1_df = pd.read_csv('MEASURES/e1_body_head@head.csv',names=colnames,header=None)
    Times=e1_df['time1']
    colnamesData=['id','time','ax','ay','az','gx','gy','gz','mx','my','mz']
    Body_df = pd.read_csv('MEASURES/nav3_body_head@body.csv',names=colnamesData,header=None)
    Head_df = pd.read_csv('MEASURES/nav3_body_head@head.csv',names=colnamesData,header=None)
    print(Body_df.head())


TuningE1()