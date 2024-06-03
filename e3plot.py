import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
def ReadData(card):
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
    data = pd.read_csv("measures/e3_body_head@"+card+".csv",names=col_names,header=None)
    
    data=data.applymap(lambda x: float(x.replace('[','').replace(']','').strip('\n')) if isinstance(x, str) else x)
    return data,NToID
def plotXZ(data,ax):#North and East position
    ax.plot(abs(data['time1']-data['time1'][0])/1000,np.sqrt(data['Hpx']**2+data['Hpz']**2),label='Head')
    # ax.plot(data['time0'],np.sqrt(data['Bpx']**2+data['Bpz']**2),label='Body')
    # ax.plot(data['time0'],np.sqrt(data['Apx']**2+data['Apz']**2),label='Arm')
    # ax.plot(data['time0'],np.sqrt(data['Fpx']**2+data['Fpz']**2),label='ForeArm')

    #plot the 2m line for reference
    ax.axhline(y=2, color='r', linestyle='--')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.legend()
def plotY(data,ax):#North and East position
    ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Hpy'],label='Head')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Bpy'],label='Body')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Apy'],label='Arm')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Fpy'],label='ForeArm')
    #plot the reference line
    ax.axhline(y=0, color='r', linestyle='--')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [m]')
    ax.legend()
def plotVXZ(data,ax):#North and East position
    ax.plot(abs(data['time1']-data['time1'][0])/1000,np.sqrt(data['HVx']**2+data['Hvz']**2),label='Head')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,np.sqrt(data['BVx']**2+data['Bvz']**2),label='Body')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,np.sqrt(data['AVx']**2+data['Avz']**2),label='Arm')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,np.sqrt(data['FVx']**2+data['Fvz']**2),label='ForeArm')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.legend()
def plotVY(data,ax):# Up axis
    ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Hvy'],label='Head')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Bvy'],label='Body')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Avy'],label='Arm')
    # ax.plot(abs(data['time1']-data['time1'][0])/1000,data['Fvy'],label='ForeArm')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.legend()
def plote3(card):
    data,NToID=ReadData(card)
    fig, axs = plt.subplots(2, 1)
    plotXZ(data,axs[0])
    plotY(data,axs[1])
    # plotVXZ(data,axs[2])
    fig.tight_layout()
    fig.savefig("figures/HighspeedPrediction.pdf")
    fig, axs = plt.subplots(2, 1)
    plotVXZ(data,axs[0])
    plotVY(data,axs[1])
    fig.tight_layout()
    fig.savefig("figures/HighspeedPredictionVelocity.pdf")
    
    plt.show()
plote3("head")

def averageDeltat(card):
    data,NToID=ReadData(card)
    mean=np.mean(np.diff(data['time1']))/1000
    print("Mean Delta t: ",mean)
    print(mean/100)
    return mean/100
averageDeltat("head")

def BarPlotForDeltat(PreviousdeltaT):
    CurrentDeltaT=averageDeltat("head")
    fig, ax = plt.subplots()
    ax.bar(["Naive implementation","Fast computing implementation"],[PreviousdeltaT,CurrentDeltaT])
    ax.set_ylabel('Time [s]')
    fig.savefig("figures/deltaTcomparison.pdf")
    plt.show()
# BarPlotForDeltat(0.42)

def BarPlotForDeltatmeasurement(PreviousdeltaT):
    CurrentDeltaT=averageDeltat("head")*100
    fig, ax = plt.subplots()
    ax.bar(["Naive implementation","Fast computing implementation"],[PreviousdeltaT,CurrentDeltaT])
    ax.set_ylabel('Time [s]')
    fig.savefig("figures/deltatmeasurement.pdf")
    plt.show()
# BarPlotForDeltatmeasurement(0.42)