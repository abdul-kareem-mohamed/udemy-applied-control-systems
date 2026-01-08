import numpy as np
import matplotlib.pyplot as plt
import support_fns as sfns
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

# Create an object for the support functions.
support = sfns.SupportFnsCar()
constants = support.constants

# Load the constant values needed in the main file
Ts=constants['Ts']
outputs=constants['outputs'] # number of outputs (psi, Y)
hz = constants['hz'] # horizon prediction period
x_dot=constants['x_dot'] # constant longitudinal velocity
time_length=constants['time_length'] # duration of the manoeuvre

#Generate the reference signals
t=np.arange(0, time_length+Ts, Ts) # time from 0 to 10 secs, sample time (Ts)
r = constants['r']
f=constants['f']
psi_ref,X_ref,Y_ref=support.trajectory_generator(t,r,f)
sim_length=len(t) # Number of control loop iterations
refSignals=np.zeros(len(X_ref)*outputs) # [psi_ref[0], Y_ref[1], psi_ref[1], Y_ref[1], .... , psi_ref[len(t)-1], Y_ref[len(t)-1]]

# use refSignals inside control loop for error calculation
# Build up the reference signal vector:
# refSignal = [psi_ref_0, Y_ref_0, psi_ref_1, Y_ref_1, psi_ref_2, Y_ref_2, ... etc.]
k=0
for i in range(0,len(refSignals),outputs):
    refSignals[i]=Y_ref[k]
    refSignals[i+1]=psi_ref[k]
    k=k+1

# Load the initial states of car
y_dot=0.
psi=0.
psi_dot=0.
Y=Y_ref[0]+10. # initial vertical position of car

states=np.array([y_dot,psi,psi_dot,Y])
statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states
psi_opt_total=np.zeros((len(t),hz)) # vector for optimal yaw angles from MPC
Y_opt_total=np.zeros((len(t),hz)) # vector for optimal Y position from MPC

# Load the initial input
input1=0 # Input at t = -1 s (steering wheel angle in rad (delta))
inputTotal=np.zeros(len(t)) # To keep track all your inputs over time
inputTotal[0]=input1

# Generate the discrete state space matrices
Ad,Bd,Cd,Dd=support.state_space()


# Generate the compact simplification matrices for the cost function
# The matrices (Hdb,Fdbt,Cdb,Adc) stay mostly constant during the simulation.
# Therefore, it is more efficient to generate them here before you start the simulation loop.
# However, in the end of the simulation, the horizon period (hz) will start decreasing.
# That is when the matrices need to be regenerated (done inside the simulation loop)
Hdb,Fdbt,Cdb,Adc=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)

# To extract psi_opt from predicted x_aug_opt
C_psi_opt=np.zeros((hz,(len(states)+np.size(input1))*hz))
for i in range(1,hz+1):
    C_psi_opt[i-1][i+4*(i-1)]=1

# To extract Y_opt from predicted x_aug_opt
C_Y_opt=np.zeros((hz,(len(states)+np.size(input1))*hz))
for i in range(3,hz+3):
    C_Y_opt[i-3][i+4*(i-3)]=1


# Initiate the controller - simulation loops
k=0
for i in range(0,sim_length-1):

    # Generate the augmented current state and the reference vector
    x_aug_t=np.transpose([np.concatenate((states,[input1]),axis=0)])

    # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
    # Example: t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
    # r=[psi_ref_3.1, Y_ref_3.1, psi_ref_3.2, Y_ref_3.2, ... , psi_ref_4.5, Y_ref_4.5]
    # With each loop, it all shifts by 0.1 second because Ts=0.1 s
    k=k+outputs
    if k+outputs*hz<=len(refSignals):
        r=refSignals[k:k+outputs*hz]
    else:
        r=refSignals[k:len(refSignals)]
        hz=hz-1

    if hz<constants['hz']: # Check if hz starts decreasing
        # These matrices (Hdb,Fdbt,Cdb,Adc) were created earlier at the beginning of the loop.
        # They constant almost throughout the entire simulation. However,
        # in the end of the simulation, the horizon period (hz) starts decreasing.
        # Therefore, the matrices need to be constantly updated in the end of the simulation.
        Hdb,Fdbt,Cdb,Adc=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)

    ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
    du=-np.matmul(np.linalg.inv(Hdb),np.transpose([ft]))
    x_aug_opt=np.matmul(Cdb,du)+np.matmul(Adc,x_aug_t)
    psi_opt=np.matmul(C_psi_opt[0:hz,0:(len(states)+np.size(input1))*hz],x_aug_opt)
    Y_opt=np.matmul(C_Y_opt[0:hz,0:(len(states)+np.size(input1))*hz],x_aug_opt)
    # if hz<4:
    #     print(x_aug_opt)
    psi_opt=np.transpose((psi_opt))[0]
    psi_opt_total[i+1][0:hz]=psi_opt
    Y_opt=np.transpose((Y_opt))[0]
    Y_opt_total[i+1][0:hz]=Y_opt

    # exit()

    # Update the real inputs
    input1=input1+du[0][0]

    # Establish the limits for the real inputs (max: pi/6 radians)

    if input1 < -np.pi/6:
        input1=-np.pi/6
    elif input1 > np.pi/6:
        input1=np.pi/6
    else:
        input1=input1

    # Keep track of your inputs as you go from t=0 --> t=7 seconds
    inputTotal[i+1]=input1

    # Compute new states in the open loop system (interval: Ts/30)
    states=support.open_loop_new_states(states,input1)
    statesTotal[i+1][0:len(states)]=states



# Plot the world
plt.plot(X_ref,Y_ref,'b',linewidth=2,label='The trajectory')
plt.plot(X_ref,statesTotal[:,3],'--r',linewidth=2,label='Car position')
plt.xlabel('x-position [m]',fontsize=15)
plt.ylabel('y-position [m]',fontsize=15)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')
plt.ylim(-X_ref[-1]/2,X_ref[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
plt.show()


# Plot the the input delta(t) and the outputs: psi(t) and Y(t)
plt.subplot(3,1,1)
plt.plot(t,inputTotal[:],'r',linewidth=2,label='steering wheel angle')
plt.xlabel('t-time [s]',fontsize=15)
plt.ylabel('steering wheel angle [rad]',fontsize=15)
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plt.subplot(3,1,2)
plt.plot(t,psi_ref,'b',linewidth=2,label='Yaw_ref angle')
plt.plot(t,statesTotal[:,1],'--r',linewidth=2,label='Car yaw angle')
plt.xlabel('t-time [s]',fontsize=15)
plt.ylabel('psi_ref-position [rad]',fontsize=15)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')

plt.subplot(3,1,3)
plt.plot(t,Y_ref,'b',linewidth=2,label='Y_ref position')
plt.plot(t,statesTotal[:,3],'--r',linewidth=2,label='Car Y position')
plt.xlabel('t-time [s]',fontsize=15)
plt.ylabel('y-position [m]',fontsize=15)
plt.grid(True)
plt.legend(loc='center right',fontsize='small')
plt.show()
