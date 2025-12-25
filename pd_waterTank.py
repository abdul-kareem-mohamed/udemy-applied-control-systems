import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random

# Inputs
density = 1000 # [kg/m^3]

# Tune the constants
K_p1 = 1000
K_p2 = 1000
K_p3 = 5000
#K_d = 50

# time step
dt = 0.04
t_end = 50
t0 = 0
t = np.arange(t0, t_end+dt, dt)

# water_tank 
volume_1 = np.zeros(len(t))
volume_2 = np.zeros(len(t))
volume_3 = np.zeros(len(t))
vol_r1 = np.zeros(len(t))
vol_r2 = np.zeros(len(t))
vol_r3 = np.zeros(len(t))
e1 = np.zeros(len(t)) # error
e2 = np.zeros(len(t)) # error
e3 = np.zeros(len(t)) # error

# initial_conditions
init_vol_1 = 10
init_vol_2 = 20
init_vol_3 = 30
volume_1[0] = init_vol_1
volume_2[0] = init_vol_2
volume_3[0] = init_vol_3

# reference_volume
init_vol_ref_1 = 70
init_vol_ref_2 = 10
init_vol_ref_3 = 20
vol_r1[0] = init_vol_ref_1
vol_r2[0] = init_vol_ref_2
vol_r3[0] = init_vol_ref_3

m_dot1 = K_p1*e1
m_dot2 = K_p2*e2
m_dot3 = K_p3*e3

# Implement PID
for i in range(1, len(t)):
    if i<300:
    	# Determine reference value vector for tank 1 and 2 for this region, if i is less than 300
        vol_r1[i]=init_vol_ref_1
        vol_r2[i]=init_vol_ref_2+3*t[i] # linear function
        vol_r3[i]=init_vol_ref_3+1*t[i]*np.sin(2*np.pi*(0.005*t[i])*t[i])
    elif i<600:
    	# Determine reference value vector for tank 1 and 2 for this region, if i is less than 600, but greater than 300
        vol_r1[i]=20
        vol_r2[i]=init_vol_ref_2+3*t[i]
        vol_r3[i]=init_vol_ref_3+1*t[i]*np.sin(2.5*np.pi*(0.005*t[i])*t[i])
        time_temp2=t[i]
        temp2=vol_r2[i]
    elif i<900:
    	# Determine reference value vector for tank 1 and 2 for this region, if i is less than 900
        vol_r1[i]=90
        vol_r2[i]=temp2-1*(t[i]-time_temp2)
        vol_r3[i]=init_vol_ref_3+1*t[i]*np.sin(3*np.pi*(0.005*t[i])*t[i])
    else:
    	# Determine reference value vector for tank 1 and 2 for this region, if i is greater or equal than 900
        vol_r1[i]=50
        vol_r2[i]=temp2-1*(t[i]-time_temp2)
        vol_r3[i]=init_vol_ref_3+1*t[i]*np.sin(3.25*np.pi*(0.005*t[i])*t[i])

    # Compute the errors between the reference values and the true values for tanks 1, 2, 3
    e1[i-1]=vol_r1[i-1]-volume_1[i-1]
    e2[i-1]=vol_r2[i-1]-volume_2[i-1]
    e3[i-1]=vol_r3[i-1]-volume_3[i-1]

    # Compute the control inputs for all the tanks
    m_dot1[i]=K_p1*e1[i-1]
    m_dot2[i]=K_p2*e2[i-1]
    m_dot3[i]=K_p3*e3[i-1]

    # Compute the true tank volumes in the next time step through this numerical integration (trapezoidal rule)
    volume_1[i]=volume_1[i-1]+(m_dot1[i-1]+m_dot1[i])/(2*density)*(dt)
    volume_2[i]=volume_2[i-1]+(m_dot2[i-1]+m_dot2[i])/(2*density)*(dt)
    volume_3[i]=volume_3[i-1]+(m_dot3[i-1]+m_dot3[i])/(2*density)*(dt)

# Start the animation
radius=5 # Radius of the tank - The tank is round.
bottom=0 # Initial volume of the tank
final_volume=100 # Final volume of the tank
dVol=10 # The change of volume on the vertical scale.
width_ratio=1 #Necessary for the horizontal axis
frame_amount=int(t_end/dt) # Frame amount of the simulation
vol_r1_2=vol_r1
vol_r2_2=vol_r2
vol_r3_2=vol_r3

def update_plot(num):
    if num>=len(volume_1):
        num=len(volume_1)-1

    tank_12.set_data([0,0],[-80,volume_1[num]-80])
    tnk_1.set_data(t[0:num],volume_1[0:num])
    vol_r1.set_data([-radius*width_ratio,radius*width_ratio],[vol_r1_2[num],vol_r1_2[num]])
    vol_r1_line.set_data([t0,t_end],[vol_r1_2[num],vol_r1_2[num]])

    tank_22.set_data([0,0],[-80,volume_2[num]-80])
    tnk_2.set_data(t[0:num],volume_2[0:num])
    vol_r2.set_data([-radius*width_ratio,radius*width_ratio],[vol_r2_2[num],vol_r2_2[num]])
    vol_r2_line.set_data([t0,t_end],[vol_r2_2[num],vol_r2_2[num]])

    tank_32.set_data([0,0],[-80,volume_3[num]-80])
    tnk_3.set_data(t[0:num],volume_3[0:num])
    vol_r3.set_data([-radius*width_ratio,radius*width_ratio],[vol_r3_2[num],vol_r3_2[num]])
    vol_r3_line.set_data([t0,t_end],[vol_r3_2[num],vol_r3_2[num]])


    return  vol_r1,tank_12,vol_r1_line,tnk_1,\
            vol_r2,tank_22,vol_r2_line,tnk_2,\
            vol_r3,tank_32,vol_r3_line,tnk_3,\

    # return vol_r1,tank_12,tnk_1,vol_r1_line\

# Set up your figure properties20
fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
gs=gridspec.GridSpec(2,3)

# Create object for Tank1
ax0=fig.add_subplot(gs[0,0],facecolor=(0.9,0.9,0.9))
vol_r1,=ax0.plot([],[],'r',linewidth=2)
tank_12,=ax0.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dVol,dVol))
plt.ylabel('tank volume [m^3]')
plt.title('Tank 1')

# Create object for Tank2
ax1=fig.add_subplot(gs[0,1],facecolor=(0.9,0.9,0.9))
vol_r2,=ax1.plot([],[],'r',linewidth=2)
tank_22,=ax1.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dVol,dVol))
plt.title('Tank 2')


# Create object for Tank3
ax2=fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
vol_r3,=ax2.plot([],[],'r',linewidth=2)
tank_32,=ax2.plot([],[],'royalblue',linewidth=260,zorder=0)
plt.xlim(-radius*width_ratio,radius*width_ratio)
plt.ylim(bottom,final_volume)
plt.xticks(np.arange(-radius,radius+1,radius))
plt.yticks(np.arange(bottom,final_volume+dVol,dVol))
plt.title('Tank 3')

# Create volume function
ax3=fig.add_subplot(gs[1,:], facecolor=(0.9,0.9,0.9))
vol_r1_line,=ax3.plot([],[],'r',linewidth=2)
vol_r2_line,=ax3.plot([],[],'r',linewidth=2)
vol_r3_line,=ax3.plot([],[],'r',linewidth=2)
tnk_1,=ax3.plot([],[],'blue',linewidth=4,label='Tank 1')
tnk_2,=ax3.plot([],[],'green',linewidth=4,label='Tank 2')
tnk_3,=ax3.plot([],[],'red',linewidth=4,label='Tank 3')
plt.xlim(0,t_end)
plt.ylim(0,final_volume)
plt.ylabel('tank volume [m^3]')
plt.grid(True)
plt.legend(loc='upper right',fontsize='small')

plane_ani=animation.FuncAnimation(fig,update_plot,
    frames=frame_amount,interval=20,repeat=True,blit=True)
plt.show()
