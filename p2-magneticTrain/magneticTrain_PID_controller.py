import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np
import random


#INPUTS THAT I USE FOR THE SIMULATION
trials = 4
incl_angle = np.pi/6*1 #keeping the angle between 0 and pi/6
g = 10 #gravity
mass_cart = 100 #[kg]

#Tune the constants
K_p = 300
K_d = 300
K_i = 50

trials_global = trials #trials get decremented for simulation, so using trials_global to store its initial value for animation

# Generate random x-positions for a falling cube
def set_x_ref(incl_angle):
    rand_h=random.uniform(0,120)

    # start_range - start_tolerance + rail_height + train_thickness/2
    # end_range - end_tolerance +  rail_height + train_thickness/2
    rand_v=random.uniform(20+120*np.tan(incl_angle)+6.5,40+120*np.tan(incl_angle)+6.5)
    return rand_h,rand_v

# time step
dt = 0.02
t_end = 5
t0 = 0
t = np.arange(t0, t_end+dt, dt) #t_end+dt - so that it creates an array till 5, not 4.98

# train
disp_rail = np.zeros((trials, len(t))) #  trials in row, time_step in column
v_rail = np.zeros((trials, len(t))) 
a_rail = np.zeros((trials, len(t)))
pos_x_train = np.zeros((trials, len(t)))
pos_y_train = np.zeros((trials, len(t)))
e = np.zeros((trials, len(t))) #error
e_dot = np.zeros((trials, len(t))) #error derivative
e_int = np.zeros((trials, len(t))) #error integral

# cube
pos_x_cube = np.zeros((trials, len(t)))
pos_y_cube = np.zeros((trials, len(t)))

# initial conditions
ini_rail_pos_x = 120
ini_rail_pos_y = 120*np.tan(incl_angle)+6.5 # starts at the farthest end of the track, 6.5 included to align the train to the center
ini_displ_rail=(ini_rail_pos_x**2+ini_rail_pos_y**2)**(0.5) # pythogoras theorem - disp along the track from the center
ini_vel_rail=0
ini_a_rail=0 

init_pos_x_global=ini_rail_pos_x # Used for determining the dimensions of the animation window.

# forces acting on the body
F_g = mass_cart*g
F_ga_t=F_g*np.sin(incl_angle) # Tangential component of the gravity force

trials_mgn = trials
history = np.ones(trials)
while(trials>0):
    times = trials_mgn - trials
    pos_x_cube_ref, pos_y_cube_ref = set_x_ref(incl_angle)
    pos_x_cube[times] = pos_x_cube_ref
    pos_y_cube[times] = pos_y_cube_ref - g/2*t**2 # (h - 1/2*g*t^2)
    win=False
    delta=1

    #Implement PID for train position
    for i in range(1, len(t)):
        if i==1:
            pos_x_train[times][0] = ini_rail_pos_x
            pos_y_train[times][0]=ini_rail_pos_y
            disp_rail[times][0] = ini_displ_rail
            v_rail[times][0] = ini_vel_rail
            a_rail[times][0] = ini_a_rail

        """
        To calculate the position of train with respect to time, we need to first find the forces
        needed to move the train and then use that force to calculate: first velocity, then position. 

        """

        # Error calculation
        e[times][i-1] = pos_x_cube_ref - pos_x_train[times][i-1] # error for K_p

        if i>1:
            e_dot[times][i-1] = (e[times][i-1]-e[times][i-2])/dt
            e_int[times][i-1]=e_int[times][i-2]+(e[times][i-2]+e[times][i-1])/2*dt # division has higher precedence - trapezoidal rule of int.
        
        if i==len(t)-1:
            # Use the previous calculated values for the last index
            e[times][-1] = e[times][-2]
            e_dot[times][-1] = e_dot[times][-2]
            e_int[times][-1]=e_int[times][-2]

        # Force calculation
        F_a = K_p*e[times][i-1]+K_d*e_dot[times][i-1]+K_i*e_int[times][i-1] # Not a matrix
        F_net = F_a + F_ga_t # Not a matrix

        # Train position
        a_rail[times][i] = F_net/mass_cart
        v_rail[times][i] = v_rail[times][i-1] + (a_rail[times][i]+a_rail[times][i-1])/2*dt # trapezoidal rule of int.
        disp_rail[times][i] = disp_rail[times][i-1]+(v_rail[times][i-1]+v_rail[times][i])/2*dt # trapezoidal rule of int.
        pos_x_train[times][i]=disp_rail[times][i]*np.cos(incl_angle)
        pos_y_train[times][i]=disp_rail[times][i]*np.sin(incl_angle)+6.5

        # Overlap of the sides - condition to move the cube along with the train when caught
        if (pos_x_train[times][i]-5<pos_x_cube[times][i]+3 and pos_x_train[times][i]+5>pos_x_cube[times][i]-3) or win==True: 
            # Check whether the cube is within a threshold to confirm that it was caught
            if (pos_y_train[times][i]+3<pos_y_cube[times][i]-2 and pos_y_train[times][i]+8>pos_y_cube[times][i]+2) or win==True:
                win=True # this condition helps to avoid checking the overlapping again for the next iterations in the trial
                if delta==1:
                    # avoid realignment of centre of the cube with train
                    change=pos_x_train[times][i]-pos_x_cube[times][i]
                    delta=0
                pos_x_cube[times][i]=pos_x_train[times][i]-change
                pos_y_cube[times][i]=pos_y_train[times][i]+5

    # uses the postion, velocity, acceleration of the trial as initial condition for the next one
    ini_displ_rail=disp_rail[times][-1]
    ini_rail_pos_x=pos_x_train[times][-1]+v_rail[times][-1]*np.cos(incl_angle)*dt
    ini_pos_y=pos_y_train[times][-1]+v_rail[times][-1]*np.sin(incl_angle)*dt
    init_vel_rail=v_rail[times][-1]
    init_a_rail=a_rail[times][-1]
    history[times]=delta
    trials=trials-1

############################## ANIMATION #################################
len_t=len(t)
frame_amount=len(t)*trials_global
def update_plot(num):

    platform.set_data([pos_x_train[int(num/len_t)][num-int(num/len_t)*len_t]-3.1,\
    pos_x_train[int(num/len_t)][num-int(num/len_t)*len_t]+3.1],\
    [pos_y_train[int(num/len_t)][num-int(num/len_t)*len_t],\
    pos_y_train[int(num/len_t)][num-int(num/len_t)*len_t]])

    cube.set_data([pos_x_cube[int(num/len_t)][num-int(num/len_t)*len_t]-1,\
    pos_x_cube[int(num/len_t)][num-int(num/len_t)*len_t]+1],\
    [pos_y_cube[int(num/len_t)][num-int(num/len_t)*len_t],\
    pos_y_cube[int(num/len_t)][num-int(num/len_t)*len_t]])

    if trials_mgn*len_t==num+1 and num>0: # All attempts must be successful
        if sum(history)==0:
            success.set_text('Block is caught !!')
        else:
            again.set_text('Try Again !!')

    displ_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        disp_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    v_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        v_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    a_rail_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        a_rail[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_dot_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e_dot[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    e_int_f.set_data(t[0:(num-int(num/len_t)*len_t)],
        e_int[int(num/len_t)][0:(num-int(num/len_t)*len_t)])

    return displ_rail_f,v_rail_f,a_rail_f,e_f,e_dot_f,e_int_f,platform,cube,success,again

fig=plt.figure(figsize=(16,9),dpi=120,facecolor=(0.8,0.8,0.8))
gs=gridspec.GridSpec(4,3)

# Create main window
ax_main=fig.add_subplot(gs[0:3,0:2],facecolor=(0.9,0.9,0.9))
plt.xlim(0,init_pos_x_global)
plt.ylim(0,init_pos_x_global)
plt.xticks(np.arange(0,init_pos_x_global+1,10))
plt.yticks(np.arange(0,init_pos_x_global+1,10))
plt.grid(True)

rail=ax_main.plot([0,init_pos_x_global],[5,init_pos_x_global*np.tan(incl_angle)+5],'k',linewidth=6)
platform,=ax_main.plot([],[],'b',linewidth=18)
cube,=ax_main.plot([],[],'k',linewidth=14)

bbox_props_success=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='g',lw=1.0)
success=ax_main.text(40,60,'',size='20',color='g',bbox=bbox_props_success)

bbox_props_again=dict(boxstyle='square',fc=(0.9,0.9,0.9),ec='r',lw=1.0)
again=ax_main.text(30,60,'',size='20',color='r',bbox=bbox_props_again)

# Plot windows
ax1v=fig.add_subplot(gs[0,2],facecolor=(0.9,0.9,0.9))
displ_rail_f,=ax1v.plot([],[],'-b',linewidth=2,label='displ. on rails [m]')
plt.xlim(t0,t_end)
plt.ylim(np.min(disp_rail)-abs(np.min(disp_rail))*0.1,np.max(disp_rail)+abs(np.max(disp_rail))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax2v=fig.add_subplot(gs[1,2],facecolor=(0.9,0.9,0.9))
v_rail_f,=ax2v.plot([],[],'-b',linewidth=2,label='velocity on rails [m/s]')
plt.xlim(t0,t_end)
plt.ylim(np.min(v_rail)-abs(np.min(v_rail))*0.1,np.max(v_rail)+abs(np.max(v_rail))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax3v=fig.add_subplot(gs[2,2],facecolor=(0.9,0.9,0.9))
a_rail_f,=ax3v.plot([],[],'-b',linewidth=2,label='accel. on rails [m/s^2] = F_net/m_platf.')
plt.xlim(t0,t_end)
plt.ylim(np.min(a_rail)-abs(np.min(a_rail))*0.1,np.max(a_rail)+abs(np.max(a_rail))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax1h=fig.add_subplot(gs[3,0],facecolor=(0.9,0.9,0.9))
e_f,=ax1h.plot([],[],'-b',linewidth=2,label='horizontal error [m]')
plt.xlim(t0,t_end)
plt.ylim(np.min(e)-abs(np.min(e))*0.1,np.max(e)+abs(np.max(e))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax2h=fig.add_subplot(gs[3,1],facecolor=(0.9,0.9,0.9))
e_dot_f,=ax2h.plot([],[],'-b',linewidth=2,label='change of horiz. error [m/s]')
plt.xlim(t0,t_end)
plt.ylim(np.min(e_dot)-abs(np.min(e_dot))*0.1,np.max(e_dot)+abs(np.max(e_dot))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

ax3h=fig.add_subplot(gs[3,2],facecolor=(0.9,0.9,0.9))
e_int_f,=ax3h.plot([],[],'-b',linewidth=2,label='sum of horiz. error [m*s]')
plt.xlim(t0,t_end)
plt.ylim(np.min(e_int)-abs(np.min(e_int))*0.1,np.max(e_int)+abs(np.max(e_int))*0.1)
plt.grid(True)
plt.legend(loc='lower left',fontsize='small')

pid_ani=animation.FuncAnimation(fig,update_plot,
    frames=frame_amount,interval=20,repeat=False,blit=True)
plt.show()
