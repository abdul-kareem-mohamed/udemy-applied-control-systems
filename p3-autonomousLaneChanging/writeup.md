
# MPC Controller Design for autonomous lane changing

This project contains a Model Predictive Control (MPC) implementation for autonomous vehicle lane-change maneuvers. Given a constant longitudinal velocity $v_x$, the controller computes the optimal steering angle $\delta$ to transition the vehicle from a current lane to a target lane while avoiding obstacles and ensuring passenger comfort.The controller optimizes the balance between tracking precision (reaching the target $Y$ and $\psi$) and maneuver smoothness (minimizing steering rate).

<img width="884" height="434" alt="image" src="https://github.com/user-attachments/assets/3b2ccd5b-54f7-4edd-8c86-4107650f310e" />
Fig 1: general overview of the system


<img width="1020" height="406" alt="image" src="https://github.com/user-attachments/assets/5b07ced6-0347-46d5-863f-133060d38e13" />
Fig 2: control loop

## Mathematical Model

We employ a Kinematic Bicycle Model, which is highly effective for highway speeds where lateral acceleration remains within moderate limits. 

<img width="1022" height="245" alt="image" src="https://github.com/user-attachments/assets/706ad1c1-6fc9-41be-8b1b-a6dfb4c6dfb1" />


consider two frames - a global inertial frame and a body frame fixed to the system. let's consider the forces applied on lateral direction only for lane change, as longitudinal velocity remains constant. 
<img width="1061" height="805" alt="image" src="https://github.com/user-attachments/assets/52e290e7-7e0a-4981-9a57-838750010bcb" />

