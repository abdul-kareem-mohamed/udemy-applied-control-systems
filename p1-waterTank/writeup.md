# Modeling and Control of Water Tank Level

The objective of this project is to model and design a P-controller to fill/drain the water in a tank to achieve the desired water level.

## 1. Mathematical model

<img width="685" height="366" alt="image" src="https://github.com/user-attachments/assets/ebff9076-a3d3-4500-8132-75af25b5e92d" />

The conservation of mass for the control volume of the tank is given by:

$$
\frac{dm_{tank}}{dt} = \dot{m}_{in} - \dot{m}_{out}
$$

### 1. Mass-Volume Relationship
Since mass $m$ is the product of density $\rho$ and volume $V$:

$$
m = \rho V
$$

Assuming an incompressible fluid (constant $\rho$), we substitute this into the balance equation:

$$
\frac{d(\rho V)}{dt} = \dot{m}_{in} - \dot{m}_{out}
$$

$$
\rho \frac{dV}{dt} = \dot{m}_{in} - \dot{m}_{out}
$$

### 2. State Equation for Volume
Rearranging to solve for the rate of change of volume:

$$
\frac{dV}{dt} = \frac{1}{\rho} \left( \dot{m}_{in} - \dot{m}_{out} \right)
$$

using trapezoidal rule of integration, we find,

$$
V_{n+1} = V_n + \frac{\Delta t}{2\rho} \left( \Delta\dot{m}_n + \Delta\dot{m}_{n+1} \right)
$$

## 2. P-Controller

This mathematical model is implemented in python and P controller is used to control the mass flow rate, which is the manipulated variable.

The input mass flow rate $\dot{m}_{in}$ is determined by a proportional controller, where the control action is proportional to the error signal:

$$
\dot{m}_{in} = K_p \cdot e(t)
$$

Where:
* $K_p$ is the **Proportional Gain**.
* $e(t)$ is the **Error Signal**, defined as the difference between the setpoint volume ($V_{set}$) and the measured volume ($V_{act}$):

$$
e(t) = V_{set} - V_{act}
$$

Our simulation will take in the manipulated $\dot{m}_{in}$ value and gives information on how it affects the volume of the plant. 
