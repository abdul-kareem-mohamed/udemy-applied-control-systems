# Modeling and Control of a frictionless Train

The objective of this project is to model and control a frictionless train to catch block falling out in space in random. 

## 1. Mathematical model

Force is the input for the model. Position is the output. 

According to Newton's law,

$$F_{a} = m \frac{dv}{dt}$$

which can be rewritten as,

$$a = \frac{F_{a}}{m}$$

where, a - acceleration 

Now, when acceleration is integrated using trapezoidal rule of integration, we derive velocity:

$$v_{t_{j}} = v_{t_{j-1}} + \left( \frac{a_{t_{j}} + a_{t_{j-1}}}{2} \right) \Delta t$$

and then, integrating velocity, we derive postion:

$$x_{t_{j}} = x_{t_{j-1}} + \int_{t_{j-1}}^{t_{j}} v(t) \, dt \approx x_{t_{j-1}} + \left( \frac{v_{t_{j}} + v_{t_{j-1}}}{2} \right) \Delta t$$

With these equations, we can predict the final position of the train for a particular force.

## 2. PID controller

When, initially a P controller is used, the train could not catch the block but oscillated about the block. 

So, then a D controller is used in combination to reduce the overshoot/oscillation by introducing a resistive force. 

But, as the railway track is inclined, the force introduced by the controller is balanced out by tangential gravity force and thus, a steady state error is always maintained, thus resulting in missing the block everytime. So, to eliminate this steady state error, a Integrated is used. 

$$F_{a} = K_{p}e + K_{d} \frac{de}{dt} + K_{i} \left( \frac{e_{t_{j}} + e_{t_{j-1}}}{2} \right) \Delta t$$

* Proportional Term ($K_{p}e$): Reacts to the current error.
* Derivative Term ($K_{d} \frac{de}{dt}$): Reacts to the rate of change of the error to prevent overshoot.
* Integral Term ($K_{i} \dots \Delta t$): Accumulates past error to eliminate steady-state offset, here using the trapezoidal average of the current and previous error.
