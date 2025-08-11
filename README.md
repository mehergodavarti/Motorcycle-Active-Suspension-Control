# Motorcycle-Active-Suspension-Control
1. Introduction:

This project focuses on the design, modeling, and simulation of an active suspension system for a motorcycle, with a particular emphasis on demonstrating the application of control theory principles. While significant work exists on both active suspension systems and motorcycle dynamics, this project uniquely integrates both to provide a comprehensive understanding for new engineers, familiarizing them with transfer functions and controller design. Additionally, this project leveraged Artificial Intelligence (AI) for various aspects, including report structuring and general support, aiming to expedite the project's completion.

2. Background:

Active suspension systems are crucial in modern vehicles, allowing real-time dynamic adjustments to handling. While various control algorithms exist, this project primarily utilizes a Model Predictive Control (MPC) due to its ability to handle complex systems with constraints. MPC calculates optimal control actions over a future horizon by predicting system behavior, making it highly effective for adaptive systems like active suspension. Though simpler control schemes like Proportional, Integral, Derivative (PID) control could be explored, the focus on MPC for this project allowed for a more advanced and robust solution

3. Problem Statement:

Motorcycles, operating on two wheels, present unique control challenges. From a suspension perspective, the system is simplified compared to a four-wheeled vehicle as only two wheels need to be modeled. A key challenge lies in balancing rider comfort with optimal bike performance. Active suspension, with its configurable settings, offers a robust solution to address this balance.


4. Objectives:
Primary Objective: To design a comprehensive model of a motorcycle suspension system capable of evaluation under various test conditions.
Secondary Objective: To incorporate AI tools into the project workflow to expedite completion.
5. Methodology:

The project was divided into three primary tasks:

5.1. System Modeling:

The initial phase involved modeling the motorcycle suspension system. This process began by modeling each wheel individually before integrating them into a complete system model, as illustrated in the original proposal's Figure 1 (Motorcycle Suspension Model). The dynamic equations for the system were derived and applied, considering the sprung and unsprung masses, spring constants, and damping constants.
Key Equations (from BMW 1000RR Parameters):
Front Wheel Vertical Motion: m_f * ddz_wf = kf * (z_b + l_f*theta - z_wf) + cf * (dz_b + l_f*dtheta - dz_wf) - u_f
Rear Wheel Vertical Motion: m_r * ddz_wr = kr * (z_b - l_r*theta - z_wr) + cr * (dz_b - l_r*dtheta - dz_wr) - u_r
Car Body Vertical Motion: m_b * ddz_b = -[kf * (z_b + l_f*theta - z_wf) + cf * (dz_b + l_f*dtheta - dz_wf) + kr * (z_b - l_r*theta - z_wr) + cr * (dz_b - l_r*dtheta - dz_wr)]
Car Body Pitch (Tilting Forward/Backward): Ib * ddtheta = -kf*l_f * (z_b + l_f*theta - z_wf) - cf*l_f * (dz_b + l_f*dtheta - dz_wf) + kr*l_r * (z_b - l_r*theta - z_wr) + cr*l_r * (dz_b - l_r*dtheta - dz_wr)

Parameters (BMW S 1000 RR - approx. 2020-2024 model years):
Mass & Inertia:
m_b = 160.5 kg (Mass of the bike body - sprung mass)
I_m = 60 kg*m^2 (Moment of inertia for bike pitch)
m_wf = 16.5 kg (Mass of the front wheel assembly - unsprung)
m_wr = 20.0 kg (Mass of the rear wheel assembly - unsprung)
Geometry:
L_f = 0.692 m (Horizontal distance from CG to front axle)
L_r = 0.765 m (Horizontal distance from CG to rear axle)
Front Suspension & Tire:
k_sf = 21000 N/m (Effective spring constant of the front suspension)
b_sf = 1330 Ns/m (Effective damping constant of the front suspension)
k_tf = 190000 N/m (Vertical stiffness of the front tire)
Rear Suspension & Tire:
k_sr = 29300 N/m (Effective spring constant of the rear suspension)
b_sr = 1500 Ns/m (Effective damping constant of the rear suspension)
k_tr = 210000 N/m (Vertical stiffness of the rear tire)
Note: Tire spring and damping constants were acknowledged as future scope in the initial notes, but vertical stiffness of the tires (k_tf, k_tr) was included in the specific BMW S 1000 RR parameters.
Key Considerations for the Project
Gravity and Equilibrium: If all displacements (z_wf, z_wr, z_b) are defined from the static equilibrium position (where spring forces already balance gravity), then gravity does not need to be included in the dynamic equation.
Tire Dynamics (Future Scope):
This project did not include the tire spring constant or tire damping constant.
Reasoning: Without these parameters, the model assumes a direct connection between the wheel mass and the road, which is less realistic.
Improvements: Including these parameters adds a spring-damper element between the wheel and the road, significantly enhancing simulation accuracy for ride and handling.
Parameter Definitions:
Tire spring constant: Models how much the tire compresses under load (tire stiffness).
Tire damping constant: Models how the tire dissipates energy due to deformation.
The initial phase involved modeling the motorcycle suspension system. This process began by modeling each wheel individually before integrating them into a complete system model, as illustrated in the original proposal's Figure 1 (Motorcycle Suspension Model). The dynamic equations for the system were derived and applied, considering the sprung and unsprung masses, spring constants, and damping constants.

5.2. Model Predictive Controller

Upon completion of the system model, a Model Predictive Controller (MPC) was designed and tuned. The initial tuning focused on making system stable and then added diverse road conditions, including single step impulses and random road profiles. Further optimization of the controller was achieved during the simulation phase.


For active suspension, the MPC algorithm was implemented to predict the system's future behavior and determine optimal actuator forces to minimize ride discomfort and maximize road holding, subject to physical limitations of the suspension system. This predictive capability allowed the system to proactively adjust to varying road input severity.


F = -Kp*(z - z_ref) - Kv*(ẋ - ẋ_ref)

Where Kp and Kv are adjusted adaptively based on road input severity.

5.3. MATLAB Simulation:

MATLAB is utilized to develop a flexible simulation environment, enabling the execution of multiple scenarios. During simulations, system parameters were adjusted to ensure optimal performance. AI tools were also leveraged to assist in code generation.
Road Conditions to Simulate:
Smooth: low amplitude, low frequency (0.01 * sin(2π * 0.5 * t))
Rough: high frequency, medium amplitude (0.03 * sin(2π * 5 * t))
Bumpy: series of spaced bumps (series of 0.05 m steps (pulse train))
Extreme: ramp large enough to bottom out the suspension (0.1 m step or ramp)
Control Modes per Wheel:
Passive: Only spring and damper.
Active: Spring and damper, augmented by a controlled actuator.

A Boolean parameter (e.g., FrontActive = true; RearActive = false;) was used to toggle between modes for each wheel. A Switch block was implemented to connect the appropriate system (passive or active).
Riding Modes:

Two system-wide riding modes were implemented:
Normal Mode: Softer damping/spring, lower actuator force limits.
Example parameters: ks = 10000; cs = 800; Fmax = 250;
Sport Mode: Stiffer suspension, higher actuator force limits, faster response.
Example parameters: ks = 15000; cs = 1200; Fmax = 500;

These parameters were applied conditionally based on a RideMode variable (e.g., 'Sport' or 'Normal').
6. Expected Outcomes (Achieved):

This project successfully yielded a functional and optimized controller for a motorcycle active suspension system. Its utility was demonstrated across a variety of simulated scenarios, showcasing its ability to adapt to diverse road conditions and rider preferences. The integration of AI tools also proved beneficial in accelerating the development process.

(z_b - l_r*theta - z_wr) + cr*l_r * (dz_b - l_r*dtheta - dz_wr)
This equation describes the angular acceleration (pitch, ddtheta) of the bike body around its center of gravity. The moments are generated by the forces from the front and rear suspensions, acting at their respective distances from the center of gravity (l_f and l_r).
Parameters (BMW S 1000 RR - approximate 2020-2024 model years):
These parameters were meticulously sourced from available data for the BMW S 1000 RR, providing realistic and representative values for the simulation:
Mass & Inertia:
m_b = 160.5 kg (Mass of the bike body - representing the sprung mass, excluding wheels and riders)
I_m = 60 kg*m^2 (Moment of inertia for bike pitch, crucial for rotational dynamics)
m_wf = 16.5 kg (Mass of the front wheel assembly - unsprung mass)
m_wr = 20.0 kg (Mass of the rear wheel assembly - unsprung mass)
Geometry:
L_f = 0.692 m (Horizontal distance from the bike's center of gravity to the front axle)
L_r = 0.765 m (Horizontal distance from the bike's center of gravity to the rear axle)
Front Suspension & Tire:
k_sf = 21000 N/m (Effective spring constant of the front suspension, representing its stiffness)
b_sf = 1330 Ns/m (Effective damping constant of the front suspension, representing its resistance to motion

7. AI Tools and Contributions:

We used "Manus (AI)" to help us with this project, making things faster and more efficient. It was a key part of our work, from research to writing the final report.

Here's how Manus (AI) helped:
Report Writing: It helped us organize and write the project report, making sure it flowed well and was easy to understand. It also gave suggestions to make the language clearer.
Understanding Suspension Physics and Equations: Manus (AI) clarified how motorcycle suspension systems work and helped us understand and correctly write the complex equations of motion for the bike's movement.
Finding BMW S 1000 RR Parameters: It helped us find and check realistic numbers for the BMW S 1000 RR's suspension, like masses, inertia, and spring/damping values. This made our simulations more accurate.
MATLAB/Simulink Code Help: It assisted with writing MATLAB code and setting up Simulink models. When we had errors, it provided solutions, which saved a lot of time.
Controller Tuning: It offered ideas for which control system to use, help us understand MPC, Eigen vector and values, also providing  the best values for "Normal" and "Sport" riding modes.
General Support: It was a quick resource for understanding difficult engineering ideas and finding solutions to design problems.
Using Manus (AI) helped us manage our time, improve our report, and focus on the main engineering challenges. It made the project faster and also helped us learn more.

8. References:
https://manus.im/app 
Automotive Suspension - MATLAB & Simulink
Robust Control of Active Suspension - MATLAB &amp; Simulink
Quarter Car Simulation Using Matlab
Derive Transfer Function of Suspension System of a Car - Student Projects

