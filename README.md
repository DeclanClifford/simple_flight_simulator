# Simple Flight Simulator
A 3DOF flight simulator based on a 7 horseshoe vortex aerodynamic model

# What is this?
This is a MATLAB based longitudinal flight simulator. The aerodynamic model is a custom-made vortex lattice method. 7 horseshoe vortices are
used to model the aircraft's geometry. 4 for the main wing, 1 for the fuselage, 1 for the horizontal stabiliser and 1 for the vertical stabiliser. In effect,
these surfaces are modelled as thin flat plate wings (yes, including the fuselage). 4 horseshoe vortices were chosen for the main wings to allow the effects of 
dihedral, sweep and taper to be modelled. The tailplane is assumed to be all moving.

The model has demonstrated good agreement with aerodynamic stability derivatives of several aircraft. For example, most longitudinal stability derivatives are within 15% for the Ryan Navion, and Convair 880M. Differences could be attributed to many things, the most significant reason would be of course the simplicity of the model. The wing model is very reasonable, however the tailplane and fuselage are admittedly very basic models. Furthermore, this model is restricted to low speed aerodynamics (does not consider compressibility) and does not model stall. For the best set of results I would recommend you stay within plus/minus 15 degrees **angle of attack** (not pitch angle!).  

In the future I will upload a more comprehensive analysis and provide a comparison between this model and Digital DATCOM. However I can say off of some very prelimnary tests I have done that Digital DATCOM yields very similar results.

# Theory
If you want to understand the theory behind this model, it is highly recommended you take a look at [simple_vortex_lattice_method](https://github.com/DeclanClifford/simple_vortex_lattice_method) before diving into the code. Here you'll find a very
indepth explanation of the vortex lattice method and you can take a look at my extension of the vortex lattice method by Professor Neil Sandham.

# Why did you do this?
This is a project I took on to keep myself busy before starting my final year of Aeronautics and Astronautics with Air Vehicle Systems and Design at the University of Southampton. The objectives of this project were

1). To keep myself busy

2). To overcome the deep sense of regret that despite being on an aerospace engineering degree I have not once had the chance to pursue my passion of designing a flight simulator

3). To prepare myself for my final year project which to my delight will involve a large amount of flight simulation

# The simulators

There are two versions of the flight simulator available. There is the pure script based simulator and the Simulink based simulator. The aerodynamic model does not differ between these versions.

## Script based simulator
This version works as a MATLAB script can be used to calculate aerodynamic stability
derivatives. It is coupled with a fourth order Runge Kutta time stepping method. **This
is more of a proof of concept model and you would really only be using this to verify stability
derivatives with data. Pilot inputs can be enterred, however they'll only be modelled as single step inputs.**

There are 5 files needed for this version of the model to work. They can be split into two groups; the scripts you should touch, i.e. the scripts you will most likely want to use and the scripts you should not touch, i.e. the scripts that are required for the scripts you should touch to work.

### The scripts you should touch
[navion_details_six_DoF.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/navion_details_six_DoF.m) contains all of the simulated aircraft's geometry. The details of the Ryan Navion are from a report by G . L. Teper.

[simulator.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/simulator.m) is a rather messy script. It contains a very, **very** crude trim calculator, the aerodynamic stability derivative calculations and the time stepping method for the flight simulator. You can use this script to find a trim point 

### The scripts you should not touch
[vfil.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/vfil.m) is requirement for "three_DoF_aircraft_model.m" and "three_DoF_aircraft_simulator.m". It calculates the induction of a vortex filament at a point some perpendicular
distance from it. Please see "" for a detailed description of this function.

[three_DoF_aircraft_model.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/three_DoF_aircraft_model.m) is a function that calculates the aircraft's pitching moment, x body force amd z body force at some instant. This is used later
in a very crude trim calculator.

[three_DoF_aircraft_simulator.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/three_DoF_aircraft_simulator.m) achieves the same purpose as three_DoF_aircraft_model.m, though it takes in a control vector, u and state vector x. Using these inputs 
xdot is calculated. This version is significant in that it considers the effect of pitch rate on the local velocity at each horseshoe vortex.

**Again I cannot stress enough that you'd only want to use this model to compare aerodynamic stability derivatives or if you don't have Simulink installed.**

## Simulink based simulator
This version is the more sophisticated model. The model works in real time and is coupled 
to FlightGear which provides visualisation of the aircraft (Ryan Navion) in flight . It's important to note
that FlightGear is only used for visualisation. The aerodynamics are calculated in a separate block. 

The model also contains a simple autopilot system where the aircraft can be set to reach a certain altitude at a specific climb rate. The autopilot will trim for steady level flight at the target altitude.

**You must run the** [initialise_constants] script before using the Simulink model.
**Make sure vfil.m is in the same directory as the SImulink file**

### System
The full system block diagram is shown in the image below. You'll notice four blocks. `Command`, `Autopilot`, `3DOF Aircraft Plant Model` and `FlightGear Visualisation`.

![Figure 1](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/images/system.JPG)

`Command` simply allows the user to specify a target altitude and maximum climb rate. This block is fairly self explanatory.
![Figure 2](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/images/command.JPG)

`Autopilot` has two outputs, the throttle output and the elevator output. Throttle is assumed to be at maximum power. The elevator command is linked to the target altitude specified in `Command` through three controller loops. Each of these controllers has been tuned to give a reasonably, albeit slightly exaggerated response for the Ryan Navion. An actuator is also present to make the elevator response more realistic.
![Figure 3](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/images/autopilot.JPG)

`3 DOF Aircraft Plant Model` is where the important things happen. The output from `Autopilot`, a control vector, is fed into [three_DoF_aircraft_simulator.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/three_DoF_aircraft_simulator.m). Again, to understand this I highly recommend you see [simple_vortex_lattice_method](https://github.com/DeclanClifford/simple_vortex_lattice_method). The state vector output, x, is then used to calculate airspeed and flight path angle. As importantly it is used to calculated the aircraft's position given as PN - Position North, PE - Position East and h - altitude. These are calculated in a very simple script titled `navigation_equation` which is in effect a matrix rotation from the body frame to the earth frame. Given the model is longitudinal only PE is always zero.
![Figure 4](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/images/3dofaircraftplantmodel.JPG)

`FlightGear Visualisation` calculates the **geodetic** latitude and longitude using PN, PE, h. This is performed using an aerospace blockset block `Flat Earth to LLA`. All of this is fed into the `FlightGear Preconfigured 6DoF Animation` block. **This block is commented out by default since I assume most people will not have FlightGear integrated with Simulink in the same way I do. If you want to try it there are many excellent videos on YouTube which explain more clearly than I can here about how to connect FlightGear to Simulink. This also means the model will not run in real time if you have the block disabled.**
![Figure 5](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/images/visualisation.JPG)


