# Simple Flight Simulator
A 3DOF flight simulator based on a 7 horseshoe vortex aerodynamic model

# What is this?
This is a MATLAB based longitudinal flight simulator. The aerodynamic model is a custom made vortex lattice method. 7 horseshoe vortices are
used to model the aircraft's geometry. 4 for the main wing, 1 for the fuselage, 1 for the horizontal stabiliser and 1 for the vertical stabiliser. In effect,
these surfaces are modelled as thin flat plate wings (yes, including the fuselage). 4 horseshoe vortices were chosen for the main wings to allow the effects of 
dihedral, sweep and taper to be modelled. The tailplane is assumed to be all moving.

The model gives generally good agreement (within 15%) with aerodynamic stability derivatives as given by G. L. Teper for a Ryan Navion

# Theory
If you want to understand the theory behind this model, it is highly recommended you take a look at "" before diving into the code. Here you'll find a very
indepth explanation of the vortex lattice method and you can take a look at my extension of the version given by Professor Neil Sandham.

# Why did you do this?
This is a project I took on to keep myself busy before starting my final year of Aeronautics and Astronautics with Air Vehicle Systems and Design at the University of Southampton. The objectives of this project were

1). To keep myself busy

2). To overcome the deep sense of regret that despite being on an aerospace engineering degree I have not once had the chance to pursue my passion of designing a flight simulator

3). To prepare myself for my final year project which to my delight will involve a large amount of flight simulation

# The simulators

There are two versions of the flight simulator available:

### Script based simulator
This version works as a MATLAB script can be used to calculate aerodynamic stability
derivatives. It is coupled with a fourth order Runge Kutta time stepping method. **This
is more of a proof of concept model and you would really only be using this to verify stability
derivatives with data. Pilot inputs can be enterred, however they'll only be modelled as single step inputs.**

There are 5 files needed for this version of the model to work:

[navion_details_six_DoF.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/navion_details_six_DoF.m) contains all of the simulated aircraft's geometry. The details of the Ryan Navion are from a report by G . L. Teper.

[vfil.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/vfil.m) is requirement for "three_DoF_aircraft_model.m" and "three_DoF_aircraft_simulator.m". It calculates the induction of a vortex filament at a point some perpendicular
distance from it. Please see "" for a detailed description of this function.

[three_DoF_aircraft_model.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/three_DoF_aircraft_model.m) is a function that calculates the aircraft's pitching moment, x body force amd z body force at some instant. This is used later
in a very crude trim calculator.

[three_DoF_aircraft_simulator.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/three_DoF_aircraft_simulator.m) achieves the same purpose as three_DoF_aircraft_model.m, though it takes in a control vector, u and state vector x. Using these inputs 
xdot is calculated. This version is significant in that it considers the effect of pitch rate on the local velocity at each horseshoe vortex.

[simulator.m](https://github.com/DeclanClifford/simple_flight_simulator/blob/master/script%20version/simulator.m) is a rather messy script. It contains a very, **very** crude trim calculator, the aerodynamic stability derivative calculations and the time stepping method for the flight simulator. You can use this script to find a trim point 

** Again I cannot stress enough that you'd only want to use this model to compare aerodynamic stability derivatives or if you don't have Simulink installed.


### Simulink based simulator
This version is the more sophisticated model. The model works in real time and is coupled 
to FlightGear which provides visualisation of the aircraft (Ryan Navion) in flight . It's important to note
that FlightGear is only used for visualisation. The aerodynamics are calculated in a separate block. The model
also contains a simple autopilot system where the aircraft can be set to reach a certain altitude at a specific climb rate.

### System











