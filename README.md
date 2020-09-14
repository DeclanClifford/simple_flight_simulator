# Simple Flight Simulator
A 3DOF flight simulator based on a 7 horseshoe vortex aerodynamic model

# What is this?
This is a MATLAB based longitudinal flight simulator. The aerodynamic model is a custom made vortex lattice method. 7 horseshoe vortices are
used to model the aircraft's geometry. 4 for the main wing, 1 for the fuselage, 1 for the horizontal stabiliser and 1 for the vertical stabiliser. In effect,
these surfaces are modelled as thin flat plate wings (yes, including the fuselage). 4 horseshoe vortices were chosen for the main wings to allow the effects of 
dihedral, sweep and taper to be modelled. The tailplane is assumed to be all moving.

If you want to understand the theory behind this model, it is highly recommended you take a look at "" before diving into the code. Here you'll find a very
indepth explanation of the vortex lattice method and you can take a look at my extension of the version given by Professor Neil Sandham.

### There are two versions of the flight simulator available:

### The script based simulator
This version works as a MATLAB script can be used to calculate aerodynamic stability
derivatives. It is coupled with a fourth order Runge Kutta time stepping method. **This
is more of a proof of concept model and you would really only be using this to verify stability
derivatives with data. Pilot inputs can be enterred, however they'll only be modelled as single step inputs.**

There are 5 files needed for this version of the model to work:

"navion_details_six_DoF.m" contains all of the simulated aircraft's geometry. The details of the Ryan Navion are from a report by G . L. Teper.

"vfil.m" is requirement for "three_DoF_aircraft_model.m" and "three_DoF_aircraft_simulator.m". It calculates the induction of a vortex filament at a point some perpendicular
distance from it. Please see "" for a detailed description of this function.

"three_DoF_aircraft_model.m" is a function that calculates the aircraft's pitching moment, x body force amd z body force at some instant. This is used later
in a free crude trim calculator.

"three_DoF_aircraft_simulator.m" achieves the same purpose as three_DoF_aircraft_model.m, though it takes in a control vector, u and state vector x. Using these inputs 
xdot is calculated. This version is significant in that it considers the effect of pitch rate on the local velocity at each horseshoe vortex.

"simulator.m" is a rather messy script. It contains a very, **very** crude trim calculator, the aerodynamic stability derivative calculations and the time stepping method for the flight simulator.


### The simulink based simulator
This version is the more sophisticated model. 
