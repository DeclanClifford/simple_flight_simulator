# Simple Flight Simulator
A 3DOF flight simulator based on a 7 horseshoe vortex aerodynamic model

# What is this?
This is a MATLAB based longitudinal flight simulator. The aerodynamic model is a custom made vortex lattice method. 7 horseshoe vortices are
used to model the aircraft's geometry. 4 for the main wing, 1 for the fuselage, 1 for the horizontal stabiliser and 1 for the vertical stabiliser. In effect,
these surfaces are modelled as thin flat plate wings (yes, including the fuselage). 4 horseshoe vortices were chosen for the main wings to allow the effects of 
dihedral, sweep and taper to be modelled. The tailplane is assumed to be all moving.



###There are two versions available:

### The script based simulator
This version works as a MATLAB script can be used to calculate aerodynamic stability
derivatives. It is coupled with a fourth order Runge Kutta time stepping method. This
is more of a proof of concept model and you would really only be using this to verify stability
derivatives with data. Pilot inputs can be enterred, however they'll only be modelled as single step inputs.

### The simulink based simulator
This version is the more sophisticated model. 
