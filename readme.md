# n-body simulator
A simulator to study the n-body problem and how extremely minor changes in intial conditions propagate into massive changes.
 Also it's really relaxing to watch the orbits.

For orbital simulation, can enable collisions between masses as well.
For instability from initial conditions, enable single start point and set a low delta for slower chaos.

Written to practice my rusty C++ skills, using the GLFW library with OpenGL for a simple renderer.

![image](preview.png)

### Settings in settings.txt:
- Time step value (double)
- Single start point, set to 0 for everywhere, 1 for single point (int)
- Number of bodies generated (int)
- Delta in inital conditions for single start (double)
- Body collisions, 0 for none, 1 for mobile (int)
- Number of primary masses, 0-3 (int)
- Trail alpha exponential decay rate (float)

### Notes:
- Using Gmm/r^2 as the formula, with G reduced in order to scale the simulation down.
- Around the 5000 body mark, the simulation experiences extreme slowdown.
- Can define bodies as immobile.

### Todo Ideas
- Optimization, approximate mean influence
- Optimization, culling range for gravital effect
- using CUDA for fast calculations
- Nicer looking points using an actual shader