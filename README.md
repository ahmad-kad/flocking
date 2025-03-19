# 3D Flocking Simulation in OpenFrameworks

This project implements a real-time 3D flocking simulation based on Craig Reynolds' boids algorithm. The simulation demonstrates emergent flocking behavior through the application of simple rules to individual agents.

## Features

- Full 3D flocking behavior with separation, alignment, and cohesion
- Spatial partitioning for efficient neighbor detection
- Interactive controls for adjusting flocking parameters
- Target seeking functionality with movable targets
- Configurable boundaries to contain the flock
- Fluid, visually appealing animation with directional representation
- Support for large numbers of boids with good performance

## Controls

### Keyboard Controls

- `h`: Toggle GUI visibility
- `f`: Toggle fullscreen
- `c`: Toggle camera controls
- `SPACE`: Toggle particle emitter (in particle mode)
- `1`: Enable flocking mode
- `2`: Enable particle emitter mode
- `t`: Place target at mouse position (projected to ground plane)
- `m`: Toggle moving target (follows circular path)

### Camera Controls

- Left Mouse Button + Drag: Rotate camera
- Right Mouse Button + Drag: Zoom camera
- Middle Mouse Button + Drag: Pan camera

## GUI Parameters

### Flocking System

- **Boid Count**: Number of boids in the simulation
- **Separation**: Weight of separation force (avoid crowding)
- **Alignment**: Weight of alignment force (match velocity with neighbors)
- **Cohesion**: Weight of cohesion force (move toward average position of neighbors)
- **Neighbor Radius**: Distance to consider other boids as neighbors
- **Separation Radius**: Radius within which separation force is applied
- **Max Speed**: Maximum speed a boid can travel
- **Max Force**: Maximum steering force that can be applied
- **Boundary Force**: Strength of force keeping boids within bounds
- **Target Seeking**: Enable/disable target following behavior
- **Target Weight**: Strength of the force pulling toward target

### Particle System

- **Initial Velocity**: Starting velocity for particles
- **Lifespan**: How long particles live before disappearing
- **Rate**: How many particles are emitted per second
- **Damping**: How quickly particles slow down
- **Gravity**: Strength of gravity on particles
- **Radius**: Size of particles
- **Turbulence**: Controls random movement of particles

## Building the Project

1. Make sure you have OpenFrameworks installed
2. Clone this repository into your OpenFrameworks apps folder
3. Open the project with your IDE (XCode, Visual Studio, etc.)
4. Build and run the project

## Implementation Details

For technical details about the implementation, see the [Implementation Notes](IMPLEMENTATION_NOTES.md) document.

## Credits

This project was developed based on Craig Reynolds' boids algorithm and implements the three classic flocking rules:
- Separation: steer to avoid crowding local flockmates
- Alignment: steer toward the average heading of local flockmates
- Cohesion: steer to move toward the average position of local flockmates

Built with OpenFrameworks. 