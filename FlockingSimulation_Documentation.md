# 3D Flocking Simulation Documentation

## Overview

This application simulates flocking behavior in 3D, based on Craig Reynolds' "Boids" algorithm. The simulation demonstrates how complex group behaviors can emerge from simple individual rules. The system features customizable parameters, allowing users to experiment with different flocking behaviors and visualize the results in real-time.

## What is Flocking?

Flocking is a collective behavior exhibited by many animal groups such as birds, fish, and insects. The fascinating aspect of flocking behavior is that it emerges from simple, local interactions between individuals, without any central coordination.

Craig Reynolds' 1987 "Boids" model identified three primary rules that generate realistic flocking behavior:

1. **Separation**: Avoid crowding neighbors (short-range repulsion)
2. **Alignment**: Steer towards the average heading of neighbors
3. **Cohesion**: Steer towards the average position of neighbors

These simple rules, when applied to many individual agents, create complex emergent behaviors that resemble natural flocks, schools, and herds.

## Parameters Explained

### Basic Flocking Parameters

- **Boid Count**: The number of agents in the simulation. Higher numbers create more complex interactions but may affect performance.

- **Separation Weight** (0.0 - 5.0): Controls how strongly boids avoid getting too close to each other.
  - Low values: Boids will cluster tightly together, often overlapping.
  - High values: Boids maintain significant distance, creating spread-out formations.

- **Alignment Weight** (0.0 - 5.0): Controls how strongly boids match their neighbors' direction of movement.
  - Low values: Boids move independently with little coordination.
  - High values: Boids move in nearly perfect unison, like a synchronized formation.

- **Cohesion Weight** (0.0 - 5.0): Controls how strongly boids are attracted to the center of their local group.
  - Low values: Boids disperse and form loose, decentralized groups.
  - High values: Boids form tight clusters, working hard to stay together.

- **Neighbor Radius** (1.0 - 10.0): The distance within which other boids are considered "neighbors" for alignment and cohesion.
  - Low values: Boids only respond to very nearby neighbors, creating smaller sub-groups.
  - High values: Boids respond to distant neighbors, creating larger coordinated groups.

- **Separation Radius** (0.5 - 5.0): The distance at which boids start to avoid each other.
  - Low values: Boids get very close before avoiding each other.
  - High values: Boids maintain larger personal space, creating more spread-out formations.

- **Max Speed** (0.5 - 10.0): The maximum velocity boids can travel.
  - Low values: Slow, gentle movement.
  - High values: Fast, energetic movement.

- **Min Speed** (0.1 - 2.0): The minimum velocity boids will maintain.
  - Low values: Boids can slow down significantly or nearly stop.
  - High values: Boids always maintain a brisk pace, never slowing below this threshold.

- **Max Force** (0.1 - 2.0): The maximum steering force boids can apply to change direction.
  - Low values: Gradual, smooth turns.
  - High values: Sharp, rapid turns and quick course corrections.

- **Boundary Force** (0.1 - 5.0): How strongly boids are pushed back when approaching the simulation boundaries.
  - Low values: Boids may briefly exit the visible area before returning.
  - High values: Boids strongly avoid the boundaries, staying well inside.

### Global System Parameters

- **Flock Mode**: Presets that adjust multiple parameters for different types of flocking behavior.
  - **Normal**: Balanced settings for typical flocking.
  - **Scattered**: Higher separation, lower cohesion for dispersed, independent movement.
  - **Tight**: Higher cohesion, lower separation for dense, closely packed flocks.
  - **Predator-Prey**: Some boids act as predators, chasing others that behave as prey.

- **Individualism** (0.0 - 1.0): How much boids deviate from group behavior based on their individual characteristics.
  - Low values: Highly uniform behavior, all boids follow group rules closely.
  - High values: More varied behavior, boids exhibit individual personalities.

- **System Chaos** (0.0 - 1.0): Introduces turbulence and randomness into the environment.
  - Low values: Smooth, predictable movement.
  - High values: Erratic, chaotic movement as if in turbulent conditions.

- **Boid Variability** (0.0 - 1.0): How different boids are from each other in terms of size, color, and energy level.
  - Low values: Boids look and behave very similarly to each other.
  - High values: Boids have distinct appearances and behavioral tendencies.

- **Target Seeking**: Enables boids to follow a target point.
  - When enabled, boids will be attracted to the target in addition to following flocking rules.

- **Target Weight** (0.1 - 2.0): How strongly boids are attracted to the target.
  - Low values: Subtle influence, flocking behavior dominates.
  - High values: Strong attraction, boids aggressively pursue the target.

### Debug Visualization

- **Show Debug**: Master toggle for all debug visualizations.
- **Show Velocities**: Display velocity vectors for each boid.
- **Show Neighborhoods**: Display perception and separation radii for boids.
- **Show Forces**: Display force vectors (separation, alignment, cohesion, etc.).
- **Show Grid**: Display the spatial partitioning grid used for optimization.

### Environment Parameters

- **Background Color**: Change the canvas background color.
- **Reset Simulation**: Clear all boids and create a new flock.
- **Spawn Boids**: Add more boids to the simulation.
- **Spawn Count**: Number of boids to add when spawning.
- **Load Boid Mesh**: Use a custom 3D model for boid visualization.

## Controls

- **h**: Toggle GUI panels
- **f**: Toggle fullscreen
- **c**: Toggle camera controls (click and drag to rotate view)
- **SPACE**: Pause/Play simulation
- **r**: Reset simulation
- **b**: Spawn new boids at the origin
- **t**: Place target at mouse position (converts 2D mouse to 3D point)
- **m**: Toggle moving target (circular movement)
- **d**: Toggle debug visualization
- **1-4**: Change flock mode (Normal, Scattered, Tight, Predator-Prey)

## Understanding Flocking Behavior

### Balance is Key

The most interesting flocking behaviors emerge from a careful balance of separation, alignment, and cohesion. If any one force becomes too dominant, the emergent behavior changes dramatically:

- **Separation-dominant**: Boids spread out evenly and avoid each other, creating dispersed patterns.
- **Alignment-dominant**: Boids move in the same direction but don't necessarily group together.
- **Cohesion-dominant**: Boids form tight clusters with little coordinated movement.

### Emergent Behaviors

Watch for these interesting patterns that can emerge:

1. **Split and Rejoin**: Flocks splitting around obstacles and rejoining on the other side.
2. **Milling**: Circular motion where boids move in a rotating pattern.
3. **Pulsing**: Groups that expand and contract rhythmically.
4. **Lane Formation**: Boids traveling in opposite directions form distinct lanes.
5. **Vacuoles**: Empty spaces that form and persist within dense flocks.

### Parameter Relationships

Some parameters have important relationships with each other:

- **Neighbor Radius vs. Separation Radius**: If the separation radius is too close to the neighbor radius, boids will join a group only to immediately try to separate.
- **Max Force vs. Max Speed**: If max force is too low compared to max speed, boids won't be able to turn quickly enough at high speeds.
- **Individualism vs. System Chaos**: Both add randomness, but individualism creates consistent individual personalities while chaos affects all boids unpredictably.

## Tips for Interesting Simulations

1. **Predator-Prey Mode**: In this mode, boids with high uniqueness act as predators. Try increasing the boid variability to create more distinct predators.

2. **Turbulent Environment**: Combine high system chaos with low max force to simulate boids trying to maintain formation in stormy conditions.

3. **Migration Patterns**: Enable target seeking with a moving target and set a moderate target weight to simulate seasonal migration with some local flocking.

4. **School of Fish**: Use tight mode, low maximum speed, and a dark blue background for a convincing underwater effect.

5. **Starling Murmuration**: Use normal mode with relatively high alignment, medium cohesion, moderate max force, and high boid count to simulate the spectacular aerial displays of starlings.

Enjoy experimenting with the simulation and discovering the beautiful complexity that emerges from these simple rules! 