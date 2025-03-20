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

## Technical Implementation

### Core Algorithm: Reynolds' Boids

The flocking simulation is based on Craig Reynolds' Boids algorithm, implemented as a set of steering behaviors. Each boid (bird-oid object) follows three core rules:

#### 1. Separation

Separation is a steering behavior that prevents crowding by making boids steer away from nearby neighbors. The mathematical formula for the separation force is:

```
steer = (0, 0, 0)
count = 0

for each neighbor in neighbors:
    if distance(boid, neighbor) < separationRadius:
        diff = boid.position - neighbor.position
        diff.normalize()
        diff /= distance(boid, neighbor)  // Weight by distance (closer = stronger)
        steer += diff
        count++

if count > 0:
    steer /= count

if steer.length() > 0:
    steer.normalize()
    steer *= maxSpeed
    steer -= boid.velocity
    steer.limit(maxForce)

return steer * separationWeight
```

**Code Implementation:**
```cpp
ofVec3f Boid::separate(vector<Boid*> neighbors) {
    float desiredSeparation = separationRadius;
    ofVec3f steer(0, 0, 0);
    int count = 0;
    
    // Check each neighbor
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Calculate distance from current boid to neighbor
        float d = position.distance(other->position);
        
        // If within separation radius
        if ((d > 0) && (d < desiredSeparation)) {
            // Calculate vector pointing away from neighbor
            ofVec3f diff = position - other->position;
            diff.normalize();
            
            // Weight by distance (closer = stronger)
            diff /= d;
            
            // Add to steering vector
            steer += diff;
            count++;
        }
    }
    
    // Average of steering vectors
    if (count > 0) {
        steer /= count;
    }
    
    // Implement Reynolds: Steering = Desired - Velocity
    if (steer.length() > 0) {
        steer.normalize();
        steer *= maxSpeed;
        steer -= velocity;
        steer.limit(maxForce);
    }
    
    return steer;
}
```

#### 2. Alignment

Alignment is a steering behavior that causes boids to steer towards the average heading of neighboring boids. The mathematical formula for the alignment force is:

```
sum = (0, 0, 0)
count = 0

for each neighbor in neighbors:
    sum += neighbor.velocity
    count++

if count > 0:
    sum /= count             // Average velocity
    sum.normalize()
    sum *= maxSpeed
    steer = sum - boid.velocity
    steer.limit(maxForce)
    return steer
else:
    return (0, 0, 0)
```

**Code Implementation:**
```cpp
ofVec3f Boid::align(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum all velocities and count number of neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        sum += other->velocity;
        count++;
    }
    
    if (count > 0) {
        // Calculate average velocity
        sum /= count;
        
        // Implement Reynolds: Steering = Desired - Velocity
        sum.normalize();
        sum *= maxSpeed;
        ofVec3f steer = sum - velocity;
        steer.limit(maxForce);
        return steer;
    }
    else {
        // If no neighbors, maintain current velocity
        return ofVec3f(0, 0, 0);
    }
}
```

#### 3. Cohesion

Cohesion is a behavior that causes boids to steer towards the average position of neighboring boids. The mathematical formula for the cohesion force is:

```
sum = (0, 0, 0)
count = 0

for each neighbor in neighbors:
    sum += neighbor.position
    count++

if count > 0:
    sum /= count                 // Calculate center of mass
    return seek(sum)             // Steer towards center
else:
    return (0, 0, 0)
```

**Code Implementation:**
```cpp
ofVec3f Boid::cohere(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum all positions and count number of neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        sum += other->position;
        count++;
    }
    
    if (count > 0) {
        // Calculate average position (center of mass)
        sum /= count;
        
        // Create vector pointing from boid towards center of mass
        return seek(sum);
    }
    else {
        // If no neighbors, maintain current direction
        return ofVec3f(0, 0, 0);
    }
}
```

#### Seek Behavior

The `seek` function is a utility method used by other behaviors (such as cohesion) to generate steering forces towards a target point:

```cpp
ofVec3f Boid::seek(ofVec3f target) {
    // Create desired velocity
    ofVec3f desired = target - position;
    
    // Scale to maximum speed
    float d = desired.length();
    desired.normalize();
    
    // If we're close to target, slow down (arrival behavior)
    if (d < 4.0) {
        float m = ofMap(d, 0, 4.0, 0, maxSpeed);
        desired *= m;
    }
    else {
        desired *= maxSpeed;
    }
    
    // Reynolds: Steering = Desired - Velocity
    ofVec3f steer = desired - velocity;
    steer.limit(maxForce);
    
    return steer;
}
```

### Spatial Partitioning

For efficient neighbor detection (which is a performance bottleneck in flocking simulations), the system implements a grid-based spatial partitioning algorithm:

```cpp
// Initialize 3D grid in FlockSystem constructor
gridResolution = 10;
grid.resize(gridResolution);
for (int x = 0; x < gridResolution; x++) {
    grid[x].resize(gridResolution);
    for (int y = 0; y < gridResolution; y++) {
        grid[x][y].resize(gridResolution);
    }
}

// Update grid cell assignments
void FlockSystem::updateSpatialGrid() {
    // Clear all cells
    for (int x = 0; x < gridResolution; x++) {
        for (int y = 0; y < gridResolution; y++) {
            for (int z = 0; z < gridResolution; z++) {
                grid[x][y][z].boids.clear();
            }
        }
    }
    
    // Add boids to cells
    for (int i = 0; i < boids.size(); i++) {
        ofVec3f gridPos = worldToGrid(boids[i]->position);
        int gx = gridPos.x;
        int gy = gridPos.y;
        int gz = gridPos.z;
        
        // Make sure coordinates are within grid bounds
        if (gx >= 0 && gx < gridResolution &&
            gy >= 0 && gy < gridResolution &&
            gz >= 0 && gz < gridResolution) {
            grid[gx][gy][gz].boids.push_back(boids[i]);
        }
    }
}

// Efficient neighbor searching
vector<Boid*> FlockSystem::getNeighbors(Boid* boid, float radius) {
    vector<Boid*> neighbors;
    
    // Convert boid position to grid coordinates
    ofVec3f gridPos = worldToGrid(boid->position);
    int gx = gridPos.x;
    int gy = gridPos.y;
    int gz = gridPos.z;
    
    // Calculate cell search radius based on neighbor radius
    int cellRadius = ceil(radius / cellSize);
    
    // Search neighboring cells
    for (int x = max(0, gx - cellRadius); x <= min(gridResolution - 1, gx + cellRadius); x++) {
        for (int y = max(0, gy - cellRadius); y <= min(gridResolution - 1, gy + cellRadius); y++) {
            for (int z = max(0, gz - cellRadius); z <= min(gridResolution - 1, gz + cellRadius); z++) {
                Cell* cell = getCell(x, y, z);
                if (cell) {
                    // Check all boids in this cell
                    for (int i = 0; i < cell->boids.size(); i++) {
                        Boid* other = cell->boids[i];
                        
                        // Don't add self to neighbors
                        if (other != boid) {
                            // Check actual distance
                            float distance = boid->position.distance(other->position);
                            if (distance < radius) {
                                neighbors.push_back(other);
                            }
                        }
                    }
                }
            }
        }
    }
    
    return neighbors;
}
```

This grid-based approach reduces the computational complexity of neighbor finding from O(n²) to O(n) by only checking boids in nearby grid cells rather than all boids in the simulation.

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