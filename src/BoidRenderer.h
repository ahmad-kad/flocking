#pragma once

#include "ofMain.h"

// Forward declare Boid to avoid circular dependency
class Boid; 

class BoidRenderer {
public:
    // Static methods as they don't need specific renderer state, just the boid's state
    static void draw(const Boid& boid, ofMesh* customMesh = nullptr);
    static void drawDebug(const Boid& boid, bool showVelocity, bool showNeighborhood, bool showForces);
}; 