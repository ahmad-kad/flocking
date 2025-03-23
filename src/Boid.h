#pragma once

#include "ofMain.h"
#include "Particle.h"

class Boid : public Particle {
public:
    Boid();
    
    // Flocking behavior parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float neighborhoodRadius;
    float separationRadius;
    float maxSpeed;
    float minSpeed;   
    float maxForce;
    
    // Rotation control parameters
    float fieldOfView;        // Field of view angle in degrees (0-360)
    float turnRate;           // Maximum rotation rate per frame
    ofVec3f previousDirection; // Keep track of previous frame's direction
    
    // Individualistic characteristics
    float uniqueness;  // How much this boid deviates from group behavior (0-1)
    ofColor personalColor;  // Individual color 
    float size;        // Individual size
    float energyLevel; // Affects movement patterns
    
    // Flocking behavior methods
    ofVec3f separate(vector<Boid*> neighbors);
    ofVec3f align(vector<Boid*> neighbors);
    ofVec3f cohere(vector<Boid*> neighbors);
    ofVec3f seek(ofVec3f target);
    ofVec3f wander(); // Random individualistic movement
    
    // Apply flocking behaviors
    void flock(vector<Boid*> boids);
    
    // Apply steering force
    void applyForce(ofVec3f force);
    
    // Neighbor check based on field of view
    bool isInFieldOfView(Boid* other);
    
    // Override draw method
    void draw(ofMesh* customMesh = nullptr);
    void drawDebug(bool showVelocity, bool showNeighborhood, bool showForces);
    
    // Override integrate method to include max speed
    void integrate();
    
    // Force vectors for debugging
    ofVec3f separationForce;
    ofVec3f alignmentForce;
    ofVec3f cohesionForce;
    ofVec3f seekForce;
    ofVec3f wanderForce;
    ofVec3f boundaryForce;
}; 