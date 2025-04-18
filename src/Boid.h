#pragma once

#include "ofMain.h"
#include "Particle.h"

class Boid : public Particle {
public:
    Boid();
    
    // Flocking parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float neighborhoodRadius;
    float separationRadius;
    float maxSpeed;
    float minSpeed;   
    float maxForce;
    
    // Rotation parameters
    float fieldOfView;        // Angle in degrees (0-360)
    float turnRate;           // Max rotation rate per frame
    ofVec3f previousDirection; // Previous direction
    
    // Individual characteristics
    float uniqueness;  // Deviation from group behavior (0-1)
    ofColor personalColor;  // Color 
    float size;        // Size
    float energyLevel; // Affects movement
    
    // Flocking methods
    ofVec3f separate(vector<Boid*> neighbors);
    ofVec3f align(vector<Boid*> neighbors);
    ofVec3f cohere(vector<Boid*> neighbors);
    ofVec3f seek(ofVec3f target);
    ofVec3f wander(); // Random movement
    
    // Apply flocking behaviors
    void flock(vector<Boid*> boids);
    
    // Apply force
    void applyForce(ofVec3f force);
    
    // Check neighbor visibility
    bool isInFieldOfView(Boid* other);
    
    // Draw methods
    void draw(ofMesh* customMesh = nullptr);
    void drawDebug(bool showVelocity, bool showNeighborhood, bool showForces);
    
    // Integrate method with max speed
    void integrate();
    
    // Debugging force vectors
    ofVec3f separationForce;
    ofVec3f alignmentForce;
    ofVec3f cohesionForce;
    ofVec3f seekForce;
    ofVec3f wanderForce;
    ofVec3f boundaryForce;
}; 