#pragma once

#include "ofMain.h"
#include "Particle.h"

class Boid : public Particle {
public:
    Boid();
    ~Boid();
    
    // Flocking behavior functions
    ofVec3f separate(vector<Boid*> neighbors);
    ofVec3f align(vector<Boid*> neighbors);
    ofVec3f cohere(vector<Boid*> neighbors);
    ofVec3f seek(ofVec3f target);
    void flock(vector<Boid*> boids);
    void update();
    void draw();
    
    // Perception parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float neighborhoodRadius;
    float separationRadius;
    
    // Maximum force and speed limits
    float maxForce;
    float maxSpeed;
    
    // Calculated forces
    ofVec3f separationForce;
    ofVec3f alignmentForce;
    ofVec3f cohesionForce;
    ofVec3f seekForce;
    
    // Utility methods
    vector<Boid*> getNeighbors(vector<Boid*> allBoids);
    bool isInFieldOfView(Boid* other);
};

// Flocking system to manage boids
class FlockingSystem {
public:
    FlockingSystem();
    ~FlockingSystem();
    
    void update();
    void draw();
    void addBoid(ofVec3f position, ofVec3f velocity);
    void removeBoid(int index);
    
    // Flock parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float neighborhoodRadius;
    float separationRadius;
    float maxSpeed;
    float maxForce;
    
    // Optional target seeking
    bool seekTarget;
    ofVec3f target;
    
    vector<Boid*> boids;
    
    // Spatial partitioning for neighbor search optimization
    // This is a simplistic approach - could be expanded with octrees or grids
    vector<Boid*> getNeighbors(Boid* boid, float radius);
}; 