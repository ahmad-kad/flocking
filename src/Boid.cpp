#include "Boid.h"

// Constructor for Boid class
Boid::Boid() {
    // Initialize with default values
    separationWeight = 1.5;
    alignmentWeight = 1.0;
    cohesionWeight = 1.0;
    neighborhoodRadius = 3.0;
    separationRadius = 1.0;
    maxForce = 0.5;
    maxSpeed = 3.0;
    
    // Initialize forces to zero
    separationForce = ofVec3f(0, 0, 0);
    alignmentForce = ofVec3f(0, 0, 0);
    cohesionForce = ofVec3f(0, 0, 0);
    seekForce = ofVec3f(0, 0, 0);
}

Boid::~Boid() {
    // Cleanup if needed
}

// Apply all flocking behaviors
void Boid::flock(vector<Boid*> boids) {
    vector<Boid*> neighbors = getNeighbors(boids);
    
    // Calculate flocking forces
    separationForce = separate(neighbors) * separationWeight;
    alignmentForce = align(neighbors) * alignmentWeight;
    cohesionForce = cohere(neighbors) * cohesionWeight;
    
    // Apply all forces
    forces += separationForce;
    forces += alignmentForce;
    forces += cohesionForce;
}

// Update method that overrides the one from Particle
void Boid::update() {
    // Limit force to maximum
    if (forces.length() > maxForce) {
        forces.normalize();
        forces *= maxForce;
    }
    
    // Apply forces to acceleration
    acceleration = forces / mass;
    forces = ofVec3f(0, 0, 0);
    
    // Update velocity
    velocity += acceleration;
    
    // Limit velocity to maximum speed
    if (velocity.length() > maxSpeed) {
        velocity.normalize();
        velocity *= maxSpeed;
    }
    
    // Update position
    position += velocity;
    
    // Apply damping
    velocity *= damping;
}

// Draw the boid with orientation based on velocity
void Boid::draw() {
    ofPushMatrix();
    ofTranslate(position);
    
    // Create rotation matrix to orient the boid in the direction it's moving
    ofVec3f heading = velocity;
    
    if (heading.length() > 0) {
        heading.normalize();
        
        // Calculate the rotation axis and angle
        ofVec3f axis(0, 1, 0);
        ofVec3f forward(0, 0, 1);
        ofVec3f rotationAxis = forward.cross(heading);
        float rotationAngle = acos(forward.dot(heading)) * RAD_TO_DEG;
        
        if (rotationAxis.length() > 0) {
            ofRotateDeg(rotationAngle, rotationAxis.x, rotationAxis.y, rotationAxis.z);
        }
    }
    
    // Draw a cone-like shape for the boid
    ofSetColor(color);
    ofDrawCone(0, 0, 0, radius, radius * 2);
    
    ofPopMatrix();
}

// Separation: steer to avoid crowding local flockmates
ofVec3f Boid::separate(vector<Boid*> neighbors) {
    ofVec3f steer(0, 0, 0);
    int count = 0;
    
    // For every nearby boid, calculate direction away from it
    for (Boid* other : neighbors) {
        float d = position.distance(other->position);
        
        // Check if the neighbor is within separation radius
        if ((d > 0) && (d < separationRadius)) {
            // Calculate vector pointing away from neighbor
            ofVec3f diff = position - other->position;
            diff.normalize();
            
            // Weight by distance (closer neighbors have more influence)
            diff /= d;
            steer += diff;
            count++;
        }
    }
    
    // Average
    if (count > 0) {
        steer /= count;
    }
    
    // If the steering force is greater than 0
    if (steer.length() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxSpeed;
        steer -= velocity;
        
        // Limit the steering force
        if (steer.length() > maxForce) {
            steer.normalize();
            steer *= maxForce;
        }
    }
    
    return steer;
}

// Alignment: steer towards the average heading of local flockmates
ofVec3f Boid::align(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum up velocities of all neighbors
    for (Boid* other : neighbors) {
        float d = position.distance(other->position);
        
        // Check if the neighbor is within neighborhood radius
        if ((d > 0) && (d < neighborhoodRadius)) {
            sum += other->velocity;
            count++;
        }
    }
    
    if (count > 0) {
        sum /= count; // Average velocity
        sum.normalize();
        sum *= maxSpeed; // Desired velocity
        
        // Steering = Desired - Current
        ofVec3f steer = sum - velocity;
        
        // Limit the steering force
        if (steer.length() > maxForce) {
            steer.normalize();
            steer *= maxForce;
        }
        
        return steer;
    }
    else {
        return ofVec3f(0, 0, 0);
    }
}

// Cohesion: steer to move toward the average position of local flockmates
ofVec3f Boid::cohere(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum up positions of all neighbors
    for (Boid* other : neighbors) {
        float d = position.distance(other->position);
        
        // Check if the neighbor is within neighborhood radius
        if ((d > 0) && (d < neighborhoodRadius)) {
            sum += other->position;
            count++;
        }
    }
    
    if (count > 0) {
        sum /= count; // Average position
        return seek(sum); // Seek toward the average position
    }
    else {
        return ofVec3f(0, 0, 0);
    }
}

// Seek: steer towards a target position
ofVec3f Boid::seek(ofVec3f target) {
    // Desired velocity: direction to target, at maximum speed
    ofVec3f desired = target - position;
    
    if (desired.length() > 0) {
        desired.normalize();
        desired *= maxSpeed;
        
        // Steering = Desired - Current
        ofVec3f steer = desired - velocity;
        
        // Limit the steering force
        if (steer.length() > maxForce) {
            steer.normalize();
            steer *= maxForce;
        }
        
        return steer;
    }
    else {
        return ofVec3f(0, 0, 0);
    }
}

// Get neighbors from all boids (without spatial optimization)
vector<Boid*> Boid::getNeighbors(vector<Boid*> allBoids) {
    vector<Boid*> neighbors;
    
    for (Boid* other : allBoids) {
        float d = position.distance(other->position);
        
        // Check if the other boid is within neighborhood radius and not this boid
        if (other != this && d < neighborhoodRadius && isInFieldOfView(other)) {
            neighbors.push_back(other);
        }
    }
    
    return neighbors;
}

// Check if another boid is within this boid's field of view
// For now, assume 360 degree field of view (no restriction)
bool Boid::isInFieldOfView(Boid* other) {
    // Could implement a field of view check here
    // For now, just return true (360 degree vision)
    return true;
}

// FlockingSystem Implementation
FlockingSystem::FlockingSystem() {
    // Default parameters
    separationWeight = 1.5;
    alignmentWeight = 1.0;
    cohesionWeight = 1.0;
    neighborhoodRadius = 3.0;
    separationRadius = 1.0;
    maxSpeed = 3.0;
    maxForce = 0.5;
    seekTarget = false;
    target = ofVec3f(0, 0, 0);
}

FlockingSystem::~FlockingSystem() {
    // Clean up all boids
    for (Boid* boid : boids) {
        delete boid;
    }
    boids.clear();
}

void FlockingSystem::update() {
    // Apply flocking behavior to all boids
    for (Boid* boid : boids) {
        // Update each boid's flocking parameters
        boid->separationWeight = separationWeight;
        boid->alignmentWeight = alignmentWeight;
        boid->cohesionWeight = cohesionWeight;
        boid->neighborhoodRadius = neighborhoodRadius;
        boid->separationRadius = separationRadius;
        boid->maxSpeed = maxSpeed;
        boid->maxForce = maxForce;
        
        // Apply flocking behavior
        boid->flock(boids);
        
        // Apply target seeking if enabled
        if (seekTarget) {
            ofVec3f seekForce = boid->seek(target);
            boid->forces += seekForce;
        }
        
        // Update the boid's physics
        boid->update();
    }
}

void FlockingSystem::draw() {
    // Draw all boids
    for (Boid* boid : boids) {
        boid->draw();
    }
    
    // Draw target if seeking is enabled
    if (seekTarget) {
        ofSetColor(ofColor::red);
        ofDrawSphere(target, 0.3);
    }
}

void FlockingSystem::addBoid(ofVec3f position, ofVec3f velocity) {
    Boid* boid = new Boid();
    boid->position = position;
    boid->velocity = velocity;
    
    // Set random color with good opacity
    boid->color = ofColor(ofRandom(50, 255), ofRandom(50, 255), ofRandom(50, 255), 200);
    boid->mass = 1.0;
    boid->damping = 0.99;
    boid->radius = 0.2;
    
    boids.push_back(boid);
}

void FlockingSystem::removeBoid(int index) {
    if (index >= 0 && index < boids.size()) {
        delete boids[index];
        boids.erase(boids.begin() + index);
    }
}

// Simplistic neighbor finding without spatial optimization
// For larger flocks, this would be replaced with spatial partitioning
vector<Boid*> FlockingSystem::getNeighbors(Boid* boid, float radius) {
    vector<Boid*> neighbors;
    
    for (Boid* other : boids) {
        if (other != boid && boid->position.distance(other->position) < radius) {
            neighbors.push_back(other);
        }
    }
    
    return neighbors;
} 