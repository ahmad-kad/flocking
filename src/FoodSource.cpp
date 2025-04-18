#include "FoodSource.h"
#include "Boid.h"

FoodSource::FoodSource(const ofVec3f& pos, FoodSourceType type, float nutrition, float size) 
    : position(pos), type(type), nutritionalValue(nutrition), size(size) {
    
    // Set accessibility based on food type
    switch (type) {
        case FoodSourceType::PLANT:
            accessibleAerial = true;
            accessibleTerrestrial = true;
            detectionDifficulty = 0.2f;
            break;
            
        case FoodSourceType::CARRION:
            accessibleAerial = true;
            accessibleTerrestrial = true;
            detectionDifficulty = 0.1f;
            break;
            
        case FoodSourceType::INSECT_SWARM:
            accessibleAerial = true;
            accessibleTerrestrial = false;
            detectionDifficulty = 0.4f;
            break;
            
        case FoodSourceType::WATER_SOURCE:
            accessibleAerial = true;
            accessibleTerrestrial = true;
            detectionDifficulty = 0.1f;
            break;
    }
    
    // Record creation time
    lastConsumedTime = ofGetElapsedTimef();
}

bool FoodSource::isAccessibleTo(const Boid* boid) const {
    if (!available) return false;
    
    // Check based on movement type
    MovementType boidMovementType = boid->movementType;
    
    if (boidMovementType == MovementType::AERIAL && !accessibleAerial) {
        return false;
    }
    
    if (boidMovementType == MovementType::TERRESTRIAL && !accessibleTerrestrial) {
        return false;
    }
    
    return true;
}

void FoodSource::update(float deltaTime) {
    // Check if food should respawn
    if (!available) {
        float currentTime = ofGetElapsedTimef();
        if (currentTime - lastConsumedTime > respawnTime) {
            available = true;
        }
    }
    
    // Add movement for insect swarms only
    if (type == FoodSourceType::INSECT_SWARM && available) {
        // Create a gentle random movement for insect swarms
        float time = ofGetElapsedTimef() * 0.5f;
        float xMovement = sin(time + position.x * 0.1f) * 0.2f;
        float yMovement = cos(time * 1.2f + position.y * 0.1f) * 0.1f;
        float zMovement = sin(time * 0.8f + position.z * 0.1f) * 0.2f;
        
        // Update position with small random movements
        position.x += xMovement;
        position.y += yMovement + sin(time) * 0.05f; // Add slight up/down movement
        position.z += zMovement;
    }
}

float FoodSource::consume(Boid* consumer) {
    if (!available || !isAccessibleTo(consumer)) return 0.0f;
    
    // Mark as consumed
    available = false;
    lastConsumedTime = ofGetElapsedTimef();
    
    // Return nutrition value
    return nutritionalValue * seasonalAbundance;
}

void FoodSource::draw() const {
    if (!available) return;
    
    ofPushStyle();
    
    // Set color based on food type
    switch (type) {
        case FoodSourceType::PLANT:
            ofSetColor(0, 150, 0);  // Green for plants
            break;
        case FoodSourceType::CARRION:
            ofSetColor(150, 50, 50);  // Reddish for carrion
            break;
        case FoodSourceType::INSECT_SWARM:
            ofSetColor(200, 200, 0);  // Yellow for insects
            break;
        case FoodSourceType::WATER_SOURCE:
            ofSetColor(0, 100, 200);  // Blue for water
            break;
    }
    
    ofPushMatrix();
    ofTranslate(position);
    
    // Draw different shapes based on food type
    switch (type) {
        case FoodSourceType::PLANT:
            ofDrawCone(size, size * 2);
            break;
        case FoodSourceType::CARRION:
            ofDrawBox(size * 1.5f);
            break;
        case FoodSourceType::INSECT_SWARM:
            // Draw multiple small points to represent a swarm
            for (int i = 0; i < 10; i++) {
                ofDrawSphere(
                    ofRandom(-size, size),
                    ofRandom(-size, size),
                    ofRandom(-size, size),
                    size * 0.2f
                );
            }
            break;
        case FoodSourceType::WATER_SOURCE:
            ofDrawSphere(size);
            break;
    }
    
    ofPopMatrix();
    ofPopStyle();
} 