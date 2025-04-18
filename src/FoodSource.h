#pragma once

#include "ofMain.h"
#include "EcosystemTypes.h"

// Forward declaration
class Boid;

class FoodSource {
public:
    // Position and value
    ofVec3f position;
    float nutritionalValue = 20.0f;
    bool available = true;
    float respawnTime = 30.0f;
    float lastConsumedTime = 0.0f;
    float size = 1.0f;
    FoodSourceType type;
    
    // Visibility parameters
    float detectionDifficulty = 0.3f;  // How hard to spot (0.0-1.0)
    float seasonalAbundance = 0.8f;    // Seasonal availability (0.0-1.0)
    
    // Domain accessibility flags
    bool accessibleAerial = true;
    bool accessibleTerrestrial = true;
    
    // Constructor
    FoodSource(const ofVec3f& pos, FoodSourceType type, float nutrition, float size);
    
    // Virtual methods
    virtual bool isAccessibleTo(const Boid* boid) const;
    virtual void update(float deltaTime);
    virtual float consume(Boid* consumer);
    virtual void draw() const;
    
    // Destructor
    virtual ~FoodSource() = default;
}; 