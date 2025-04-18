#pragma once

#include "ofMain.h"

class LifeCycle {
public:
    // Age tracking
    float age = 0.0f;               // Current age in seconds
    float maxAge = 100.0f;          // Maximum lifespan
    float maturity = 0.0f;          // Development level (0.0-1.0)
    
    // Reproduction
    float reproductiveReadiness = 0.0f; // Readiness to mate (0.0-1.0)
    float gestationProgress = 0.0f;  // Pregnancy progress (0.0-1.0)
    bool isPregnant = false;         // Currently pregnant
    
    // Energy and health
    float energy = 100.0f;          // Current energy
    float maxEnergy = 100.0f;       // Maximum energy capacity
    float hungerLevel = 0.0f;       // Hunger status (0.0-1.0)
    float lastMealTime = 0.0f;      // Simulation time of last feeding
    
    // Vital status
    bool isAlive = true;            // Currently alive
    float health = 1.0f;            // Current health (0.0-1.0)
    float stress = 0.0f;            // Current stress level (0.0-1.0)
    
    // Constructor
    LifeCycle(float maxAge = 100.0f, float maxEnergy = 100.0f);
    
    // Update lifecycle state
    void update(float deltaTime, float baseConsumption);
    
    // Feed to restore energy
    bool feed(float nutritionValue);
    
    // Take damage from attack
    bool takeDamage(float amount);
    
    // Start pregnancy
    void conceive();
    
    // Check if reproduction is possible
    bool canReproduce() const;
}; 