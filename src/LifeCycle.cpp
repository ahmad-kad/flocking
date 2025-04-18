#include "LifeCycle.h"

LifeCycle::LifeCycle(float maxAge, float maxEnergy) 
    : maxAge(maxAge), maxEnergy(maxEnergy) {
    // Initialize with full energy
    energy = maxEnergy;
    
    // Record starting time for meal tracking
    lastMealTime = ofGetElapsedTimef();
}

void LifeCycle::update(float deltaTime, float baseConsumption) {
    if (!isAlive) return;
    
    // Update age
    age += deltaTime;
    
    // Calculate maturity (reaches 1.0 at 30% of max age)
    maturity = ofClamp(age / (maxAge * 0.3f), 0.0f, 1.0f);
    
    // Consume energy based on base consumption rate (modified by stress)
    float consumption = baseConsumption * (1.0f + stress * 0.5f) * deltaTime;
    energy = ofClamp(energy - consumption, 0.0f, maxEnergy);
    
    // Update hunger level (0 = full, 1 = starving)
    hungerLevel = 1.0f - (energy / maxEnergy);
    
    // Update stress - increases with hunger
    if (hungerLevel > 0.7f) {
        stress += 0.01f * deltaTime;
    } else if (hungerLevel < 0.3f) {
        stress -= 0.01f * deltaTime;
    }
    stress = ofClamp(stress, 0.0f, 1.0f);
    
    // Health effects from stress and hunger
    if (hungerLevel > 0.8f || stress > 0.8f) {
        health -= 0.01f * deltaTime;
    } else if (hungerLevel < 0.3f && stress < 0.3f) {
        health += 0.005f * deltaTime;
    }
    health = ofClamp(health, 0.0f, 1.0f);
    
    // Check for death from starvation or old age
    if (energy <= 0 || health <= 0) {
        isAlive = false;
    } else if (age >= maxAge) {
        isAlive = false;
    }
    
    // Update reproductive readiness
    if (maturity >= 1.0f && !isPregnant && energy > maxEnergy * 0.5f) {
        reproductiveReadiness += 0.005f * deltaTime;
    } else {
        reproductiveReadiness -= 0.01f * deltaTime;
    }
    reproductiveReadiness = ofClamp(reproductiveReadiness, 0.0f, 1.0f);
    
    // Update pregnancy
    if (isPregnant) {
        gestationProgress += 0.01f * deltaTime;
        
        // Extra energy consumption during pregnancy
        energy -= 0.05f * deltaTime;
        
        // Complete pregnancy
        if (gestationProgress >= 1.0f) {
            isPregnant = false;
            gestationProgress = 0.0f;
            reproductiveReadiness = 0.0f;
        }
    }
}

bool LifeCycle::feed(float nutritionValue) {
    // Only can feed if alive
    if (!isAlive) return false;
    
    // Add energy from nutrition value
    float previousEnergy = energy;
    energy = ofClamp(energy + nutritionValue, 0.0f, maxEnergy);
    
    // Record last meal time
    lastMealTime = ofGetElapsedTimef();
    
    // Return true if feeding was beneficial
    return energy > previousEnergy;
}

bool LifeCycle::takeDamage(float amount) {
    // Only take damage if alive
    if (!isAlive) return false;
    
    // Reduce health by damage amount
    health -= amount;
    
    // Increase stress from being attacked
    stress += amount * 0.5f;
    stress = ofClamp(stress, 0.0f, 1.0f);
    
    // Check if still alive
    if (health <= 0) {
        isAlive = false;
        health = 0;
        return true; // Fatal damage
    }
    
    return false; // Non-fatal damage
}

void LifeCycle::conceive() {
    if (!isAlive || isPregnant || maturity < 1.0f) return;
    
    isPregnant = true;
    gestationProgress = 0.0f;
    
    // Pregnancy reduces energy
    energy *= 0.8f;
}

bool LifeCycle::canReproduce() const {
    return isAlive && maturity >= 1.0f && !isPregnant && 
           reproductiveReadiness > 0.8f && energy > maxEnergy * 0.6f;
} 