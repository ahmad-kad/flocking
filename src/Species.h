#pragma once

#include "ofMain.h"
#include "EcosystemTypes.h"

class Species {
public:
    // Identification
    std::string name;
    TrophicLevel trophicLevel;
    MovementType movementType;
    
    // Combat capabilities
    float attackStrength = 1.0f;    // Base damage (0.0-10.0)
    float defenseStrength = 1.0f;   // Damage reduction (0.0-10.0)
    float detectionRange = 10.0f;   // Vision radius
    float stealthFactor = 0.5f;     // Hide ability (0.0-1.0)
    
    // Movement parameters
    float maxSpeed = 2.0f;          // Maximum units/second
    float acceleration = 0.5f;      // Units/second²
    float maneuverability = 0.7f;   // Turning rate (0.0-1.0)
    
    // Flocking parameters
    float separationWeight = 1.5f;  // Force multiplier
    float alignmentWeight = 1.0f;   // Force multiplier
    float cohesionWeight = 1.0f;    // Force multiplier
    
    // Lifecycle parameters
    float maxEnergy = 100.0f;       // Maximum energy capacity
    float energyConsumptionRate = 0.1f; // Units/second
    float maxAge = 100.0f;          // Simulation seconds
    float reproductionRate = 0.005f; // Chance per second
    
    // Prey and predator relationships
    std::vector<std::string> preySpecies;
    std::vector<std::string> predatorSpecies;
    
    // Visual representation
    ofColor baseColor = ofColor(128, 128, 128);
    
    // Constructor with required parameters
    Species(const std::string& name, TrophicLevel level, MovementType moveType);
    
    // Check if this species can hunt another
    bool canHunt(const std::string& otherSpecies) const;
    
    // Check if this species is hunted by another
    bool isHuntedBy(const std::string& otherSpecies) const;
}; 