#include "Species.h"

Species::Species(const std::string& name, TrophicLevel level, MovementType moveType)
    : name(name), trophicLevel(level), movementType(moveType) {
    
    // Set default color based on trophic level
    switch (trophicLevel) {
        case TrophicLevel::APEX_PREDATOR:
            baseColor = ofColor(200, 0, 0);  // Red for apex predators
            break;
        case TrophicLevel::MID_PREDATOR:
            baseColor = ofColor(200, 100, 0);  // Orange for mid predators
            break;
        case TrophicLevel::PREY:
            baseColor = ofColor(0, 200, 100);  // Green for prey
            break;
        case TrophicLevel::APEX_PREY:
            baseColor = ofColor(100, 200, 255);  // Blue for apex prey
            break;
    }
    
    // Set default parameters based on trophic level
    switch (trophicLevel) {
        case TrophicLevel::APEX_PREDATOR:
            attackStrength = 8.0f;
            defenseStrength = 5.0f;
            maxSpeed = 2.5f;
            energyConsumptionRate = 0.15f;
            maxEnergy = 150.0f;
            break;
        case TrophicLevel::MID_PREDATOR:
            attackStrength = 5.0f;
            defenseStrength = 3.0f;
            maxSpeed = 2.2f;
            energyConsumptionRate = 0.12f;
            maxEnergy = 120.0f;
            break;
        case TrophicLevel::PREY:
            attackStrength = 1.0f;
            defenseStrength = 1.0f;
            maxSpeed = 2.0f;
            energyConsumptionRate = 0.08f;
            maxEnergy = 90.0f;
            break;
        case TrophicLevel::APEX_PREY:
            attackStrength = 3.0f;
            defenseStrength = 8.0f;
            maxSpeed = 1.8f;
            energyConsumptionRate = 0.07f;
            maxEnergy = 200.0f;
            break;
    }
    
    // Set default parameters based on movement type
    switch (movementType) {
        case MovementType::AERIAL:
            maxSpeed *= 1.3f;
            maneuverability = 0.9f;
            energyConsumptionRate *= 1.2f;
            break;
        case MovementType::TERRESTRIAL:
            maxSpeed *= 0.9f;
            maneuverability = 0.6f;
            energyConsumptionRate *= 0.9f;
            break;
    }
}

bool Species::canHunt(const std::string& otherSpecies) const {
    // Check if the other species is in our prey list
    return std::find(preySpecies.begin(), preySpecies.end(), otherSpecies) != preySpecies.end();
}

bool Species::isHuntedBy(const std::string& otherSpecies) const {
    // Check if the other species is in our predator list
    return std::find(predatorSpecies.begin(), predatorSpecies.end(), otherSpecies) != predatorSpecies.end();
} 