#pragma once

#include "ofMain.h"

// Strict hierarchy of predator and prey relationships
enum class TrophicLevel {
    APEX_PREDATOR,   // Top predator, hunts mid-predators and prey
    MID_PREDATOR,    // Intermediate predator, hunts prey
    PREY,            // Standard prey species
    APEX_PREY        // Special prey that cannot be killed
};

// Domain-specific movement types
enum class MovementType {
    AERIAL,          // Flying creatures
    TERRESTRIAL      // Ground-based creatures
};

// Explicit behavior states for state machine
enum class BehaviorState {
    NORMAL,         // Standard flocking
    HUNTING,        // Actively pursuing prey
    FLEEING,        // Running from predators
    HIDING,         // Using cover to avoid detection
    FIGHTING,       // Defending against attack
    FEEDING,        // Consuming food
    MATING,         // Reproductive behavior
    TERRITORIAL,    // Defending territory
    RESTING         // Conserving energy
};

// Specific defensive tactics
enum class DefenseTactic {
    FLEE,           // Run away at maximum speed
    HIDE,           // Seek cover and reduce visibility
    GROUP_DEFENSE,  // Stay with flock for safety
    FIGHT_BACK      // Counter-attack predator
};

// Food source types
enum class FoodSourceType {
    PLANT,              // For herbivores (grass, fruit)
    CARRION,            // For scavengers and omnivores
    INSECT_SWARM,       // Small abundant food source
    WATER_SOURCE        // Hydration (not nutritional)
};

// Terrain types
enum class TerrainType {
    OPEN_FIELD,
    FOREST,
    WATER,
    MOUNTAIN,
    CAVE
};

// Event types
enum class EventType {
    PREDATION,           // Predator kills prey
    MUTATION,            // Mutation occurs
    BOSS_SPAWNED,        // Boss-level mutation
    BIRTH,               // New offspring
    DEATH_AGE,           // Death from old age
    DEATH_HUNGER,        // Death from starvation
    DEATH_PREDATION,     // Death from predator
    TERRITORY_CLAIM,     // Territory established
    FOOD_CONSUMED,       // Food source consumed
    ECOSYSTEM_IMBALANCE, // Population imbalance detected
    APEX_PREY_DETECTED   // Special apex prey spawned/detected
}; 