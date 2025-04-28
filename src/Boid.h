#pragma once

#include "ofMain.h"
#include "Particle.h"
#include "EcosystemTypes.h"
#include "Genetics.h"
#include "LifeCycle.h"
#include "Species.h"
#include "BoidRenderer.h"

// Forward declarations
class TerrainSystem;
class FoodSource;
class Octree;

class Boid : public Particle {
public:
    // Constructors
    Boid();
    Boid(const std::string& speciesName, TrophicLevel level, MovementType moveType);
    
    // Species and genetic information
    std::string speciesName;
    TrophicLevel trophicLevel;
    MovementType movementType;
    Genetics genetics;
    
    // State tracking
    BehaviorState behaviorState = BehaviorState::NORMAL;
    DefenseTactic defenseTactic = DefenseTactic::FLEE;
    LifeCycle lifecycle;
    
    // Current targets
    Boid* currentTarget = nullptr;     // Current prey being hunted
    Boid* currentThreat = nullptr;     // Current predator threat
    FoodSource* currentFood = nullptr; // Current food source
    
    // Territory
    ofVec3f territoryCenter;
    float territoryRadius = 10.0f;
    
    // Interaction cooldowns
    float attackCooldown = 0.0f;
    float lastAttackTime = 0.0f;
    float lastDefenseTime = 0.0f;
    
    // Defensive capabilities
    float hideEffectiveness = 0.0f;    // Current hiding (0.0-1.0)
    ofVec3f lastKnownSafePosition;     // Retreat position
    
    // Core flocking parameters
    float separationWeight = 1.5f;
    float alignmentWeight = 1.0f;
    float cohesionWeight = 1.0f;
    float neighborhoodRadius = 20.0f;
    float separationRadius = 10.0f;
    float fieldOfView = 270.0f;        // Angle in degrees (0-360)
    float maxSpeed = 2.0f;
    float minSpeed = 0.5f;
    float maxForce = 0.5f;
    
    // Visual representation
    ofColor personalColor;
    float size = 1.0f;
    
    // Rotation parameters
    float turnRate;           // Max rotation rate per frame
    ofVec3f previousDirection; // Previous direction
    
    // Individual characteristics
    float uniqueness;  // Deviation from group behavior (0-1)
    float energyLevel; // Affects movement
    
    // Predator-prey behaviors
    ofVec3f huntAsPredator(const std::vector<Boid*>& potentialPrey, TerrainSystem* terrain);
    bool attackTarget(Boid* target);
    void respondToAttack(Boid* attacker);
    void chooseBestDefenseTactic(Boid* attacker);
    
    // Defense behaviors
    ofVec3f executeDefenseTactic(TerrainSystem* terrain);
    ofVec3f executeFleeBehavior();
    ofVec3f executeHideBehavior(TerrainSystem* terrain);
    ofVec3f executeGroupDefenseBehavior();
    ofVec3f executeFightBehavior();
    
    // Competitor behaviors
    ofVec3f competitionBehavior(const std::vector<Boid*>& otherPredators);
    
    // Flocking behaviors
    ofVec3f separate(const std::vector<Boid*>& neighbors);
    ofVec3f align(const std::vector<Boid*>& neighbors);
    ofVec3f cohere(const std::vector<Boid*>& neighbors);
    ofVec3f wander(); // Random movement
    
    // Food seeking behavior
    ofVec3f seekFood(const std::vector<FoodSource*>& foodSources);
    bool isHungry() const;
    
    // Movement helpers
    ofVec3f seek(const ofVec3f& target);
    ofVec3f flee(const ofVec3f& target);
    void applyForce(ofVec3f force);
    bool isInFieldOfView(Boid* other);
    
    // Environment interaction helpers
    ofVec3f findNearestCover(TerrainSystem* terrain);
    float findDistanceToNearestCover(TerrainSystem* terrain);
    ofVec3f findFlockCenter(float radius);
    int countNearbyFlockmates(float radius);
    
    // Apply flocking behaviors
    void flock(std::vector<Boid*> boids);
    
    // Update methods
    void update(float deltaTime, TerrainSystem* terrain, Octree* octree);
    void integrate();
    
    // Rendering is now handled by BoidRenderer
    // void draw(ofMesh* customMesh = nullptr);
    // void drawDebug(bool showVelocity, bool showNeighborhood, bool showForces);
    
    // Apply species parameters
    void applySpeciesParams(const Species& species);
    
    // Debugging force vectors (accessible to BoidRenderer)
    ofVec3f separationForce;
    ofVec3f alignmentForce;
    ofVec3f cohesionForce;
    ofVec3f seekForce;
    ofVec3f wanderForce;
    ofVec3f boundaryForce;
    
    // Parameter update
    void updateParameters();
    
    // Predator-prey methods (ensure consistency, remove duplicate declaration if present)
    // void huntAsPredator(vector<Boid*>& potentialPrey); // Remove if duplicate of line 70

    // Friend class declaration allows BoidRenderer to access private/protected members if needed
    // (Alternatively, make necessary members public or provide getter methods)
    friend class BoidRenderer;
}; 
