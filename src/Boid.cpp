#include "Boid.h"
#include "Octree.h"
#include "FoodSource.h"
#include "FlockSystem.h"

Boid::Boid() : Particle() {
    // Initialize basic flocking parameters
    separationWeight = 1.5f;
    alignmentWeight = 1.0f;
    cohesionWeight = 1.0f;
    neighborhoodRadius = 20.0f;
    separationRadius = 10.0f;
    maxSpeed = 2.0f;
    minSpeed = 0.5f;
    maxForce = 0.5f;
    
    // Default field of view and turn rate
    fieldOfView = 270.0f;
    turnRate = 2.0f;
    
    // Individual variation
    uniqueness = ofRandom(0.0f, 1.0f);
    personalColor = ofColor(ofRandom(100, 255), ofRandom(100, 255), ofRandom(100, 255));
    size = ofRandom(0.8f, 1.2f);
    energyLevel = 1.0f;
    
    // Default species parameters
    speciesName = "Generic";
    trophicLevel = TrophicLevel::PREY;
    movementType = MovementType::TERRESTRIAL;
    
    // Initialize territory
    territoryCenter = position;
    
    // Set initial behavior state
    behaviorState = BehaviorState::NORMAL;
}

Boid::Boid(const std::string& speciesName, TrophicLevel level, MovementType moveType) : Boid() {
    this->speciesName = speciesName;
    this->trophicLevel = level;
    this->movementType = moveType;
    
    // Set color based on trophic level
    switch (trophicLevel) {
        case TrophicLevel::APEX_PREDATOR:
            personalColor = ofColor(200, 0, 0);
            break;
        case TrophicLevel::MID_PREDATOR:
            personalColor = ofColor(200, 100, 0);
            break;
        case TrophicLevel::PREY:
            personalColor = ofColor(0, 200, 100);
            break;
        case TrophicLevel::APEX_PREY:
            personalColor = ofColor(100, 200, 255);
            break;
    }
    
    // Adjust size based on genetics and trophic level
    size = genetics.size;
    switch (trophicLevel) {
        case TrophicLevel::APEX_PREDATOR:
            size *= 1.3f;
            break;
        case TrophicLevel::MID_PREDATOR:
            size *= 1.1f;
            break;
        case TrophicLevel::PREY:
            size *= 0.9f;
            break;
        case TrophicLevel::APEX_PREY:
            size *= 1.2f;
            break;
    }
    
    // Set movement parameters based on type
    if (movementType == MovementType::AERIAL) {
        maxSpeed *= 1.3f;
    } else {
        maxSpeed *= 0.9f;
    }
}

void Boid::applySpeciesParams(const Species& species) {
    // Apply flocking parameters
    separationWeight = species.separationWeight;
    alignmentWeight = species.alignmentWeight;
    cohesionWeight = species.cohesionWeight;
    
    // Apply movement parameters
    maxSpeed = species.maxSpeed * genetics.speed;
    
    // Set personal color based on species with slight variation
    personalColor = species.baseColor;
    personalColor.r = ofClamp(personalColor.r + genetics.colorVariation.r, 0, 255);
    personalColor.g = ofClamp(personalColor.g + genetics.colorVariation.g, 0, 255);
    personalColor.b = ofClamp(personalColor.b + genetics.colorVariation.b, 0, 255);
    
    // Apply genetics to physical traits
    size = species.defenseStrength * 0.2f * genetics.size;
    
    // Setup lifecycle
    lifecycle.maxAge = species.maxAge;
    lifecycle.maxEnergy = species.maxEnergy * genetics.stamina;
    
    // Initialize energy
    lifecycle.energy = lifecycle.maxEnergy;
}

void Boid::update(float deltaTime, TerrainSystem* terrain, Octree* octree) {
    // Skip update if dead
    if (!lifecycle.isAlive) return;
    
    // Update lifecycle
    lifecycle.update(deltaTime, 0.1f);  // Base consumption rate
    
    // Update cooldowns
    if (attackCooldown > 0) {
        attackCooldown -= deltaTime;
    }
    
    // Apply speed limits based on movement type and trophic level
    float typeMaxSpeed = maxSpeed;
    
    // Movement type modifications
    if (movementType == MovementType::AERIAL) {
        // Aerial creatures move slower to prevent excessive speed
        typeMaxSpeed *= 0.6f;
    } else if (movementType == MovementType::TERRESTRIAL) {
        // Terrestrial creatures are slightly slower
        typeMaxSpeed *= 0.8f;
    }
    
    // Trophic level modifications
    if (trophicLevel == TrophicLevel::APEX_PREDATOR) {
        // Apex predators move faster when hunting
        if (behaviorState == BehaviorState::HUNTING) {
            typeMaxSpeed *= 1.2f;
        }
    } else if (trophicLevel == TrophicLevel::PREY) {
        // Prey move faster when fleeing
        if (behaviorState == BehaviorState::FLEEING) {
            typeMaxSpeed *= 1.3f;
        } else if (behaviorState == BehaviorState::FEEDING) {
            typeMaxSpeed *= 0.5f; // Much slower when feeding
        }
    }
    
    // Limit velocity to max speed
    if (velocity.length() > typeMaxSpeed) {
        velocity.normalize();
        velocity *= typeMaxSpeed;
    }
    
    // Ensure minimum speed 
    if (velocity.length() < minSpeed && velocity.length() > 0) {
        velocity.normalize();
        velocity *= minSpeed;
    }
    
    // Store previous direction
    previousDirection = velocity.getNormalized();
    
    // Apply forces to modify velocity
    velocity += acceleration * deltaTime;
    
    // Reset acceleration
    acceleration = ofVec3f(0, 0, 0);
    
    // Update positions
    position += velocity * deltaTime;
    
    // Update behavior based on state
    ofVec3f steeringForce = ofVec3f(0, 0, 0);
    
    // Get nearby boids using octree
    std::vector<Boid*> nearbyBoids = octree->queryRange(position, neighborhoodRadius);
    
    // Default flocking behavior
    if (behaviorState == BehaviorState::NORMAL) {
        flock(nearbyBoids);
    }
    // Predator hunting behavior
    else if (behaviorState == BehaviorState::HUNTING && currentTarget) {
        steeringForce = seek(currentTarget->position);
    }
    // Fleeing behavior
    else if (behaviorState == BehaviorState::FLEEING && currentThreat) {
        steeringForce = executeDefenseTactic(terrain);
    }
    // Feeding behavior
    else if (behaviorState == BehaviorState::FEEDING && currentFood) {
        steeringForce = seek(currentFood->position);
    }
    // Resting behavior
    else if (behaviorState == BehaviorState::RESTING) {
        steeringForce = wander() * 0.3f; // Minimal movement when resting
    }
    
    // Apply the steering force
    applyForce(steeringForce);
    
    // Integrate position and velocity
    integrate();
}

void Boid::integrate() {
    // Store previous direction for smooth turning
    previousDirection = velocity.getNormalized();
    
    // Add acceleration to velocity
    velocity += acceleration;
    
    // Limit velocity by max speed
    if (velocity.length() > maxSpeed) {
        velocity.normalize();
        velocity *= maxSpeed;
    }
    
    // Enforce minimum speed
    if (velocity.length() < minSpeed && velocity.length() > 0) {
        velocity.normalize();
        velocity *= minSpeed;
    }
    
    // Update position
    position += velocity;
    
    // Reset acceleration
    acceleration *= 0;
}

void Boid::flock(std::vector<Boid*> boids) {
    // If no boids nearby, just wander
    if (boids.empty()) {
        applyForce(wander());
        return;
    }
    
    // Initialize forces
    separationForce = ofVec3f(0);
    alignmentForce = ofVec3f(0);
    cohesionForce = ofVec3f(0);
    
    // Filter to only include boids in field of view
    std::vector<Boid*> visibleBoids;
    for (auto* other : boids) {
        if (other != this && isInFieldOfView(other)) {
            visibleBoids.push_back(other);
        }
    }
    
    // Apply flocking behaviors
    ofVec3f sep = separate(visibleBoids);
    ofVec3f ali = align(visibleBoids);
    ofVec3f coh = cohere(visibleBoids);
    ofVec3f wan = wander();
    
    // Add flocking forces to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
    
    // Apply small wander force for natural movement
    applyForce(wan * 0.3f);
    
    // Add additional forces based on individual uniqueness
    if (uniqueness > 0.7f) {
        // Highly unique boids wander more
        applyForce(wander() * uniqueness * 0.5f);
    }
}

bool Boid::isInFieldOfView(Boid* other) {
    if (!other) return false;
    
    // Get direction to other boid
    ofVec3f toOther = other->position - position;
    
    // Get heading direction (normalized velocity)
    ofVec3f heading;
    if (velocity.length() > 0.1f) {
        heading = velocity.getNormalized();
    } else {
        // If not moving, use previous direction or default forward
        heading = previousDirection.length() > 0.1f ? previousDirection : ofVec3f(1, 0, 0);
    }
    
    // Calculate angle between heading and direction to other
    float angle = acos(heading.dot(toOther.getNormalized())) * RAD_TO_DEG;
    
    // Check if within field of view
    return angle <= fieldOfView * 0.5f;
}

ofVec3f Boid::separate(const std::vector<Boid*>& neighbors) {
    // Keep a minimum distance from other boids to avoid crowding
    ofVec3f steeringForce = ofVec3f(0, 0, 0);
    int count = 0;
    
    for (auto* other : neighbors) {
        // Skip self
        if (other == this) continue;
        
        // Calculate vector pointing away from neighbor
        ofVec3f diff = position - other->position;
        float distance = diff.length();
        
        // Only consider neighbors within separation radius
        if (distance > 0 && distance < separationRadius) {
            // Weight inversely proportional to distance
            diff.normalize();
            diff /= distance; // Closer boids have stronger influence
            steeringForce += diff;
            count++;
        }
    }
    
    // Average the forces and apply separation weight
    if (count > 0) {
        steeringForce /= count;
        steeringForce.normalize();
        steeringForce *= maxForce;
        
        // Store for debugging visualization
        separationForce = steeringForce * separationWeight;
        return steeringForce * separationWeight;
    }
    
    separationForce = ofVec3f(0, 0, 0);
    return steeringForce;
}

ofVec3f Boid::align(const std::vector<Boid*>& neighbors) {
    // Align with the average heading of nearby boids
    ofVec3f averageVelocity = ofVec3f(0, 0, 0);
    int count = 0;
    
    for (auto* other : neighbors) {
        // Skip self or different species
        if (other == this || other->speciesName != speciesName) continue;
        
        // Only consider neighbors within neighborhood radius and in field of view
        float distance = (other->position - position).length();
        if (distance > 0 && distance < neighborhoodRadius && isInFieldOfView(other)) {
            averageVelocity += other->velocity;
            count++;
        }
    }
    
    // Calculate steering force towards average velocity
    if (count > 0) {
        averageVelocity /= count;
        averageVelocity.normalize();
        averageVelocity *= maxSpeed;
        
        ofVec3f steeringForce = averageVelocity - velocity;
        if (steeringForce.length() > maxForce) {
            steeringForce.normalize();
            steeringForce *= maxForce;
        }
        
        // Store for debugging visualization
        alignmentForce = steeringForce * alignmentWeight;
        return steeringForce * alignmentWeight;
    }
    
    alignmentForce = ofVec3f(0, 0, 0);
    return ofVec3f(0, 0, 0);
}

ofVec3f Boid::cohere(const std::vector<Boid*>& neighbors) {
    // Move toward the average position of nearby boids
    ofVec3f centerOfMass = ofVec3f(0, 0, 0);
    int count = 0;
    
    for (auto* other : neighbors) {
        // Skip self or different species
        if (other == this || other->speciesName != speciesName) continue;
        
        // Only consider neighbors within neighborhood radius and in field of view
        float distance = (other->position - position).length();
        if (distance > 0 && distance < neighborhoodRadius && isInFieldOfView(other)) {
            centerOfMass += other->position;
            count++;
        }
    }
    
    // Calculate steering force towards center of mass
    if (count > 0) {
        centerOfMass /= count;
        ofVec3f desiredVelocity = centerOfMass - position;
        
        // Only steer if the center of mass is far enough away
        if (desiredVelocity.length() > 0.1f) {
            desiredVelocity.normalize();
            desiredVelocity *= maxSpeed;
            
            ofVec3f steeringForce = desiredVelocity - velocity;
            if (steeringForce.length() > maxForce) {
                steeringForce.normalize();
                steeringForce *= maxForce;
            }
            
            // Store for debugging visualization
            cohesionForce = steeringForce * cohesionWeight;
            return steeringForce * cohesionWeight;
        }
    }
    
    cohesionForce = ofVec3f(0, 0, 0);
    return ofVec3f(0, 0, 0);
}

ofVec3f Boid::wander() {
    // Random movement with some persistence
    float wanderRadius = 1.0f;
    float wanderDistance = 2.0f;
    float wanderJitter = 0.5f;
    
    // Static wander target that persists between calls for smoother wandering
    static ofVec3f wanderTarget = ofVec3f(0, 0, 0);
    
    // Update wander target with some randomness
    wanderTarget += ofVec3f(
        ofRandom(-1.0f, 1.0f) * wanderJitter,
        ofRandom(-1.0f, 1.0f) * wanderJitter,
        ofRandom(-1.0f, 1.0f) * wanderJitter
    );
    
    // Constrain to radius
    wanderTarget.normalize();
    wanderTarget *= wanderRadius;
    
    // Calculate wander force
    ofVec3f localTarget = wanderTarget + ofVec3f(wanderDistance, 0, 0);
    
    // Convert to world space
    ofVec3f worldTarget;
    if (velocity.length() > 0.1f) {
        // Create a matrix based on boid's orientation
        ofVec3f forward = velocity.getNormalized();
        ofVec3f right = ofVec3f(0, 1, 0).getCrossed(forward).getNormalized();
        ofVec3f up = forward.getCrossed(right);
        
        worldTarget.x = position.x + localTarget.x * forward.x + localTarget.y * right.x + localTarget.z * up.x;
        worldTarget.y = position.y + localTarget.x * forward.y + localTarget.y * right.y + localTarget.z * up.y;
        worldTarget.z = position.z + localTarget.x * forward.z + localTarget.y * right.z + localTarget.z * up.z;
    } else {
        // If very slow/stopped, just use a random direction
        worldTarget = position + ofVec3f(ofRandom(-1, 1), ofRandom(-1, 1), ofRandom(-1, 1));
    }
    
    // Seek the world target position
    ofVec3f desiredVelocity = worldTarget - position;
    desiredVelocity.normalize();
    desiredVelocity *= maxSpeed;
    
    ofVec3f steeringForce = desiredVelocity - velocity;
    if (steeringForce.length() > maxForce * 0.5f) {
        steeringForce.normalize();
        steeringForce *= maxForce * 0.5f; // Reduce force for wandering
    }
    
    // Store for debugging visualization
    wanderForce = steeringForce;
    return steeringForce;
}

ofVec3f Boid::seek(const ofVec3f& target) {
    // Calculate desired velocity
    ofVec3f desiredVelocity = target - position;
    
    // If very close to target, reduce speed
    float distance = desiredVelocity.length();
    if (distance < 5.0f) {
        float speed = ofMap(distance, 0, 5.0f, 0, maxSpeed);
        desiredVelocity.normalize();
        desiredVelocity *= speed;
    } else {
        desiredVelocity.normalize();
        desiredVelocity *= maxSpeed;
    }
    
    // Steering force = desired velocity - current velocity
    ofVec3f steeringForce = desiredVelocity - velocity;
    
    // Limit maximum steering force
    if (steeringForce.length() > maxForce) {
        steeringForce.normalize();
        steeringForce *= maxForce;
    }
    
    // Store for debugging visualization
    seekForce = steeringForce;
    return steeringForce;
}

ofVec3f Boid::flee(const ofVec3f& target) {
    // Only flee if the target is within a certain radius
    float fleeRadius = neighborhoodRadius * 2.0f;
    float distance = (position - target).length();
    
    if (distance < fleeRadius) {
        // Calculate desired velocity away from target
        ofVec3f desiredVelocity = position - target;
        desiredVelocity.normalize();
        
        // Scale by max speed and add urgency for closer threats
        float urgency = 1.0f - (distance / fleeRadius); // 0-1 scale, higher when closer
        desiredVelocity *= maxSpeed * (1.0f + urgency * 0.5f); // Up to 50% speed boost
        
        // Steering force = desired velocity - current velocity
        ofVec3f steeringForce = desiredVelocity - velocity;
        
        // Limit maximum steering force but allow stronger reactions when very close
        float maxFleeForce = maxForce * (1.0f + urgency);
        if (steeringForce.length() > maxFleeForce) {
            steeringForce.normalize();
            steeringForce *= maxFleeForce;
        }
        
        return steeringForce;
    }
    
    return ofVec3f(0, 0, 0);
}

void Boid::applyForce(ofVec3f force) {
    // Apply force to acceleration based on mass (size)
    acceleration += force / size;
}

bool Boid::isHungry() const {
    return lifecycle.hungerLevel > 0.6f;
}

ofVec3f Boid::huntAsPredator(const std::vector<Boid*>& potentialPrey, TerrainSystem* terrain) {
    // Only hunt if not on cooldown and predator level is higher than prey
    if (attackCooldown > 0 || trophicLevel == TrophicLevel::PREY) {
        return ofVec3f(0, 0, 0);
    }
    
    ofVec3f huntingForce = ofVec3f(0, 0, 0);
    float closestDistance = neighborhoodRadius * 3; // Extended hunting radius
    Boid* bestTarget = nullptr;
    
    // Check current target first
    if (currentTarget && currentTarget->lifecycle.isAlive) {
        float d = (currentTarget->position - position).length();
        // Continue pursuing current target if it's alive and within range
        if (d < closestDistance) {
            // Apply terrain visibility modifiers
            float visibilityFactor = terrain->getVisibilityModifier(currentTarget->position);
            if (ofRandom(1.0) < visibilityFactor) {
                closestDistance = d;
                bestTarget = currentTarget;
            }
        } else {
            // Target moved too far away, forget it
            currentTarget = nullptr;
        }
    }
    
    // Look for better targets
    for (auto* prey : potentialPrey) {
        // Skip if not alive, same species or not a valid prey level
        if (!prey->lifecycle.isAlive || prey->speciesName == speciesName || 
            (int)prey->trophicLevel >= (int)trophicLevel) {
            continue;
        }
        
        float d = (prey->position - position).length();
        if (d < closestDistance && isInFieldOfView(prey)) {
            // Apply terrain visibility factors
            float visibilityFactor = terrain->getVisibilityModifier(prey->position);
            
            // Stealth check - harder to see stealthy prey
            float detectionChance = visibilityFactor * (1.0f - prey->genetics.stealth * 0.5f);
            detectionChance *= genetics.perception; // Better perception improves detection
            
            if (ofRandom(1.0) < detectionChance) {
                closestDistance = d;
                bestTarget = prey;
            }
        }
    }
    
    // Set current target and change state to hunting
    if (bestTarget) {
        currentTarget = bestTarget;
        behaviorState = BehaviorState::HUNTING;
        
        // Calculate hunting force (seek target)
        ofVec3f targetOffset = currentTarget->position - position;
        
        // Apply terrain speed modifier
        float speedMod = terrain->getSpeedModifier(position, movementType);
        
        huntingForce = targetOffset.getNormalized() * maxForce * speedMod * genetics.speed;
    } else {
        // No target found, return to normal behavior
        behaviorState = BehaviorState::NORMAL;
    }
    
    return huntingForce;
}

bool Boid::attackTarget(Boid* target) {
    // Check if target exists and is alive
    if (!target || !target->lifecycle.isAlive) {
        currentTarget = nullptr;
        behaviorState = BehaviorState::NORMAL;
        return false;
    }
    
    float attackRange = size * 2.0f;
    float distance = (target->position - position).length();
    
    // Only attack if within range and cooldown expired
    if (distance <= attackRange && attackCooldown <= 0) {
        // Attack success based on strength vs target's size/agility
        float attackStrength = genetics.strength;
        float defenseStrength = target->genetics.size * 0.5f + target->genetics.speed * 0.5f;
        
        // Calculate damage amount
        float baseDamage = ofRandom(0.1f, 0.3f);
        float damageMultiplier = attackStrength / defenseStrength;
        float finalDamage = baseDamage * damageMultiplier;
        
        // Deliver damage
        bool killed = target->lifecycle.takeDamage(finalDamage);
        
        // Reset cooldown and notify target
        attackCooldown = 2.0f;  // 2 second cooldown
        lastAttackTime = ofGetElapsedTimef();
        
        // Trigger target's defensive response
        target->respondToAttack(this);
        
        // Feed if killed prey
        if (killed) {
            float nutritionValue = target->lifecycle.energy * 0.5f;
            lifecycle.feed(nutritionValue);
            currentTarget = nullptr;
            behaviorState = BehaviorState::FEEDING; // Brief feeding state
            return true;
        }
        
        return true;
    }
    
    return false;
}

void Boid::respondToAttack(Boid* attacker) {
    // Set threat status
    currentThreat = attacker;
    lastDefenseTime = ofGetElapsedTimef();
    
    // Increase stress level
    lifecycle.stress += 0.3f;
    lifecycle.stress = ofClamp(lifecycle.stress, 0.0f, 1.0f);
    
    // Change behavior state
    behaviorState = BehaviorState::FLEEING;
    
    // Choose best defense tactic
    chooseBestDefenseTactic(attacker);
}

void Boid::chooseBestDefenseTactic(Boid* attacker) {
    // Default to fleeing
    defenseTactic = DefenseTactic::FLEE;
    
    // Calculate threat level (0-1)
    float threatLevel = 0.0f;
    if (attacker) {
        float strengthRatio = attacker->genetics.strength / genetics.strength;
        float sizeRatio = attacker->size / size;
        threatLevel = (strengthRatio + sizeRatio) * 0.5f;
        threatLevel = ofClamp(threatLevel, 0.2f, 1.0f);
    }
    
    // Store last safe position for potential return
    lastKnownSafePosition = position;
    
    // Choose tactic based on genetics, health, and threat level
    float healthFactor = lifecycle.health;
    float energyFactor = lifecycle.energy / lifecycle.maxEnergy;
    float geneticAggressiveness = genetics.aggressiveness;
    
    // Calculate probabilities for each tactic
    float fleeProb = (1.0f - geneticAggressiveness) * threatLevel;
    float hideProb = genetics.stealth * 0.5f;
    float groupProb = 0.2f; // Base probability for grouping
    float fightProb = geneticAggressiveness * (1.0f - threatLevel) * healthFactor;
    
    // Adjust based on current state
    if (healthFactor < 0.3f) fleeProb += 0.4f; // Low health increases flee probability
    if (energyFactor < 0.3f) fightProb -= 0.3f; // Low energy decreases fight probability
    
    // Normalize probabilities
    float totalProb = fleeProb + hideProb + groupProb + fightProb;
    fleeProb /= totalProb;
    hideProb /= totalProb;
    groupProb /= totalProb;
    fightProb /= totalProb;
    
    // Choose tactic based on probabilities
    float r = ofRandom(1.0f);
    if (r < fleeProb) {
        defenseTactic = DefenseTactic::FLEE;
    } else if (r < fleeProb + hideProb) {
        defenseTactic = DefenseTactic::HIDE;
    } else if (r < fleeProb + hideProb + groupProb) {
        defenseTactic = DefenseTactic::GROUP_DEFENSE;
    } else {
        defenseTactic = DefenseTactic::FIGHT_BACK;
    }
}

ofVec3f Boid::executeDefenseTactic(TerrainSystem* terrain) {
    // Execute the current defense tactic
    switch (defenseTactic) {
        case DefenseTactic::FLEE:
            return executeFleeBehavior();
            
        case DefenseTactic::HIDE:
            return executeHideBehavior(terrain);
            
        case DefenseTactic::GROUP_DEFENSE:
            return executeGroupDefenseBehavior();
            
        case DefenseTactic::FIGHT_BACK:
            return executeFightBehavior();
            
        default:
            return executeFleeBehavior(); // Default to flee
    }
}

ofVec3f Boid::executeFleeBehavior() {
    // Simple flee behavior - move directly away from threat
    if (!currentThreat) return ofVec3f(0, 0, 0);
    
    // Calculate direction away from threat
    ofVec3f awayVector = position - currentThreat->position;
    
    // If no direction, choose random
    if (awayVector.length() < 0.001f) {
        awayVector = ofVec3f(ofRandom(-1, 1), ofRandom(-1, 1), ofRandom(-1, 1));
    }
    
    // Normalize and scale by maximum force and speed boost
    awayVector.normalize();
    awayVector *= maxForce * 1.5f; // Extra force when fleeing
    
    // Add some randomness to prevent predictable movement
    awayVector += ofVec3f(ofRandom(-0.2f, 0.2f), ofRandom(-0.2f, 0.2f), ofRandom(-0.2f, 0.2f));
    
    // After fleeing for a while, return to normal
    float fleeTime = ofGetElapsedTimef() - lastDefenseTime;
    if (fleeTime > 5.0f) { // After 5 seconds
        behaviorState = BehaviorState::NORMAL;
        currentThreat = nullptr;
    }
    
    return awayVector;
}

ofVec3f Boid::executeHideBehavior(TerrainSystem* terrain) {
    // Find cover and hide
    ofVec3f coverPos = findNearestCover(terrain);
    
    // If cover found, move towards it
    if (coverPos != position) {
        // Calculate path to cover
        ofVec3f toCover = coverPos - position;
        float distToCover = toCover.length();
        
        // If reached cover, stay hidden
        if (distToCover < 1.0f) {
            // Increase effectiveness of hiding
            hideEffectiveness = terrain->getVegetationDensity(position);
            
            // Add some small movement to prevent getting stuck
            return ofVec3f(ofRandom(-0.1f, 0.1f), ofRandom(-0.1f, 0.1f), ofRandom(-0.1f, 0.1f));
        }
        
        // Move towards cover
        toCover.normalize();
        return toCover * maxForce;
    }
    
    // No cover found, default to flee
    return executeFleeBehavior();
}

ofVec3f Boid::executeGroupDefenseBehavior() {
    // Move towards other members of the same species
    ofVec3f flockCenter = findFlockCenter(neighborhoodRadius * 2.0f);
    
    // If flock center exists, move towards it
    if (flockCenter != position) {
        ofVec3f toCenter = flockCenter - position;
        toCenter.normalize();
        return toCenter * maxForce;
    }
    
    // No flockmates found, default to flee
    return executeFleeBehavior();
}

ofVec3f Boid::executeFightBehavior() {
    // Fight back against the attacker
    if (!currentThreat) return ofVec3f(0, 0, 0);
    
    // Move towards the attacker
    ofVec3f toThreat = currentThreat->position - position;
    float distToThreat = toThreat.length();
    
    // If within attack range, perform counterattack
    if (distToThreat < size * 2.0f && attackCooldown <= 0) {
        attackTarget(currentThreat);
        
        // After counterattack, switch to flee
        defenseTactic = DefenseTactic::FLEE;
        return executeFleeBehavior();
    }
    
    // Move towards the attacker to get within attack range
    toThreat.normalize();
    return toThreat * maxForce;
}

ofVec3f Boid::competitionBehavior(const std::vector<Boid*>& otherPredators) {
    // Competition with other predators for the same prey
    if (!currentTarget || otherPredators.empty()) {
        return ofVec3f(0, 0, 0);
    }
    
    ofVec3f competitionForce = ofVec3f(0, 0, 0);
    int competitors = 0;
    
    // Check for other predators targeting the same prey
    for (auto* other : otherPredators) {
        // Skip self or non-predators
        if (other == this || other->trophicLevel == TrophicLevel::PREY) {
            continue;
        }
        
        // Check if targeting same prey
        if (other->currentTarget == currentTarget) {
            // Calculate strength comparison
            float strengthRatio = genetics.strength / other->genetics.strength;
            
            if (strengthRatio > 1.1f) {
                // We're stronger - move towards competitor to assert dominance
                ofVec3f toCompetitor = other->position - position;
                competitionForce += toCompetitor.getNormalized() * 0.5f;
            } else if (strengthRatio < 0.9f) {
                // We're weaker - yield and find another target
                currentTarget = nullptr;
                behaviorState = BehaviorState::NORMAL;
                return ofVec3f(0, 0, 0);
            } else {
                // Similar strength - maintain distance
                ofVec3f fromCompetitor = position - other->position;
                competitionForce += fromCompetitor.getNormalized() * 0.3f;
            }
            
            competitors++;
        }
    }
    
    // Scale force based on number of competitors
    if (competitors > 0) {
        competitionForce /= competitors;
        return competitionForce * maxForce;
    }
    
    return ofVec3f(0, 0, 0);
}

ofVec3f Boid::findNearestCover(TerrainSystem* terrain) {
    // Use terrain system to find cover
    return terrain->findNearestCover(position, neighborhoodRadius * 2.0f);
}

float Boid::findDistanceToNearestCover(TerrainSystem* terrain) {
    ofVec3f coverPos = findNearestCover(terrain);
    return (coverPos - position).length();
}

ofVec3f Boid::findFlockCenter(float radius) {
    // Placeholder for future implementation
    // This would use Octree or spatial partitioning from FlockSystem
    return position; // Default to current position for now
}

int Boid::countNearbyFlockmates(float radius) {
    // Placeholder for future implementation
    // This would use Octree or spatial partitioning from FlockSystem
    return 0; // Default to 0 for now
}

ofVec3f Boid::seekFood(const std::vector<FoodSource*>& foodSources) {
    // Only seek food if hungry
    if (!isHungry()) {
        return ofVec3f(0, 0, 0);
    }
    
    // Find closest appropriate food source
    float closestDist = FLT_MAX;
    FoodSource* bestFood = nullptr;
    
    for (auto* food : foodSources) {
        // Skip if not compatible with trophic level
        if ((trophicLevel == TrophicLevel::PREY && food->type != FoodSourceType::PLANT) ||
            ((trophicLevel == TrophicLevel::MID_PREDATOR || trophicLevel == TrophicLevel::APEX_PREDATOR) && 
             food->type != FoodSourceType::CARRION)) {
            continue;
        }
        
        // Check if food has nutrition
        if (food->nutritionalValue <= 0) continue;
        
        // Calculate distance
        float dist = (food->position - position).length();
        if (dist < closestDist && dist < neighborhoodRadius * 2.0f) {
            closestDist = dist;
            bestFood = food;
        }
    }
    
    // If food found, change behavior and return force
    if (bestFood) {
        currentFood = bestFood;
        behaviorState = BehaviorState::FEEDING;
        
        // If close enough, consume food
        if (closestDist < size * 1.5f) {
            float nutrition = currentFood->consume(this);
            lifecycle.feed(nutrition);
            
            // Return to normal if satiated
            if (!isHungry()) {
                behaviorState = BehaviorState::NORMAL;
                currentFood = nullptr;
            }
            
            // Stay in place while eating
            return ofVec3f(0, 0, 0);
        }
        
        // Move towards food
        ofVec3f toFood = bestFood->position - position;
        toFood.normalize();
        return toFood * maxForce;
    }
    
    return ofVec3f(0, 0, 0);
}

void Boid::updateParameters() {
    // Adjust parameters based on current state and trophic level
    
    // Adjust speed based on trophic level and current behavior
    if (trophicLevel == TrophicLevel::APEX_PREDATOR) {
        // Predators move slower when patrolling but faster when hunting
        if (behaviorState == BehaviorState::HUNTING) {
            maxSpeed = maxSpeed * 1.5f; // Faster when pursuing prey
        } else {
            maxSpeed = maxSpeed * 0.7f; // Slower during normal movement
        }
    } else if (trophicLevel == TrophicLevel::PREY) {
        // Prey move at normal speed when normal, faster when fleeing
        if (behaviorState == BehaviorState::FLEEING) {
            maxSpeed = maxSpeed * 1.3f; // Faster when fleeing
        } else if (behaviorState == BehaviorState::FEEDING) {
            maxSpeed = minSpeed; // Very slow when feeding
        }
    }
}

