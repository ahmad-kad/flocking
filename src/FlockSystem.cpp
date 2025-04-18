#include "FlockSystem.h"

FlockSystem::FlockSystem() {
    // Set up spatial grid
    gridResolution = 10;
    gridMin = ofVec3f(-20, -20, -20);
    gridMax = ofVec3f(20, 20, 20);
    
    // Calculate cell size
    float xSize = (gridMax.x - gridMin.x) / gridResolution;
    float ySize = (gridMax.y - gridMin.y) / gridResolution;
    float zSize = (gridMax.z - gridMin.z) / gridResolution;
    cellSize = min(min(xSize, ySize), zSize);
    
    // Create 3D grid
    grid.resize(gridResolution);
    for (int x = 0; x < gridResolution; x++) {
        grid[x].resize(gridResolution);
        for (int y = 0; y < gridResolution; y++) {
            grid[x][y].resize(gridResolution);
        }
    }
    
    // Set bounds
    boundsMin = gridMin;
    boundsMax = gridMax;
    
    // Set boundary parameters
    boundaryForceWeight = 1.0;
    boundaryDistance = 5.0;
    
    // Set target seeking
    hasTarget = false;
    targetWeight = 0.5;
    
    // Set global parameters
    flockMode = NORMAL;
    individualismFactor = 0.5;
    systemChaos = 0.0;
    boidsVariability = 0.5;
    
    // Set color-based flocking
    colorBasedFlockingEnabled = false;
    colorInfluenceFactor = 0.5;
    colorSimilarityThreshold = 0.3;
    
    // Set debug visualization
    showDebug = false;
    showVelocities = false;
    showNeighborhoods = false;
    showForces = false;
    showGrid = false;
}

FlockSystem::~FlockSystem() {
    // Remove all boids
    for (int i = 0; i < boids.size(); i++) {
        delete boids[i];
    }
    boids.clear();
}



void FlockSystem::draw(ofMesh* customMesh) {
    // Render all boids
    for (auto boid : boids) {
        if (customMesh != nullptr) {
            boid->draw(customMesh);
        } else {
            boid->draw();
        }
        
        if (showDebug) {
            boid->drawDebug(showVelocities, showNeighborhoods, showForces);
        }
    }
    
    // Draw boundaries
    drawBoundaries();
}

void FlockSystem::update() {
    // Refresh spatial grid
    updateSpatialGrid();
    
    // Apply system chaos if active
    if (systemChaos > 0) {
        float time = ofGetElapsedTimef() * 0.1;
        
        for (int i = 0; i < boids.size(); i++) {
            // Scale chaos based on settings but reduce intensity for smoother motion
            float chaos = systemChaos * boids[i]->energyLevel * 0.3;
            
            // Generate turbulence for each boid with slower variation
            float noiseX = ofSignedNoise(boids[i]->position.x * 0.03, boids[i]->position.y * 0.03, time);
            float noiseY = ofSignedNoise(boids[i]->position.y * 0.03, boids[i]->position.z * 0.03, time);
            float noiseZ = ofSignedNoise(boids[i]->position.z * 0.03, boids[i]->position.x * 0.03, time);
            
            ofVec3f chaosForce = ofVec3f(noiseX, noiseY, noiseZ) * chaos;
            boids[i]->applyForce(chaosForce);
        }
    }
    
    // Apply flocking behavior
    for (int i = 0; i < boids.size(); i++) {
        // Get neighbors
        vector<Boid*> neighbors = getNeighbors(boids[i], boids[i]->neighborhoodRadius);
        
        // Set individuality
        boids[i]->uniqueness = ofClamp(boids[i]->uniqueness * individualismFactor, 0.1, 0.9);
        
        // Apply flocking behavior based on color if enabled
        if (colorBasedFlockingEnabled && !neighbors.empty()) {
            vector<Boid*> colorFiltered;
            
            // Filter neighbors by color similarity
            for (auto neighbor : neighbors) {
                float similarity = getColorSimilarity(boids[i], neighbor);
                if (similarity > colorSimilarityThreshold) {
                    colorFiltered.push_back(neighbor);
                }
            }
            
            // Use color-similar neighbors if available
            if (!colorFiltered.empty()) {
                // Calculate forces with regular neighbors and color-filtered neighbors
                ofVec3f sepForce = boids[i]->separate(neighbors);
                ofVec3f aliForce = boids[i]->align(neighbors);
                ofVec3f cohForce = boids[i]->cohere(neighbors);
                
                ofVec3f colorSepForce = boids[i]->separate(colorFiltered);
                ofVec3f colorAliForce = boids[i]->align(colorFiltered);
                ofVec3f colorCohForce = boids[i]->cohere(colorFiltered);
                
                // Blend forces with smooth transition
                float normalWeight = 1.0 - colorInfluenceFactor;
                float colorWeight = colorInfluenceFactor;
                
                // Smooth the forces with interpolation for reduced jitter
                float smoothFactor = 0.3; // Lower = smoother
                boids[i]->separationForce = boids[i]->separationForce.getInterpolated(
                    sepForce * normalWeight + colorSepForce * colorWeight, smoothFactor);
                boids[i]->alignmentForce = boids[i]->alignmentForce.getInterpolated(
                    aliForce * normalWeight + colorAliForce * colorWeight, smoothFactor);
                boids[i]->cohesionForce = boids[i]->cohesionForce.getInterpolated(
                    cohForce * normalWeight + colorCohForce * colorWeight, smoothFactor);
                
                // Apply blended forces
                boids[i]->applyForce(boids[i]->separationForce * boids[i]->separationWeight);
                boids[i]->applyForce(boids[i]->alignmentForce * boids[i]->alignmentWeight);
                boids[i]->applyForce(boids[i]->cohesionForce * boids[i]->cohesionWeight);
            } else {
                // Apply normal flocking with smoothing
                applySmoothedFlockingBehavior(boids[i], neighbors);
            }
        } else {
            // Apply normal flocking with smoothing
            applySmoothedFlockingBehavior(boids[i], neighbors);
        }
        
        // Apply target seeking if enabled
        if (hasTarget) {
            ofVec3f targetForce = boids[i]->seek(target);
            boids[i]->seekForce = targetForce;
            boids[i]->applyForce(targetForce * targetWeight);
        }
        
        // Apply wandering with reduced jitter
        ofVec3f newWanderForce = boids[i]->wander() * boids[i]->uniqueness * 0.7;
        boids[i]->wanderForce = boids[i]->wanderForce.getInterpolated(newWanderForce, 0.2);
        boids[i]->applyForce(boids[i]->wanderForce);
        
        // Update physics
        boids[i]->integrate();
    }
}

// Add this helper method to your FlockSystem class
void FlockSystem::adjustForcesForFlockMode(Boid* boid) {
    float separationMult = 1.0;
    float alignmentMult = 1.0;
    float cohesionMult = 1.0;
    
    switch (flockMode) {
        case SCATTERED:
            separationMult = 2.0;
            cohesionMult = 0.5;
            break;
            
        case TIGHT:
            separationMult = 0.5;
            cohesionMult = 2.0;
            break;
            
        case PREDATOR_PREY:
            if (boid->energyLevel > 0.7) {
                separationMult = 0.3;
                boid->maxSpeed *= 1.2;
            } else {
                separationMult = 2.0;
                alignmentMult = 1.5;
            }
            break;
            
        default: // NORMAL
            break;
    }
    
    // Apply multipliers
    boid->separationForce *= separationMult;
    boid->alignmentForce *= alignmentMult;
    boid->cohesionForce *= cohesionMult;
}

// Add this helper method to your FlockSystem class
void FlockSystem::applySmoothedFlockingBehavior(Boid* boid, vector<Boid*>& neighbors) {
    // Calculate new forces
    ofVec3f newSeparationForce = boid->separate(neighbors);
    ofVec3f newAlignmentForce = boid->align(neighbors);
    ofVec3f newCohesionForce = boid->cohere(neighbors);
    
    // Smooth forces with interpolation
    float smoothFactor = 0.2; // Lower = smoother transitions
    boid->separationForce = boid->separationForce.getInterpolated(newSeparationForce, smoothFactor);
    boid->alignmentForce = boid->alignmentForce.getInterpolated(newAlignmentForce, smoothFactor);
    boid->cohesionForce = boid->cohesionForce.getInterpolated(newCohesionForce, smoothFactor);
    
    // Adjust forces based on flocking mode
    adjustForcesForFlockMode(boid);
    
    // Apply forces
    boid->applyForce(boid->separationForce * boid->separationWeight);
    boid->applyForce(boid->alignmentForce * boid->alignmentWeight);
    boid->applyForce(boid->cohesionForce * boid->cohesionWeight);
}



void FlockSystem::drawGrid() {
    if (!showGrid) return;
    
    ofPushStyle();
    ofSetColor(50, 50, 50, 80);
    ofSetLineWidth(0.5);
    
    // Draw grid lines
    for (int x = 0; x <= gridResolution; x++) {
        for (int y = 0; y <= gridResolution; y++) {
            for (int z = 0; z <= gridResolution; z++) {
                float xPos = ofMap(x, 0, gridResolution, gridMin.x, gridMax.x);
                float yPos = ofMap(y, 0, gridResolution, gridMin.y, gridMax.y);
                float zPos = ofMap(z, 0, gridResolution, gridMin.z, gridMax.z);
                
                if (x < gridResolution) {
                    ofDrawLine(xPos, yPos, zPos, 
                               ofMap(x+1, 0, gridResolution, gridMin.x, gridMax.x), yPos, zPos);
                }
                if (y < gridResolution) {
                    ofDrawLine(xPos, yPos, zPos, 
                               xPos, ofMap(y+1, 0, gridResolution, gridMin.y, gridMax.y), zPos);
                }
                if (z < gridResolution) {
                    ofDrawLine(xPos, yPos, zPos, 
                               xPos, yPos, ofMap(z+1, 0, gridResolution, gridMin.z, gridMax.z));
                }
            }
        }
    }
    
    ofPopStyle();
}

void FlockSystem::drawBoundaries() {
    ofPushStyle();
    ofNoFill();
    ofSetColor(120, 120, 120, 180);
    ofSetLineWidth(1.5);
    
    // Calculate dimensions
    ofVec3f boundsDimensions = boundsMax - boundsMin;
    ofVec3f boundsCenter = boundsMin + boundsDimensions * 0.5f;
    
    // Draw wireframe box
    ofDrawBox(boundsCenter, boundsDimensions.x, boundsDimensions.y, boundsDimensions.z);
    
    ofPopStyle();
}

void FlockSystem::addBoid(Boid* boid) {
    boids.push_back(boid);
    
    // Set variability for new boid
    boid->uniqueness = ofRandom(0.1, 0.9) * boidsVariability;
    boid->size = ofRandom(0.8, 1.2);
    boid->energyLevel = ofRandom(0.7, 1.3);
}

void FlockSystem::addBoids(int count, ofVec3f position, float spreadRadius) {
    for (int i = 0; i < count; i++) {
        Boid* boid = new Boid();
        
        // Set random position
        ofVec3f randomOffset = ofVec3f(
            ofRandom(-spreadRadius, spreadRadius),
            ofRandom(-spreadRadius, spreadRadius),
            ofRandom(-spreadRadius, spreadRadius)
        );
        boid->position = position + randomOffset;
        
        // Set random velocity
        boid->velocity = ofVec3f(
            ofRandom(-0.1, 0.1),
            ofRandom(-0.1, 0.1),
            ofRandom(-0.1, 0.1)
        );
        boid->velocity.normalize();
        boid->velocity *= ofRandom(boid->minSpeed * 0.5, boid->maxSpeed * 0.5);
        
        // Add to flock
        addBoid(boid);
    }
}

vector<Boid*> FlockSystem::getNeighbors(Boid* boid, float radius) {
    vector<Boid*> neighbors;
    
    // Convert position to grid coordinates
    ofVec3f gridPos = worldToGrid(boid->position);
    int gx = gridPos.x;
    int gy = gridPos.y;
    int gz = gridPos.z;
    
    // Calculate search radius
    int cellRadius = ceil(radius / cellSize);
    
    // Search neighboring cells
    for (int x = max(0, gx - cellRadius); x <= min(gridResolution - 1, gx + cellRadius); x++) {
        for (int y = max(0, gy - cellRadius); y <= min(gridResolution - 1, gy + cellRadius); y++) {
            for (int z = max(0, gz - cellRadius); z <= min(gridResolution - 1, gz + cellRadius); z++) {
                Cell* cell = getCell(x, y, z);
                if (cell) {
                    // Check boids in this cell
                    for (int i = 0; i < cell->boids.size(); i++) {
                        Boid* other = cell->boids[i];
                        
                        // Exclude self
                        if (other != boid) {
                            // Check distance
                            float distance = boid->position.distance(other->position);
                            if (distance < radius) {
                                neighbors.push_back(other);
                            }
                        }
                    }
                }
            }
        }
    }
    
    return neighbors;
}

void FlockSystem::updateSpatialGrid() {
    // Clear cells
    for (int x = 0; x < gridResolution; x++) {
        for (int y = 0; y < gridResolution; y++) {
            for (int z = 0; z < gridResolution; z++) {
                grid[x][y][z].boids.clear();
            }
        }
    }
    
    // Add boids to cells
    for (int i = 0; i < boids.size(); i++) {
        ofVec3f gridPos = worldToGrid(boids[i]->position);
        int gx = gridPos.x;
        int gy = gridPos.y;
        int gz = gridPos.z;
        
        // Ensure coordinates are valid
        if (gx >= 0 && gx < gridResolution &&
            gy >= 0 && gy < gridResolution &&
            gz >= 0 && gz < gridResolution) {
            grid[gx][gy][gz].boids.push_back(boids[i]);
        }
    }
}

void FlockSystem::setParameters(float sepWeight, float aliWeight, float cohWeight) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->separationWeight = sepWeight;
        boids[i]->alignmentWeight = aliWeight;
        boids[i]->cohesionWeight = cohWeight;
    }
}

void FlockSystem::setMaxSpeed(float speed) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->maxSpeed = speed;
    }
}

void FlockSystem::setMinSpeed(float speed) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->minSpeed = speed;
    }
}

void FlockSystem::setMaxForce(float force) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->maxForce = force;
    }
}

void FlockSystem::setNeighborhoodRadius(float radius) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->neighborhoodRadius = radius;
    }
}

void FlockSystem::setSeparationRadius(float radius) {
    for (int i = 0; i < boids.size(); i++) {
        boids[i]->separationRadius = radius;
    }
}

void FlockSystem::setFlockMode(FlockMode mode) {
    flockMode = mode;
}

void FlockSystem::setIndividualismFactor(float factor) {
    individualismFactor = ofClamp(factor, 0, 1);
}

void FlockSystem::setSystemChaos(float chaos) {
    systemChaos = ofClamp(chaos, 0, 1);
}

void FlockSystem::setBoidsVariability(float variability) {
    boidsVariability = ofClamp(variability, 0, 1);
    
    // Adjust existing boids
    for (int i = 0; i < boids.size(); i++) {
        float original = boids[i]->uniqueness;
        boids[i]->uniqueness = original * 0.5 + ofRandom(0.1, 0.9) * variability * 0.5;
    }
}

void FlockSystem::setBounds(ofVec3f min, ofVec3f max) {
    boundsMin = min;
    boundsMax = max;
}

ofVec3f FlockSystem::boundaryForce(Boid& boid) {
    ofVec3f force = ofVec3f(0, 0, 0);
    float repulsionStrength = 2.0;
    float margin = 4.0;  // Increased margin for earlier response
    
    // Calculate force direction - toward center
    ofVec3f toCenter = (boundsMin + boundsMax) * 0.5 - boid.position;
    float distanceFromCenter = toCenter.length();
    float maxRadius = (boundsMax - boundsMin).length() * 0.5;
    
    // Calculate distance to boundaries in each direction
    float distToBottom = boid.position.y - boundsMin.y;
    float distToTop = boundsMax.y - boid.position.y;
    float distToLeft = boid.position.x - boundsMin.x;
    float distToRight = boundsMax.x - boid.position.x;
    float distToFront = boid.position.z - boundsMin.z;
    float distToBack = boundsMax.z - boid.position.z;
    
    // Find the closest boundary
    float minDist = min(min(min(distToBottom, distToTop),
                        min(distToLeft, distToRight)),
                        min(distToFront, distToBack));
    
    // Apply repulsion force that increases as boid gets closer to boundary
    if (minDist < margin) {
        // Calculate force components based on each boundary
        if (distToBottom < margin) {
            float strength = repulsionStrength * (1.0 - distToBottom/margin);
            force.y += strength;
        }
        if (distToTop < margin) {
            float strength = repulsionStrength * (1.0 - distToTop/margin);
            force.y -= strength;
        }
        if (distToLeft < margin) {
            float strength = repulsionStrength * (1.0 - distToLeft/margin);
            force.x += strength;
        }
        if (distToRight < margin) {
            float strength = repulsionStrength * (1.0 - distToRight/margin);
            force.x -= strength;
        }
        if (distToFront < margin) {
            float strength = repulsionStrength * (1.0 - distToFront/margin);
            force.z += strength;
        }
        if (distToBack < margin) {
            float strength = repulsionStrength * (1.0 - distToBack/margin);
            force.z -= strength;
        }
        
        // Smooth the application of force with a sigmoid function
        // This creates a more natural-feeling forcefield
        float normalizedDist = minDist / margin;
        float sigmoid = 1.0 / (1.0 + exp(5.0 * normalizedDist - 2.5));
        force *= sigmoid;
    }
    
    return force;
}

void FlockSystem::setTarget(ofVec3f targetPos) {
    target = targetPos;
    hasTarget = true;
}

ofVec3f FlockSystem::worldToGrid(ofVec3f position) {
    // Map position to grid coordinates
    int x = (int)((position.x - gridMin.x) / (gridMax.x - gridMin.x) * gridResolution);
    int y = (int)((position.y - gridMin.y) / (gridMax.y - gridMin.y) * gridResolution);
    int z = (int)((position.z - gridMin.z) / (gridMax.z - gridMin.z) * gridResolution);
    
    // Clamp to grid range
    x = ofClamp(x, 0, gridResolution - 1);
    y = ofClamp(y, 0, gridResolution - 1);
    z = ofClamp(z, 0, gridResolution - 1);
    
    return ofVec3f(x, y, z);
}

Cell* FlockSystem::getCell(int x, int y, int z) {
    if (x >= 0 && x < gridResolution &&
        y >= 0 && y < gridResolution &&
        z >= 0 && z < gridResolution) {
        return &grid[x][y][z];
    }
    return nullptr;
}

void FlockSystem::removeBoid(Boid* boid) {
    // Locate the boid
    auto it = std::find(boids.begin(), boids.end(), boid);
    if (it != boids.end()) {
        // Remove and delete
        boids.erase(it);
        delete boid;
    }
}

void FlockSystem::clear() {
    // Remove all boids
    for (int i = 0; i < boids.size(); i++) {
        delete boids[i];
    }
    boids.clear();
}

void FlockSystem::setColorBasedFlocking(bool enabled, float threshold, float influence) {
    colorBasedFlockingEnabled = enabled;
    colorSimilarityThreshold = threshold;
    colorInfluenceFactor = influence;
    
    // Default to 1.0 for similar colors
    if (enabled && influence == 0) {
        colorInfluenceFactor = 1.0;
    }
}

float FlockSystem::getColorSimilarity(Boid* boid1, Boid* boid2) {
    // Calculate color similarity
    float h1, s1, b1, h2, s2, b2;
    boid1->color.getHsb(h1, s1, b1);
    boid2->color.getHsb(h2, s2, b2);
    
    // Calculate hue distance
    float hueDist = fabs(h1 - h2);
    if (hueDist > 180) hueDist = 360 - hueDist;
    hueDist /= 180.0; 
    
    // Calculate saturation and brightness distances
    float satDist = fabs(s1 - s2) / 255.0;
    float briDist = fabs(b1 - b2) / 255.0;
    
    // Weighted sum for similarity
    float similarity = 1.0 - (hueDist * 0.6 + satDist * 0.2 + briDist * 0.2);
    
    return similarity;
} 
