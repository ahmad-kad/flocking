#include "FlockSystem.h"

FlockSystem::FlockSystem() {
    // Initialize spatial grid
    gridResolution = 10;
    gridMin = ofVec3f(-20, -20, -20);
    gridMax = ofVec3f(20, 20, 20);
    
    // Calculate cell size based on grid dimensions
    float xSize = (gridMax.x - gridMin.x) / gridResolution;
    float ySize = (gridMax.y - gridMin.y) / gridResolution;
    float zSize = (gridMax.z - gridMin.z) / gridResolution;
    cellSize = min(min(xSize, ySize), zSize);
    
    // Initialize 3D grid
    grid.resize(gridResolution);
    for (int x = 0; x < gridResolution; x++) {
        grid[x].resize(gridResolution);
        for (int y = 0; y < gridResolution; y++) {
            grid[x][y].resize(gridResolution);
        }
    }
    
    // Set default bounds
    boundsMin = gridMin;
    boundsMax = gridMax;
    
    // Initialize boundary parameters
    boundaryForceWeight = 1.0;
    boundaryDistance = 2.0;
    
    // Initialize target seeking
    hasTarget = false;
    targetWeight = 0.5;
    
    // Initialize global parameters
    flockMode = NORMAL;
    individualismFactor = 0.5;
    systemChaos = 0.0;
    boidsVariability = 0.5;
    
    // Initialize color-based flocking
    colorBasedFlockingEnabled = false;
    colorInfluenceFactor = 0.5;
    colorSimilarityThreshold = 0.3;
    
    // Initialize debug visualization
    showDebug = false;
    showVelocities = false;
    showNeighborhoods = false;
    showForces = false;
    showGrid = false;
}

FlockSystem::~FlockSystem() {
    // Delete all boids
    for (int i = 0; i < boids.size(); i++) {
        delete boids[i];
    }
    boids.clear();
}

void FlockSystem::update() {
    // Update spatial grid for efficient neighbor search
    updateSpatialGrid();
    
    // Apply global system chaos if enabled
    ofVec3f chaosForce;
    if (systemChaos > 0) {
        // Create a perlin noise-based turbulence field
        float time = ofGetElapsedTimef() * 0.2;
        
        for (int i = 0; i < boids.size(); i++) {
            // Scale chaos based on global setting and boid's energy level
            float chaos = systemChaos * boids[i]->energyLevel;
            
            // Generate unique turbulence for each boid using noise
            float noiseX = ofSignedNoise(boids[i]->position.x * 0.1, boids[i]->position.y * 0.1, time);
            float noiseY = ofSignedNoise(boids[i]->position.y * 0.1, boids[i]->position.z * 0.1, time);
            float noiseZ = ofSignedNoise(boids[i]->position.z * 0.1, boids[i]->position.x * 0.1, time);
            
            chaosForce = ofVec3f(noiseX, noiseY, noiseZ) * chaos;
            boids[i]->applyForce(chaosForce);
        }
    }
    
    // Apply flocking behavior based on the current mode
    for (int i = 0; i < boids.size(); i++) {
        // Get neighbors for this boid
        vector<Boid*> neighbors = getNeighbors(boids[i], boids[i]->neighborhoodRadius);
        
        // If color-based flocking is enabled, filter neighbors by color similarity
        if (colorBasedFlockingEnabled && !neighbors.empty()) {
            vector<Boid*> colorFiltered;
            vector<float> colorWeights;
            
            // Calculate color similarity for each neighbor
            for (auto neighbor : neighbors) {
                float similarity = getColorSimilarity(boids[i], neighbor);
                
                // Use similarity as a weight factor - only include neighbors above threshold
                if (similarity > colorSimilarityThreshold) {
                    colorFiltered.push_back(neighbor);
                    colorWeights.push_back(similarity);
                }
            }
            
            // If we have color-similar neighbors, use those primarily
            if (!colorFiltered.empty()) {
                // Blend normal behavior with color-based behavior
                // This allows boids to still flock normally but prioritize similar colors
                
                // First, get normal flocking forces
                ofVec3f sepForce = boids[i]->separate(neighbors);
                ofVec3f aliForce = boids[i]->align(neighbors);
                ofVec3f cohForce = boids[i]->cohere(neighbors);
                
                // Then, get color-based flocking forces
                ofVec3f colorSepForce, colorAliForce, colorCohForce;
                if (colorFiltered.size() > 0) {
                    colorSepForce = boids[i]->separate(colorFiltered);
                    colorAliForce = boids[i]->align(colorFiltered);
                    colorCohForce = boids[i]->cohere(colorFiltered);
                }
                
                // Blend forces based on color influence factor
                float normalWeight = 1.0 - colorInfluenceFactor;
                float colorWeight = colorInfluenceFactor;
                
                boids[i]->separationForce = sepForce * normalWeight + colorSepForce * colorWeight;
                boids[i]->alignmentForce = aliForce * normalWeight + colorAliForce * colorWeight;
                boids[i]->cohesionForce = cohForce * normalWeight + colorCohForce * colorWeight;
                
                // Apply blended forces directly
                boids[i]->applyForce(boids[i]->separationForce * boids[i]->separationWeight);
                boids[i]->applyForce(boids[i]->alignmentForce * boids[i]->alignmentWeight);
                boids[i]->applyForce(boids[i]->cohesionForce * boids[i]->cohesionWeight);
                
                // Skip the normal flocking call since we've manually applied the forces
                goto skipNormalFlocking;
            }
        }
        
        // Set boid's individuality based on global setting and individual uniqueness
        boids[i]->uniqueness = ofClamp(boids[i]->uniqueness * individualismFactor, 0.1, 0.9);
        
        // Adjust flocking weights based on flock mode
        switch (flockMode) {
            case NORMAL:
                // Standard weights (already set)
                break;
                
            case SCATTERED:
                // Higher separation, lower cohesion
                boids[i]->separationWeight *= 2.0;
                boids[i]->cohesionWeight *= 0.5;
                break;
                
            case TIGHT:
                // Lower separation, higher cohesion
                boids[i]->separationWeight *= 0.5;
                boids[i]->cohesionWeight *= 2.0;
                break;
                
            case PREDATOR_PREY:
                // Some boids chase others
                if (boids[i]->energyLevel > 0.7) {
                    // Predators chase
                    boids[i]->separationWeight *= 0.3;
                    boids[i]->maxSpeed *= 1.2;
                }
                else {
                    // Prey flee
                    boids[i]->separationWeight *= 2.0;
                    boids[i]->alignmentWeight *= 1.5;
                }
                break;
        }
        
        // Apply normal flocking behavior
        boids[i]->flock(neighbors);
        
    skipNormalFlocking:
        
        // Apply boundary force
        ofVec3f boundary = boundaryForce(boids[i]);
        boids[i]->boundaryForce = boundary;
        boids[i]->applyForce(boundary * boundaryForceWeight);
        
        // Apply target seeking if enabled
        if (hasTarget) {
            ofVec3f targetForce = boids[i]->seek(target);
            boids[i]->seekForce = targetForce;
            boids[i]->applyForce(targetForce * targetWeight);
        }
        
        // Apply random wandering based on boid's uniqueness
        ofVec3f wanderForce = boids[i]->wander() * boids[i]->uniqueness;
        boids[i]->wanderForce = wanderForce;
        boids[i]->applyForce(wanderForce);
        
        // Update particle physics
        boids[i]->integrate();
        
        // Reset forces for the next frame
        boids[i]->forces = ofVec3f(0, 0, 0);
    }
}

void FlockSystem::draw(ofMesh* customMesh) {
    // Draw all boids
    for (int i = 0; i < boids.size(); i++) {
        if (customMesh) {
            // Use custom mesh for rendering
            ofPushMatrix();
            
            // Move to boid position
            ofTranslate(boids[i]->position);
            
            // Calculate rotation to align with movement direction
            ofVec3f direction = boids[i]->velocity;
            if (direction.length() > 0) {
                direction.normalize();
                
                // Calculate rotation to align with velocity
                ofQuaternion rotation;
                ofVec3f axis(0, 1, 0);
                rotation.makeRotate(axis, direction);
                
                // Apply rotation
                ofMatrix4x4 mat;
                rotation.get(mat);
                glMultMatrixf(mat.getPtr());
            }
            
            // Draw the custom mesh
            ofSetColor(boids[i]->color);
            customMesh->draw();
            
            ofPopMatrix();
        } else {
            // Use default boid drawing method
            boids[i]->draw();
        }
        
        // Draw debug visualizations if enabled
        if (showDebug) {
            boids[i]->drawDebug(showVelocities, showNeighborhoods, showForces);
        }
    }
    
    // Draw spatial grid for debugging
    if (showDebug && showGrid) {
        drawGrid();
    }
    
    // Draw target if enabled
    if (hasTarget) {
        ofPushStyle();
        ofSetColor(ofColor::red);
        ofDrawSphere(target, 0.3);
        ofPopStyle();
    }
}

void FlockSystem::drawGrid() {
    ofPushStyle();
    ofSetColor(ofColor(100, 100, 100, 50));
    
    // Draw grid outline
    ofNoFill();
    ofDrawBox(gridMin + (gridMax - gridMin) * 0.5, (gridMax - gridMin).x, (gridMax - gridMin).y, (gridMax - gridMin).z);
    
    // Draw grid cells with boids
    ofSetColor(ofColor(150, 150, 255, 40));
    for (int x = 0; x < gridResolution; x++) {
        for (int y = 0; y < gridResolution; y++) {
            for (int z = 0; z < gridResolution; z++) {
                if (grid[x][y][z].boids.size() > 0) {
                    // Calculate cell position
                    float xPos = ofMap(x, 0, gridResolution, gridMin.x, gridMax.x) + cellSize/2;
                    float yPos = ofMap(y, 0, gridResolution, gridMin.y, gridMax.y) + cellSize/2;
                    float zPos = ofMap(z, 0, gridResolution, gridMin.z, gridMax.z) + cellSize/2;
                    
                    // Draw cell
                    ofDrawBox(xPos, yPos, zPos, cellSize, cellSize, cellSize);
                }
            }
        }
    }
    
    ofPopStyle();
}

void FlockSystem::addBoid(Boid* boid) {
    boids.push_back(boid);
    
    // Apply variability to new boid
    boid->uniqueness = ofRandom(0.1, 0.9) * boidsVariability;
    boid->size = ofRandom(0.8, 1.2);
    boid->energyLevel = ofRandom(0.7, 1.3);
}

void FlockSystem::addBoids(int count, ofVec3f position, float spreadRadius) {
    for (int i = 0; i < count; i++) {
        Boid* boid = new Boid();
        
        // Set random position within spread radius
        ofVec3f randomOffset = ofVec3f(
            ofRandom(-spreadRadius, spreadRadius),
            ofRandom(-spreadRadius, spreadRadius),
            ofRandom(-spreadRadius, spreadRadius)
        );
        boid->position = position + randomOffset;
        
        // Set random initial velocity
        boid->velocity = ofVec3f(
            ofRandom(-1, 1),
            ofRandom(-1, 1),
            ofRandom(-1, 1)
        );
        boid->velocity.normalize();
        boid->velocity *= ofRandom(boid->minSpeed, boid->maxSpeed);
        
        // Add to flock
        addBoid(boid);
    }
}

vector<Boid*> FlockSystem::getNeighbors(Boid* boid, float radius) {
    vector<Boid*> neighbors;
    
    // Convert boid position to grid coordinates
    ofVec3f gridPos = worldToGrid(boid->position);
    int gx = gridPos.x;
    int gy = gridPos.y;
    int gz = gridPos.z;
    
    // Calculate cell search radius based on neighbor radius
    int cellRadius = ceil(radius / cellSize);
    
    // Search neighboring cells
    for (int x = max(0, gx - cellRadius); x <= min(gridResolution - 1, gx + cellRadius); x++) {
        for (int y = max(0, gy - cellRadius); y <= min(gridResolution - 1, gy + cellRadius); y++) {
            for (int z = max(0, gz - cellRadius); z <= min(gridResolution - 1, gz + cellRadius); z++) {
                Cell* cell = getCell(x, y, z);
                if (cell) {
                    // Check all boids in this cell
                    for (int i = 0; i < cell->boids.size(); i++) {
                        Boid* other = cell->boids[i];
                        
                        // Don't add self to neighbors
                        if (other != boid) {
                            // Check actual distance
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
    // Clear all cells
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
        
        // Make sure coordinates are within grid bounds
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
    
    // Apply to existing boids
    for (int i = 0; i < boids.size(); i++) {
        // Adjust uniqueness but keep some of the original characteristics
        float original = boids[i]->uniqueness;
        boids[i]->uniqueness = original * 0.5 + ofRandom(0.1, 0.9) * variability * 0.5;
    }
}

void FlockSystem::setBounds(ofVec3f min, ofVec3f max) {
    boundsMin = min;
    boundsMax = max;
}

ofVec3f FlockSystem::boundaryForce(Boid* boid) {
    ofVec3f force(0, 0, 0);
    
    // X-axis boundary
    if (boid->position.x < boundsMin.x + boundaryDistance) {
        force.x = boundaryDistance - (boid->position.x - boundsMin.x);
    } 
    else if (boid->position.x > boundsMax.x - boundaryDistance) {
        force.x = (boundsMax.x - boundaryDistance) - boid->position.x;
    }
    
    // Y-axis boundary
    if (boid->position.y < boundsMin.y + boundaryDistance) {
        force.y = boundaryDistance - (boid->position.y - boundsMin.y);
    } 
    else if (boid->position.y > boundsMax.y - boundaryDistance) {
        force.y = (boundsMax.y - boundaryDistance) - boid->position.y;
    }
    
    // Z-axis boundary
    if (boid->position.z < boundsMin.z + boundaryDistance) {
        force.z = boundaryDistance - (boid->position.z - boundsMin.z);
    } 
    else if (boid->position.z > boundsMax.z - boundaryDistance) {
        force.z = (boundsMax.z - boundaryDistance) - boid->position.z;
    }
    
    return force;
}

void FlockSystem::setTarget(ofVec3f targetPos) {
    target = targetPos;
    hasTarget = true;
}

ofVec3f FlockSystem::worldToGrid(ofVec3f position) {
    // Map world position to grid coordinates
    int x = (int)((position.x - gridMin.x) / (gridMax.x - gridMin.x) * gridResolution);
    int y = (int)((position.y - gridMin.y) / (gridMax.y - gridMin.y) * gridResolution);
    int z = (int)((position.z - gridMin.z) / (gridMax.z - gridMin.z) * gridResolution);
    
    // Clamp to valid grid range
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
    // Find the boid in the vector
    auto it = std::find(boids.begin(), boids.end(), boid);
    if (it != boids.end()) {
        // Remove from vector
        boids.erase(it);
        
        // Delete the boid
        delete boid;
    }
}

void FlockSystem::clear() {
    // Delete all boids
    for (int i = 0; i < boids.size(); i++) {
        delete boids[i];
    }
    boids.clear();
}

void FlockSystem::setColorBasedFlocking(bool enabled, float influence, float threshold) {
    colorBasedFlockingEnabled = enabled;
    colorInfluenceFactor = ofClamp(influence, 0, 1.0);
    colorSimilarityThreshold = ofClamp(threshold, 0, 1.0);
}

float FlockSystem::getColorSimilarity(Boid* boid1, Boid* boid2) {
    // Calculate color similarity based on RGB distance
    ofColor color1 = boid1->personalColor;
    ofColor color2 = boid2->personalColor;
    
    // Calculate distance in RGB space
    float rDiff = (color1.r - color2.r) / 255.0;
    float gDiff = (color1.g - color2.g) / 255.0;
    float bDiff = (color1.b - color2.b) / 255.0;
    
    // Euclidean distance in RGB space, normalized to 0-1
    float distance = sqrt(rDiff*rDiff + gDiff*gDiff + bDiff*bDiff) / sqrt(3.0);
    
    // Convert distance to similarity (1.0 = identical, 0.0 = completely different)
    float similarity = 1.0 - distance;
    
    return similarity;
} 