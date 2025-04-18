#include "TerrainSystem.h"
#include "ofNoise.h"

void TerrainSystem::setup(int w, int h, float cSize) {
    width = w;
    height = h;
    cellSize = cSize;
    
    // Initialize grid
    cells.resize(width);
    for (int x = 0; x < width; x++) {
        cells[x].resize(height);
    }
}

void TerrainSystem::generateTerrain(float heightScale, float roughness) {
    maxHeight = heightScale;
    roughnessFactor = roughness;
    
    // Generate base heightmap using Perlin noise
    for (int x = 0; x < width; x++) {
        for (int z = 0; z < height; z++) {
            float nx = x / (float)width - 0.5f;
            float nz = z / (float)height - 0.5f;
            
            // Generate multi-octave noise
            float noiseVal = 0.0f;
            float amplitude = 1.0f;
            float frequency = 1.0f;
            float maxVal = 0.0f;
            
            for (int o = 0; o < octaves; o++) {
                float n = ofNoise(nx * frequency, nz * frequency);
                noiseVal += n * amplitude;
                maxVal += amplitude;
                amplitude *= roughnessFactor;
                frequency *= 2.0f;
            }
            
            // Normalize the result
            noiseVal /= maxVal;
            
            // Set height based on noise
            cells[x][z].height = noiseVal * maxHeight;
        }
    }
    
    // Apply smoothing
    std::vector<std::vector<TerrainCell>> smoothed = cells;
    for (int x = 1; x < width - 1; x++) {
        for (int z = 1; z < height - 1; z++) {
            // Simple box blur
            float sum = 0.0f;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dz = -1; dz <= 1; dz++) {
                    sum += cells[x + dx][z + dz].height;
                }
            }
            smoothed[x][z].height = sum / 9.0f;
        }
    }
    cells = smoothed;
    
    // Calculate derived values
    updateDerivedValues();
}

void TerrainSystem::updateDerivedValues() {
    // Calculate slopes and assign terrain types
    for (int x = 0; x < width; x++) {
        for (int z = 0; z < height; z++) {
            // Calculate slope using neighboring cells
            float dx = 0.0f, dz = 0.0f;
            
            if (x > 0 && x < width - 1) {
                dx = (cells[x + 1][z].height - cells[x - 1][z].height) / (2.0f * cellSize);
            }
            
            if (z > 0 && z < height - 1) {
                dz = (cells[x][z + 1].height - cells[x][z - 1].height) / (2.0f * cellSize);
            }
            
            cells[x][z].slope = sqrt(dx * dx + dz * dz);
            
            // Determine terrain type
            float height = cells[x][z].height;
            float slope = cells[x][z].slope;
            
            if (height < waterLevel) {
                cells[x][z].type = TerrainType::WATER;
                cells[x][z].vegetation = 0.0f;
                cells[x][z].aerialSpeedModifier = 1.0f;
                cells[x][z].terrestrialSpeedModifier = 0.3f;
                cells[x][z].visibilityFactor = 1.0f;
                cells[x][z].foodFactor = 0.7f; // Some food near water
            } 
            else if (height > maxHeight * 0.7f && slope > 0.5f) {
                cells[x][z].type = TerrainType::MOUNTAIN;
                cells[x][z].vegetation = 0.2f;
                cells[x][z].aerialSpeedModifier = 0.8f;
                cells[x][z].terrestrialSpeedModifier = 0.5f;
                cells[x][z].visibilityFactor = 1.0f;
                cells[x][z].foodFactor = 0.3f;
            }
            else if (height > maxHeight * 0.8f) {
                cells[x][z].type = TerrainType::CAVE;
                cells[x][z].vegetation = 0.1f;
                cells[x][z].aerialSpeedModifier = 0.5f;
                cells[x][z].terrestrialSpeedModifier = 0.7f;
                cells[x][z].visibilityFactor = 0.3f;
                cells[x][z].foodFactor = 0.2f;
            }
            else if (ofRandom(1.0f) < 0.3f && height > waterLevel + 1.0f) {
                cells[x][z].type = TerrainType::FOREST;
                cells[x][z].vegetation = ofRandom(0.6f, 1.0f);
                cells[x][z].aerialSpeedModifier = 0.7f;
                cells[x][z].terrestrialSpeedModifier = 0.8f;
                cells[x][z].visibilityFactor = 0.5f;
                cells[x][z].foodFactor = 1.0f;
            }
            else {
                cells[x][z].type = TerrainType::OPEN_FIELD;
                cells[x][z].vegetation = ofRandom(0.0f, 0.5f);
                cells[x][z].aerialSpeedModifier = 1.0f;
                cells[x][z].terrestrialSpeedModifier = 1.0f;
                cells[x][z].visibilityFactor = 1.0f;
                cells[x][z].foodFactor = 0.7f;
            }
        }
    }
}

TerrainSystem::TerrainCell& TerrainSystem::getCellAt(float worldX, float worldZ) {
    int x = gridX(worldX);
    int z = gridZ(worldZ);
    
    // Clamp to valid range
    x = ofClamp(x, 0, width - 1);
    z = ofClamp(z, 0, height - 1);
    
    return cells[x][z];
}

float TerrainSystem::getHeightAt(float worldX, float worldZ) {
    return getCellAt(worldX, worldZ).height;
}

ofVec3f TerrainSystem::getNormalAt(float worldX, float worldZ) {
    int x = gridX(worldX);
    int z = gridZ(worldZ);
    
    // Clamp to valid range
    x = ofClamp(x, 0, width - 1);
    z = ofClamp(z, 0, height - 1);
    
    // Get neighboring heights
    float h = cells[x][z].height;
    float hL = (x > 0) ? cells[x - 1][z].height : h;
    float hR = (x < width - 1) ? cells[x + 1][z].height : h;
    float hU = (z > 0) ? cells[x][z - 1].height : h;
    float hD = (z < height - 1) ? cells[x][z + 1].height : h;
    
    // Calculate tangent vectors
    ofVec3f tangentX(2 * cellSize, hR - hL, 0);
    ofVec3f tangentZ(0, hD - hU, 2 * cellSize);
    
    // Normal is cross product of tangents
    ofVec3f normal = tangentX.getCrossed(tangentZ);
    normal.normalize();
    
    return normal;
}

float TerrainSystem::getVegetationDensity(const ofVec3f& position) {
    return getCellAt(position.x, position.z).vegetation;
}

float TerrainSystem::getSpeedModifier(const ofVec3f& position, MovementType moveType) {
    TerrainCell& cell = getCellAt(position.x, position.z);
    
    if (moveType == MovementType::AERIAL) {
        return cell.aerialSpeedModifier;
    } else {
        return cell.terrestrialSpeedModifier;
    }
}

float TerrainSystem::getVisibilityModifier(const ofVec3f& position) {
    // Get the basic visibility factor from the cell
    TerrainCell& cell = getCellAt(position.x, position.z);
    float baseFactor = cell.visibilityFactor;
    
    // Apply additional modifiers based on environment
    
    // Time of day modifier (could be passed in as a parameter)
    // Assuming dayTime is a value from 0 (midnight) to 1 (noon) and back to 0
    float dayTime = 0.5f; // Default to midday
    float timeModifier = 0.5f + dayTime * 0.5f; // 0.5 at night, 1.0 at noon
    
    // Weather modifier (could be part of a weather system)
    float weatherModifier = 1.0f; // Default clear weather
    
    // Height modifier (higher positions are more visible)
    float heightModifier = 1.0f;
    if (position.y > maxHeight * 0.7f) {
        // Exposed high positions are more visible
        heightModifier = 1.2f;
    }
    
    // Vegetation modifier (denser vegetation reduces visibility)
    float vegetationModifier = 1.0f - (cell.vegetation * 0.7f);
    
    // Combine modifiers
    float finalVisibility = baseFactor * timeModifier * weatherModifier * heightModifier * vegetationModifier;
    
    // Ensure result is within reasonable bounds
    return ofClamp(finalVisibility, 0.1f, 1.0f);
}

ofVec3f TerrainSystem::findNearestCover(const ofVec3f& position, float maxDistance) {
    // Start with current position as default
    ofVec3f bestCover = position;
    float bestCoverDensity = getVegetationDensity(position);
    
    // Check cells within maxDistance
    int searchRadius = (int)(maxDistance / cellSize);
    int centerX = gridX(position.x);
    int centerZ = gridZ(position.z);
    
    // Only search if we need better cover than current position
    if (bestCoverDensity > 0.7f) {
        return position; // Already in excellent cover
    }
    
    float bestScore = bestCoverDensity;
    
    // Search in expanding radius
    for (int radius = 1; radius <= searchRadius; radius++) {
        // Check cells in a spiral pattern
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dz = -radius; dz <= radius; dz++) {
                // Skip cells that aren't on the edge of the current search radius
                // (We only want to check the perimeter of each radius step)
                if (abs(dx) != radius && abs(dz) != radius) continue;
                
                int x = centerX + dx;
                int z = centerZ + dz;
                
                // Skip if out of bounds
                if (!isInBounds(x, z)) continue;
                
                // Calculate world position
                ofVec3f worldPos(x * cellSize, cells[x][z].height, z * cellSize);
                
                // Get cover value of this cell
                float coverValue = cells[x][z].vegetation;
                
                // Prioritize forest and cave terrain types
                if (cells[x][z].type == TerrainType::FOREST) {
                    coverValue *= 1.5f;
                } else if (cells[x][z].type == TerrainType::CAVE) {
                    coverValue *= 2.0f;
                }
                
                // Calculate distance factor (closer is better)
                float dist = (worldPos - position).length();
                float distFactor = 1.0f - (dist / maxDistance);
                
                // Calculate final score (weighted combination of cover and distance)
                float score = coverValue * 0.7f + distFactor * 0.3f;
                
                // Update best cover if found better
                if (score > bestScore && coverValue > 0.4f) {
                    bestScore = score;
                    bestCover = worldPos;
                    bestCoverDensity = coverValue;
                }
            }
        }
        
        // Early exit if we found good enough cover
        if (bestCoverDensity > 0.7f) {
            break;
        }
    }
    
    return bestCover;
}

ofVec3f TerrainSystem::findNearestWater(const ofVec3f& position, float maxDistance) {
    // Find nearest water body
    int searchRadius = (int)(maxDistance / cellSize);
    int centerX = gridX(position.x);
    int centerZ = gridZ(position.z);
    
    float closestDist = maxDistance;
    ofVec3f waterPos = position; // Default to current position if no water found
    bool foundWater = false;
    
    // Search in expanding radius
    for (int radius = 1; radius <= searchRadius; radius++) {
        // Check cells on the perimeter of the current radius
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dz = -radius; dz <= radius; dz++) {
                // Skip cells that aren't on the edge of the current search radius
                if (abs(dx) != radius && abs(dz) != radius) continue;
                
                int x = centerX + dx;
                int z = centerZ + dz;
                
                // Skip if out of bounds
                if (!isInBounds(x, z)) continue;
                
                // Check if water cell
                if (cells[x][z].type == TerrainType::WATER) {
                    ofVec3f cellPos(x * cellSize, cells[x][z].height, z * cellSize);
                    float dist = (cellPos - position).length();
                    
                    if (dist < closestDist) {
                        closestDist = dist;
                        waterPos = cellPos;
                        foundWater = true;
                    }
                }
            }
        }
        
        // Early exit if we found water
        if (foundWater) {
            break;
        }
    }
    
    return waterPos;
}

float TerrainSystem::getCoverDensity(const ofVec3f& position, float radius) {
    // Calculate average vegetation density in the area
    int cellRadius = (int)(radius / cellSize) + 1;
    int centerX = gridX(position.x);
    int centerZ = gridZ(position.z);
    
    float totalVegetation = 0.0f;
    int cellCount = 0;
    
    // Sum vegetation values within radius
    for (int dx = -cellRadius; dx <= cellRadius; dx++) {
        for (int dz = -cellRadius; dz <= cellRadius; dz++) {
            int x = centerX + dx;
            int z = centerZ + dz;
            
            // Skip if out of bounds
            if (!isInBounds(x, z)) continue;
            
            // Calculate world position of cell center
            ofVec3f cellPos(x * cellSize, cells[x][z].height, z * cellSize);
            
            // Check if within radius
            float dist = (cellPos - position).length();
            if (dist <= radius) {
                float cellValue = cells[x][z].vegetation;
                
                // Adjust based on terrain type
                if (cells[x][z].type == TerrainType::FOREST) {
                    cellValue *= 1.2f;
                } else if (cells[x][z].type == TerrainType::CAVE) {
                    cellValue *= 1.5f;
                } else if (cells[x][z].type == TerrainType::MOUNTAIN && cells[x][z].slope > 0.5f) {
                    cellValue += 0.3f; // Steep terrain provides some cover
                }
                
                // Apply distance falloff (closer cells matter more)
                float weight = 1.0f - (dist / radius);
                totalVegetation += cellValue * weight;
                cellCount++;
            }
        }
    }
    
    // Calculate weighted average
    if (cellCount > 0) {
        return ofClamp(totalVegetation / cellCount, 0.0f, 1.0f);
    }
    
    return 0.0f;
}

void TerrainSystem::draw() const {
    ofPushStyle();
    
    // Draw terrain as a mesh
    ofMesh terrainMesh;
    terrainMesh.setMode(OF_PRIMITIVE_TRIANGLES);
    
    for (int x = 0; x < width - 1; x++) {
        for (int z = 0; z < height - 1; z++) {
            // Calculate world positions for the four corners of this grid cell
            ofVec3f v1(
                x * cellSize - (width * cellSize) / 2.0f,
                cells[x][z].height,
                z * cellSize - (height * cellSize) / 2.0f
            );
            
            ofVec3f v2(
                (x + 1) * cellSize - (width * cellSize) / 2.0f,
                cells[x + 1][z].height,
                z * cellSize - (height * cellSize) / 2.0f
            );
            
            ofVec3f v3(
                (x + 1) * cellSize - (width * cellSize) / 2.0f,
                cells[x + 1][z + 1].height,
                (z + 1) * cellSize - (height * cellSize) / 2.0f
            );
            
            ofVec3f v4(
                x * cellSize - (width * cellSize) / 2.0f,
                cells[x][z + 1].height,
                (z + 1) * cellSize - (height * cellSize) / 2.0f
            );
            
            // Get colors for each cell
            ofColor c1 = getTerrainColor(cells[x][z]);
            ofColor c2 = getTerrainColor(cells[x + 1][z]);
            ofColor c3 = getTerrainColor(cells[x + 1][z + 1]);
            ofColor c4 = getTerrainColor(cells[x][z + 1]);
            
            // Add triangles (two per grid cell)
            terrainMesh.addVertex(v1);
            terrainMesh.addVertex(v2);
            terrainMesh.addVertex(v3);
            
            terrainMesh.addVertex(v1);
            terrainMesh.addVertex(v3);
            terrainMesh.addVertex(v4);
            
            // Add colors
            terrainMesh.addColor(c1);
            terrainMesh.addColor(c2);
            terrainMesh.addColor(c3);
            
            terrainMesh.addColor(c1);
            terrainMesh.addColor(c3);
            terrainMesh.addColor(c4);
        }
    }
    
    terrainMesh.draw();
    
    ofPopStyle();
}

void TerrainSystem::drawDebug() const {
    ofPushStyle();
    
    // Draw terrain types with different colors
    for (int x = 0; x < width; x += 4) {  // Skip cells for performance
        for (int z = 0; z < height; z += 4) {
            ofVec3f pos(
                x * cellSize - (width * cellSize) / 2.0f,
                cells[x][z].height + 0.5f,  // Slightly above terrain
                z * cellSize - (height * cellSize) / 2.0f
            );
            
            // Draw point with color based on terrain type
            ofColor color;
            switch (cells[x][z].type) {
                case TerrainType::WATER:
                    color = ofColor(0, 0, 255);
                    break;
                case TerrainType::FOREST:
                    color = ofColor(0, 150, 0);
                    break;
                case TerrainType::MOUNTAIN:
                    color = ofColor(150, 150, 150);
                    break;
                case TerrainType::CAVE:
                    color = ofColor(100, 50, 0);
                    break;
                case TerrainType::OPEN_FIELD:
                    color = ofColor(200, 200, 100);
                    break;
            }
            
            ofSetColor(color);
            ofDrawSphere(pos, 0.3f);
        }
    }
    
    ofPopStyle();
}

ofColor TerrainSystem::getTerrainColor(const TerrainCell& cell) const {
    ofColor color;
    
    switch (cell.type) {
        case TerrainType::WATER:
            color = ofColor(0, 50, 200); // Blue
            break;
        case TerrainType::FOREST:
            color = ofColor(0, 100 + cell.vegetation * 100, 0); // Green
            break;
        case TerrainType::MOUNTAIN:
            color = ofColor(100 + cell.height * 5, 100 + cell.height * 5, 100 + cell.height * 5); // Gray
            break;
        case TerrainType::CAVE:
            color = ofColor(100, 50, 0); // Brown
            break;
        case TerrainType::OPEN_FIELD:
            // Blend between brown and green based on vegetation
            color = ofColor(150, 120, 50).getLerped(ofColor(100, 200, 50), cell.vegetation);
            break;
    }
    
    return color;
}

ofVec3f TerrainSystem::getMin() const {
    return ofVec3f(
        -width * cellSize / 2.0f,
        0,
        -height * cellSize / 2.0f
    );
}

ofVec3f TerrainSystem::getMax() const {
    return ofVec3f(
        width * cellSize / 2.0f,
        maxHeight,
        height * cellSize / 2.0f
    );
}

int TerrainSystem::gridX(float worldX) const {
    return (int)((worldX + (width * cellSize) / 2.0f) / cellSize);
}

int TerrainSystem::gridZ(float worldZ) const {
    return (int)((worldZ + (height * cellSize) / 2.0f) / cellSize);
}

bool TerrainSystem::isInBounds(int x, int z) const {
    return x >= 0 && x < width && z >= 0 && z < height;
} 