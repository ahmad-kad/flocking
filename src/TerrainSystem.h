#pragma once

#include "ofMain.h"
#include "EcosystemTypes.h"

class TerrainSystem {
public:
    // Terrain cell structure
    struct TerrainCell {
        float height = 0.0f;
        float slope = 0.0f;
        float vegetation = 0.0f;
        TerrainType type = TerrainType::OPEN_FIELD;
        
        // Movement modifiers
        float aerialSpeedModifier = 1.0f;
        float terrestrialSpeedModifier = 1.0f;
        
        // Visibility modifier
        float visibilityFactor = 1.0f;
        
        // Food abundance factor
        float foodFactor = 1.0f;
    };
    
    // Terrain data
    std::vector<std::vector<TerrainCell>> cells;
    int width = 256;
    int height = 256;
    float cellSize = 1.0f;
    
    // Height parameters
    float maxHeight = 20.0f;
    float waterLevel = 2.0f;
    
    // Setup and generation
    void setup(int w, int h, float cellSize);
    void generateTerrain(float heightScale = 10.0f, float roughness = 0.5f);
    
    // Terrain queries
    TerrainCell& getCellAt(float worldX, float worldZ);
    float getHeightAt(float worldX, float worldZ);
    ofVec3f getNormalAt(float worldX, float worldZ);
    float getVegetationDensity(const ofVec3f& position);
    float getSpeedModifier(const ofVec3f& position, MovementType moveType);
    float getVisibilityModifier(const ofVec3f& position);
    
    // Cover and navigation
    ofVec3f findNearestCover(const ofVec3f& position, float maxDistance = 20.0f);
    ofVec3f findNearestWater(const ofVec3f& position, float maxDistance = 20.0f);
    float getCoverDensity(const ofVec3f& position, float radius = 5.0f);
    
    // Rendering
    void draw() const;
    void drawDebug() const;
    
    // Bounds
    ofVec3f getMin() const;
    ofVec3f getMax() const;
    
    // Accessors
    int getWidth() const { return width; }
    int getDepth() const { return height; }
    float getHeightScale() const { return maxHeight; }
    float getRoughness() const { return roughnessFactor; }
    bool isInitialized() const { return !cells.empty(); }
    
private:
    // Generation parameters
    float roughnessFactor = 0.5f;
    int octaves = 6;
    
    // Helper functions
    ofColor getTerrainColor(const TerrainCell& cell) const;
    void updateDerivedValues();
    int gridX(float worldX) const;
    int gridZ(float worldZ) const;
    bool isInBounds(int x, int z) const;
}; 