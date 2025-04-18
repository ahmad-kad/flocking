#pragma once

#include "ofMain.h"
#include "FoodSource.h"
#include "EcosystemTypes.h"

// Forward declarations
class TerrainSystem;
class Boid;

class FoodManager {
public:
    // Food source storage
    std::vector<std::unique_ptr<FoodSource>> foodSources;
    TerrainSystem* terrain = nullptr;
    
    // Food generation parameters
    float plantRegenerationRate = 1.0f;
    float insectRegenerationRate = 1.5f;
    float carrionDecayRate = 0.5f;
    
    // Constructor
    FoodManager(TerrainSystem* terrain = nullptr);
    
    // System methods
    void setup(int plantFoodCount, int insectSwarmCount);
    void update(float deltaTime);
    void draw() const;
    
    // Terrain integration
    void setTerrain(TerrainSystem* terrainSystem) { terrain = terrainSystem; }
    
    // Food management
    void createPlantFood(int count);
    void createInsectSwarms(int count);
    void addCarrion(const ofVec3f& position, float size, MovementType sourceType);
    
    // Food discovery
    std::vector<FoodSource*> findAccessibleFood(Boid* boid, float searchRadius);
    bool consumeFood(Boid* boid, FoodSource* food);
    
    // Statistics
    int getPlantFoodCount() const;
    int getInsectSwarmCount() const;
    int getCarrionCount() const;
    float getTotalAvailableNutrition() const;
    
    // Bounds setting
    void setBounds(ofVec3f min, ofVec3f max);
    
private:
    // Simulation bounds
    ofVec3f boundsMin = ofVec3f(-100, -50, -100);
    ofVec3f boundsMax = ofVec3f(100, 50, 100);
}; 