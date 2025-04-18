#pragma once

#include "ofMain.h"
#include "Boid.h"

// Spatial grid cell
struct Cell {
    vector<Boid*> boids;
};

// Flocking modes
enum FlockMode {
    NORMAL,          // Flocking
    SCATTERED,       // Individualistic
    TIGHT,           // Grouped
    PREDATOR_PREY    // Chasing
};

class FlockSystem {
public:
    FlockSystem();
    ~FlockSystem();
    
    // Update and draw
    void update();
    void draw(ofMesh* customMesh = nullptr);
    
    // Boid management
    void addBoid(Boid* boid);
    void addBoids(int count, ofVec3f position, float spreadRadius = 1.0f);
    void removeBoid(Boid* boid);
    void clear();
    vector<Boid*> getAllBoids() { return boids; }
    int getCount() { return boids.size(); }
    
    // Neighbors and grid
    vector<Boid*> getNeighbors(Boid* boid, float radius);
    void updateSpatialGrid();
    
    // Set parameters
    void setParameters(float sepWeight, float aliWeight, float cohWeight);
    void setMaxSpeed(float speed);
    void setMinSpeed(float speed);
    void setMaxForce(float force);
    void setNeighborhoodRadius(float radius);
    void setSeparationRadius(float radius);
    
    // Global parameters
    void setFlockMode(FlockMode mode);
    void setIndividualismFactor(float factor);   // 0-1, individual traits
    void setSystemChaos(float chaos);           // 0-1, turbulence
    void setBoidsVariability(float variability); // 0-1, differences
    void setColorBasedFlocking(bool enabled, float threshold, float influence = 0);
    
    // Boundary settings
    void setBounds(ofVec3f min, ofVec3f max);
    ofVec3f boundaryForce(Boid& boid);
    float boundaryForceWeight;
    float boundaryDistance;
    void applySmoothedFlockingBehavior(Boid* boid, vector<Boid*>& neighbors);
    void adjustForcesForFlockMode(Boid* boid);
    
    
    // Target seeking
    void setTarget(ofVec3f target);
    bool hasTarget;
    ofVec3f target;
    float targetWeight;
    
    // Debug options
    bool showDebug;
    bool showVelocities;
    bool showNeighborhoods;
    bool showForces;
    bool showGrid;
    
private:
    vector<Boid*> boids;
    
    // Global settings
    FlockMode flockMode;
    float individualismFactor;
    float systemChaos;
    float boidsVariability;
    
    // Color-based settings
    bool colorBasedFlockingEnabled;
    float colorInfluenceFactor;
    float colorSimilarityThreshold;
    float getColorSimilarity(Boid* boid1, Boid* boid2);
    
    // Spatial grid
    vector<vector<vector<Cell>>> grid;
    ofVec3f gridMin;
    ofVec3f gridMax;
    int gridResolution;
    float cellSize;
    
    // Boundary limits
    ofVec3f boundsMin;
    ofVec3f boundsMax;
    
    // Convert position to grid coordinates
    ofVec3f worldToGrid(ofVec3f position);
    
    // Get cell at coordinates
    Cell* getCell(int x, int y, int z);
    
    // Draw grid
    void drawGrid();
    
    // Draw boundaries
    void drawBoundaries();
}; 
