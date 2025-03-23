#pragma once

#include "ofMain.h"
#include "Boid.h"

// Simple spatial grid cell
struct Cell {
    vector<Boid*> boids;
};

// Flocking system modes
enum FlockMode {
    NORMAL,          // Standard flocking
    SCATTERED,       // More individualistic behavior
    TIGHT,           // Tightly grouped
    PREDATOR_PREY    // Some boids chase others
};

class FlockSystem {
public:
    FlockSystem();
    ~FlockSystem();
    
    // Flock behaviors
    void update();
    void draw(ofMesh* customMesh = nullptr);
    
    // Boid management
    void addBoid(Boid* boid);
    void addBoids(int count, ofVec3f position, float spreadRadius = 1.0f);
    void removeBoid(Boid* boid);
    void clear();
    vector<Boid*> getAllBoids() { return boids; }
    int getCount() { return boids.size(); }
    
    // Spatial partitioning
    vector<Boid*> getNeighbors(Boid* boid, float radius);
    void updateSpatialGrid();
    
    // Configuration
    void setParameters(float sepWeight, float aliWeight, float cohWeight);
    void setMaxSpeed(float speed);
    void setMinSpeed(float speed);
    void setMaxForce(float force);
    void setNeighborhoodRadius(float radius);
    void setSeparationRadius(float radius);
    
    // Global behavior parameters
    void setFlockMode(FlockMode mode);
    void setIndividualismFactor(float factor);   // 0-1, how much individual traits affect behavior
    void setSystemChaos(float chaos);           // 0-1, global turbulence level
    void setBoidsVariability(float variability); // 0-1, how different boids are from each other
    void setColorBasedFlocking(bool enabled, float threshold, float influence = 0);
    
    // Boundary handling
    void setBounds(ofVec3f min, ofVec3f max);
    ofVec3f boundaryForce(Boid& boid);
    float boundaryForceWeight;
    float boundaryDistance;
    
    // Optional target seeking
    void setTarget(ofVec3f target);
    bool hasTarget;
    ofVec3f target;
    float targetWeight;
    
    // Debug visualization
    bool showDebug;
    bool showVelocities;
    bool showNeighborhoods;
    bool showForces;
    bool showGrid;
    
private:
    vector<Boid*> boids;
    
    // Global parameters
    FlockMode flockMode;
    float individualismFactor;
    float systemChaos;
    float boidsVariability;
    
    // Color-based flocking
    bool colorBasedFlockingEnabled;
    float colorInfluenceFactor;
    float colorSimilarityThreshold;
    float getColorSimilarity(Boid* boid1, Boid* boid2);
    
    // Spatial partitioning grid
    vector<vector<vector<Cell>>> grid;
    ofVec3f gridMin;
    ofVec3f gridMax;
    int gridResolution;
    float cellSize;
    
    // Boundary constraints
    ofVec3f boundsMin;
    ofVec3f boundsMax;
    
    // Convert world position to grid cell coordinates
    ofVec3f worldToGrid(ofVec3f position);
    
    // Get grid cell at coordinates
    Cell* getCell(int x, int y, int z);
    
    // Draw spatial grid for debugging
    void drawGrid();
    
    // Draw boundaries wireframe
    void drawBoundaries();
}; 