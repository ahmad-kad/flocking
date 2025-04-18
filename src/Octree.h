#pragma once

#include "ofMain.h"
#include "EcosystemTypes.h"
#include <vector>

// Forward declaration
class Boid;

class OctreeNode {
public:
    // Node structure
    ofVec3f center;
    float halfSize;
    std::vector<Boid*> boids;
    OctreeNode* children[8] = {nullptr};
    bool isLeaf = true;
    
    // Constants
    static const int MAX_BOIDS_PER_NODE = 8;
    static const int MIN_NODE_SIZE = 2;
    
    // Constructor/destructor
    OctreeNode(const ofVec3f& center, float halfSize);
    ~OctreeNode();
    
    // Core operations
    bool contains(const ofVec3f& point) const;
    void insert(Boid* boid);
    void split();
    int getOctant(const ofVec3f& point) const;
    
    // Query operations
    void queryRange(const ofVec3f& point, float radius, std::vector<Boid*>& result) const;
    void queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel, std::vector<Boid*>& result) const;
    void queryPotentialPrey(Boid* predator, float radius, std::vector<Boid*>& result) const;
    void queryPotentialThreats(Boid* prey, float radius, std::vector<Boid*>& result) const;
    void queryCompetitors(Boid* boid, float radius, std::vector<Boid*>& result) const;
    
    // Helper methods
    bool sphereIntersectsNode(const ofVec3f& point, float radius) const;
    float calculateVisibility(Boid* observer, Boid* target) const;
    
    // Debug
    void debugDraw() const;
};

class Octree {
public:
    // Root node and bounds
    OctreeNode* root = nullptr;
    ofVec3f worldMin = ofVec3f(-100, -50, -100);
    ofVec3f worldMax = ofVec3f(100, 50, 100);
    
    // Constructor/destructor
    Octree();
    ~Octree();
    
    // Core operations
    void build(const std::vector<Boid*>& boids);
    bool isInBounds(const ofVec3f& point) const;
    
    // Query operations
    std::vector<Boid*> queryRange(const ofVec3f& point, float radius) const;
    std::vector<Boid*> queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel) const;
    std::vector<Boid*> queryPotentialPrey(Boid* predator, float radius) const;
    std::vector<Boid*> queryPotentialThreats(Boid* prey, float radius) const;
    std::vector<Boid*> queryCompetitors(Boid* boid, float radius) const;
    
    // Bounds setting
    void setBounds(ofVec3f min, ofVec3f max);
    
    // Debug
    void debugDraw() const;
}; 