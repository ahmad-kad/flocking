#include "Octree.h"
#include "Boid.h"

//----------------------------
// OctreeNode Implementation
//----------------------------

OctreeNode::OctreeNode(const ofVec3f& center, float halfSize) 
    : center(center), halfSize(halfSize) {
}

OctreeNode::~OctreeNode() {
    // Delete all children
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            delete children[i];
            children[i] = nullptr;
        }
    }
}

bool OctreeNode::contains(const ofVec3f& point) const {
    return (point.x >= center.x - halfSize && point.x <= center.x + halfSize &&
            point.y >= center.y - halfSize && point.y <= center.y + halfSize &&
            point.z >= center.z - halfSize && point.z <= center.z + halfSize);
}

void OctreeNode::insert(Boid* boid) {
    // Check if this node contains the boid
    if (!contains(boid->position)) {
        return;
    }
    
    // If this is a leaf node with space, add the boid
    if (isLeaf && boids.size() < MAX_BOIDS_PER_NODE) {
        boids.push_back(boid);
        return;
    }
    
    // If this is a leaf but full, split it
    if (isLeaf && halfSize > MIN_NODE_SIZE) {
        split();
    }
    
    // If this is not a leaf, pass to appropriate child
    if (!isLeaf) {
        int octant = getOctant(boid->position);
        if (children[octant]) {
            children[octant]->insert(boid);
        }
    } else {
        // If we're at minimum size, just add it to this node
        boids.push_back(boid);
    }
}

void OctreeNode::split() {
    // Early return if already split or too small
    if (!isLeaf || halfSize <= MIN_NODE_SIZE) return;
    
    float childHalfSize = halfSize / 2.0f;
    
    // Create 8 children in all octants
    for (int i = 0; i < 8; i++) {
        ofVec3f childCenter = center;
        childCenter.x += ((i & 1) ? childHalfSize : -childHalfSize);
        childCenter.y += ((i & 2) ? childHalfSize : -childHalfSize);
        childCenter.z += ((i & 4) ? childHalfSize : -childHalfSize);
        
        children[i] = new OctreeNode(childCenter, childHalfSize);
    }
    
    // Move existing boids to children
    for (Boid* boid : boids) {
        int octant = getOctant(boid->position);
        children[octant]->insert(boid);
    }
    
    // Clear boids from this node and mark as non-leaf
    boids.clear();
    isLeaf = false;
}

int OctreeNode::getOctant(const ofVec3f& point) const {
    int octant = 0;
    if (point.x >= center.x) octant |= 1;
    if (point.y >= center.y) octant |= 2;
    if (point.z >= center.z) octant |= 4;
    return octant;
}

bool OctreeNode::sphereIntersectsNode(const ofVec3f& point, float radius) const {
    // Calculate closest point to sphere within the node
    float x = std::max(center.x - halfSize, std::min(point.x, center.x + halfSize));
    float y = std::max(center.y - halfSize, std::min(point.y, center.y + halfSize));
    float z = std::max(center.z - halfSize, std::min(point.z, center.z + halfSize));
    
    // Calculate distance from closest point to sphere center
    float distance = ofVec3f(x, y, z).distance(point);
    
    // If distance is less than radius, they intersect
    return distance <= radius;
}

void OctreeNode::queryRange(const ofVec3f& point, float radius, std::vector<Boid*>& result) const {
    // If this node doesn't intersect with the query sphere, return
    if (!sphereIntersectsNode(point, radius)) return;
    
    // If this is a leaf, check each boid
    if (isLeaf) {
        for (Boid* boid : boids) {
            if (boid->position.distance(point) <= radius) {
                result.push_back(boid);
            }
        }
        return;
    }
    
    // If not a leaf, query each child
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->queryRange(point, radius, result);
        }
    }
}

float OctreeNode::calculateVisibility(Boid* observer, Boid* target) const {
    // Simple visibility calculation
    // In a more complex system, this would account for terrain, obstacles, etc.
    float distance = observer->position.distance(target->position);
    float visibility = 1.0f - (distance / (observer->neighborhoodRadius * 2));
    return ofClamp(visibility, 0.0f, 1.0f);
}

void OctreeNode::queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel, std::vector<Boid*>& result) const {
    // If this node doesn't intersect with the query sphere, return
    if (!sphereIntersectsNode(point, radius)) return;
    
    // If this is a leaf, check each boid
    if (isLeaf) {
        for (Boid* boid : boids) {
            if (boid->trophicLevel == targetLevel && 
                boid->position.distance(point) <= radius) {
                result.push_back(boid);
            }
        }
        return;
    }
    
    // If not a leaf, query each child
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->queryByType(point, radius, targetLevel, result);
        }
    }
}

void OctreeNode::queryPotentialPrey(Boid* predator, float radius, std::vector<Boid*>& result) const {
    // If this node doesn't intersect with the query sphere, return
    if (!sphereIntersectsNode(predator->position, radius)) return;
    
    // If this is a leaf, check each boid
    if (isLeaf) {
        for (Boid* boid : boids) {
            // Check if boid is potential prey (of a lower trophic level)
            if (boid != predator && 
                ((int)predator->trophicLevel < (int)boid->trophicLevel) &&
                boid->position.distance(predator->position) <= radius) {
                
                // Factor in visibility
                float visibility = calculateVisibility(predator, boid);
                if (visibility > 0.2f) {  // Only add if somewhat visible
                    result.push_back(boid);
                }
            }
        }
        return;
    }
    
    // If not a leaf, query each child
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->queryPotentialPrey(predator, radius, result);
        }
    }
}

void OctreeNode::queryPotentialThreats(Boid* prey, float radius, std::vector<Boid*>& result) const {
    // If this node doesn't intersect with the query sphere, return
    if (!sphereIntersectsNode(prey->position, radius)) return;
    
    // If this is a leaf, check each boid
    if (isLeaf) {
        for (Boid* boid : boids) {
            // Check if boid is potential threat (of a higher trophic level)
            if (boid != prey && 
                ((int)boid->trophicLevel < (int)prey->trophicLevel) &&
                boid->position.distance(prey->position) <= radius) {
                
                // Factor in visibility
                float visibility = calculateVisibility(prey, boid);
                if (visibility > 0.1f) {  // Only add if somewhat visible (prey more sensitive)
                    result.push_back(boid);
                }
            }
        }
        return;
    }
    
    // If not a leaf, query each child
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->queryPotentialThreats(prey, radius, result);
        }
    }
}

void OctreeNode::queryCompetitors(Boid* boid, float radius, std::vector<Boid*>& result) const {
    // If this node doesn't intersect with the query sphere, return
    if (!sphereIntersectsNode(boid->position, radius)) return;
    
    // If this is a leaf, check each boid
    if (isLeaf) {
        for (Boid* other : boids) {
            // Check if boid is competitor (same trophic level)
            if (other != boid && 
                other->trophicLevel == boid->trophicLevel &&
                other->position.distance(boid->position) <= radius) {
                result.push_back(other);
            }
        }
        return;
    }
    
    // If not a leaf, query each child
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->queryCompetitors(boid, radius, result);
        }
    }
}

void OctreeNode::debugDraw() const {
    ofPushStyle();
    
    // Draw node bounds
    ofNoFill();
    ofSetColor(255, 255, 255, 80);
    ofDrawBox(center, halfSize * 2, halfSize * 2, halfSize * 2);
    
    // If this is a leaf with boids, highlight it
    if (isLeaf && !boids.empty()) {
        ofSetColor(255, 0, 0, 50);
        ofFill();
        ofDrawBox(center, halfSize * 2, halfSize * 2, halfSize * 2);
    }
    
    // Draw children
    for (int i = 0; i < 8; i++) {
        if (children[i]) {
            children[i]->debugDraw();
        }
    }
    
    ofPopStyle();
}

//----------------------------
// Octree Implementation
//----------------------------

Octree::Octree() {
    // Create root node at world center
    ofVec3f worldCenter = (worldMin + worldMax) / 2.0f;
    float size = std::max(std::max(worldMax.x - worldMin.x, worldMax.y - worldMin.y), worldMax.z - worldMin.z) / 2.0f;
    root = new OctreeNode(worldCenter, size);
}

Octree::~Octree() {
    if (root) {
        delete root;
        root = nullptr;
    }
}

void Octree::build(const std::vector<Boid*>& boids) {
    // Clear existing tree
    if (root) {
        delete root;
    }
    
    // Create new root
    ofVec3f worldCenter = (worldMin + worldMax) / 2.0f;
    float size = std::max(std::max(worldMax.x - worldMin.x, worldMax.y - worldMin.y), worldMax.z - worldMin.z) / 2.0f;
    root = new OctreeNode(worldCenter, size);
    
    // Insert all boids
    for (Boid* boid : boids) {
        if (isInBounds(boid->position)) {
            root->insert(boid);
        }
    }
}

bool Octree::isInBounds(const ofVec3f& point) const {
    return (point.x >= worldMin.x && point.x <= worldMax.x &&
            point.y >= worldMin.y && point.y <= worldMax.y &&
            point.z >= worldMin.z && point.z <= worldMax.z);
}

std::vector<Boid*> Octree::queryRange(const ofVec3f& point, float radius) const {
    std::vector<Boid*> result;
    if (root) {
        root->queryRange(point, radius, result);
    }
    return result;
}

std::vector<Boid*> Octree::queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel) const {
    std::vector<Boid*> result;
    if (root) {
        root->queryByType(point, radius, targetLevel, result);
    }
    return result;
}

std::vector<Boid*> Octree::queryPotentialPrey(Boid* predator, float radius) const {
    std::vector<Boid*> result;
    if (root) {
        root->queryPotentialPrey(predator, radius, result);
    }
    return result;
}

std::vector<Boid*> Octree::queryPotentialThreats(Boid* prey, float radius) const {
    std::vector<Boid*> result;
    if (root) {
        root->queryPotentialThreats(prey, radius, result);
    }
    return result;
}

std::vector<Boid*> Octree::queryCompetitors(Boid* boid, float radius) const {
    std::vector<Boid*> result;
    if (root) {
        root->queryCompetitors(boid, radius, result);
    }
    return result;
}

void Octree::setBounds(ofVec3f min, ofVec3f max) {
    worldMin = min;
    worldMax = max;
}

void Octree::debugDraw() const {
    if (root) {
        root->debugDraw();
    }
} 