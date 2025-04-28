#include "BoidRenderer.h"
#include "Boid.h" // Include Boid.h for access to its members

void BoidRenderer::draw(const Boid& boid, ofMesh* customMesh) {
    ofPushMatrix();
    ofPushStyle();
    
    // Set base color based on trophic level
    ofColor boidColor;
    switch (boid.trophicLevel) {
        case TrophicLevel::APEX_PREDATOR:
            boidColor = ofColor(220, 50, 50, 200); // Stronger red for apex predators
            break;
        case TrophicLevel::MID_PREDATOR:
            boidColor = ofColor(220, 100, 50, 200); // Orange-red for mid predators
            break;
        case TrophicLevel::PREY:
            boidColor = ofColor(100, 220, 100, 200); // Green for prey
            break;
        case TrophicLevel::APEX_PREY:
            boidColor = ofColor(50, 180, 220, 200); // Blue-green for apex prey
            break;
        default:
            boidColor = ofColor(200, 200, 200, 200); // Gray for undefined
    }
    
    // Modify color based on behavior state - make more distinct
    if (boid.behaviorState == BehaviorState::HUNTING) {
        boidColor = boidColor.getLerped(ofColor(255, 0, 0), 0.7); // Stronger red tint when hunting
    } else if (boid.behaviorState == BehaviorState::FLEEING) {
        boidColor = boidColor.getLerped(ofColor(255, 255, 0), 0.7); // Stronger yellow tint when fleeing
    } else if (boid.behaviorState == BehaviorState::FEEDING) {
        boidColor = boidColor.getLerped(ofColor(0, 255, 0), 0.7); // Stronger green tint when feeding
    }
    
    // Adjust visual representation based on movement type
    if (boid.movementType == MovementType::AERIAL) {
        // More transparent for aerial creatures
        ofSetColor(boidColor, 180);
    } else if (boid.movementType == MovementType::TERRESTRIAL) {
        // Terrestrial creatures are more solid
        ofSetColor(boidColor, 255);
    }
    
    // Translate to position
    ofTranslate(boid.position);
    
    // Rotate to face velocity direction
    ofVec3f forward = boid.velocity.getNormalized();
    ofVec3f up(0, 1, 0);
    
    if (forward.length() > 0) {
        ofVec3f right = up.getCrossed(forward).getNormalized();
        up = forward.getCrossed(right).getNormalized();
        
        // Fix rotation matrix setup
        float rotAngle = ofRadToDeg(atan2(forward.z, forward.x));
        ofRotateDeg(rotAngle, 0, 1, 0);
        ofRotateDeg(ofRadToDeg(asin(-forward.y)), 1, 0, 0);
    }
    
    // Draw custom mesh if available, otherwise draw default shape
    if (customMesh != nullptr) {
        // Scale the mesh based on size and trophic level
        float scaleFactor = boid.size;
        ofScale(scaleFactor, scaleFactor, scaleFactor);
        
        customMesh->draw();
    } else {
        // Draw distinct shapes based on trophic level and movement type
        if (boid.trophicLevel == TrophicLevel::APEX_PREDATOR || boid.trophicLevel == TrophicLevel::MID_PREDATOR) {
            // Predator shape - sharper cone for better visibility
            ofDrawCone(0, 0, 0, boid.size * 0.8f, boid.size * 2.5f);
        } else if (boid.trophicLevel == TrophicLevel::PREY) {
            // Different shapes based on movement type
            if (boid.movementType == MovementType::AERIAL) {
                // Bird shape with wingspan
                ofPushMatrix();
                // Body
                ofDrawSphere(0, 0, 0, boid.size * 0.6f);
                
                // Wings
                float wingScale = boid.size * 1.8f;
                float wingHeight = boid.size * 0.1f;
                ofPushMatrix();
                ofRotateDeg(sin(ofGetElapsedTimef() * 2.0) * 20, 0, 0, 1); // Wing flapping animation
                ofDrawBox(-wingScale/2, 0, 0, wingScale, wingHeight, boid.size * 0.8f);
                ofPopMatrix();
                
                ofPushMatrix();
                ofRotateDeg(-sin(ofGetElapsedTimef() * 2.0) * 20, 0, 0, 1); // Wing flapping animation
                ofDrawBox(wingScale/2, 0, 0, wingScale, wingHeight, boid.size * 0.8f);
                ofPopMatrix();
                ofPopMatrix();
            } else if (boid.movementType == MovementType::TERRESTRIAL) {
                // More defined quadruped shape
                ofDrawSphere(0, 0, 0, boid.size * 1.2f); // Body
                ofDrawSphere(0, -boid.size * 0.8f, boid.size * 0.8f, boid.size * 0.5f); // Head
            } else {
                // Default shape - slightly elliptical
                ofDrawSphere(0, 0, 0, boid.size);
            }
        } else if (boid.trophicLevel == TrophicLevel::APEX_PREY) {
            // Special prey with distinctive shape
            if (boid.movementType == MovementType::AERIAL) {
                // Distinctive flying creature
                ofDrawSphere(0, 0, 0, boid.size * 1.2f);
                
                // Add wings
                ofPushMatrix();
                ofRotateDeg(90, 0, 1, 0);
                ofDrawCylinder(-boid.size * 1.2f, 0, boid.size * 0.1f, boid.size * 0.8f);
                ofDrawCylinder(boid.size * 1.2f, 0, boid.size * 0.1f, boid.size * 0.8f);
                ofPopMatrix();
            } else {
                // More rounded special prey shape
                ofDrawSphere(0, 0, 0, boid.size * 1.0f); // Body
                ofDrawSphere(0, -boid.size * 0.8f, boid.size * 0.8f, boid.size * 0.6f); // Head
            }
        } else {
            // Default shape
            ofDrawSphere(0, 0, 0, boid.size);
        }
    }
    
    ofPopStyle();
    ofPopMatrix();
}

void BoidRenderer::drawDebug(const Boid& boid, bool showVelocity, bool showNeighborhood, bool showForces) {
    ofPushStyle();
    
    // Draw velocity vector
    if (showVelocity) {
        ofSetColor(0, 200, 200);
        ofDrawArrow(boid.position, boid.position + boid.velocity, 5);
    }
    
    // Draw neighborhood sphere
    if (showNeighborhood) {
        ofColor neighborhoodColor(200, 200, 0, 40);
        ofSetColor(neighborhoodColor);
        ofDrawSphere(boid.position, boid.neighborhoodRadius);
    }
    
    // Draw forces if requested
    if (showForces) {
        // Draw separation force
        ofSetColor(255, 0, 0, 200);
        ofDrawArrow(boid.position, boid.position + boid.separationForce * 2.0, 2);
        
        // Draw alignment force
        ofSetColor(0, 255, 0, 200);
        ofDrawArrow(boid.position, boid.position + boid.alignmentForce * 2.0, 2);
        
        // Draw cohesion force
        ofSetColor(0, 0, 255, 200);
        ofDrawArrow(boid.position, boid.position + boid.cohesionForce * 2.0, 2);
        
        // Draw seek force if applicable
        if (boid.seekForce.length() > 0) {
            ofSetColor(255, 0, 255, 200);
            ofDrawArrow(boid.position, boid.position + boid.seekForce * 2.0, 2);
        }
        
        // Draw wander force
        ofSetColor(255, 255, 0, 200);
        ofDrawArrow(boid.position, boid.position + boid.wanderForce * 2.0, 2);
        
        // Draw boundary force
        ofSetColor(0, 255, 255, 200);
        ofDrawArrow(boid.position, boid.position + boid.boundaryForce * 2.0, 2);
    }
    
    // Draw current state text
    ofColor stateColor;
    std::string stateText = ""; // Use std::string
    
    switch (boid.behaviorState) {
        case BehaviorState::NORMAL:
            stateColor = ofColor(200, 200, 200);
            stateText = "NORMAL";
            break;
        case BehaviorState::HUNTING:
            stateColor = ofColor(255, 0, 0);
            stateText = "HUNTING";
            break;
        case BehaviorState::FLEEING:
            stateColor = ofColor(255, 255, 0);
            stateText = "FLEEING";
            break;
        case BehaviorState::FEEDING:
            stateColor = ofColor(0, 255, 0);
            stateText = "FEEDING";
            break;
        case BehaviorState::HIDING:
            stateColor = ofColor(150, 150, 255);
            stateText = "HIDING";
            break;
        // Add RESTING case if needed based on Boid.h definitions
        case BehaviorState::RESTING: // Assuming RESTING is defined
            stateColor = ofColor(100, 100, 100);
            stateText = "RESTING";
            break;
        default:
            stateColor = ofColor(200, 200, 200);
            stateText = "UNKNOWN";
    }
    
    ofSetColor(stateColor);
    ofDrawBitmapString(stateText, boid.position + ofVec3f(0, 0, 20));
    
    ofPopStyle();
} 