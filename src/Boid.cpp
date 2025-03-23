#include "Boid.h"

Boid::Boid() {
    // Initialize flocking parameters with default values
    separationWeight = 1.5;
    alignmentWeight = 1.0;
    cohesionWeight = 1.0;
    neighborhoodRadius = 3.0;
    separationRadius = 2.0;
    maxSpeed = 0.15;  // Reduced from 3.0 to 0.15 for this scale
    minSpeed = 0.1;   // Reduced from 0.5 to 0.1 for this scale
    maxForce = 0.05;  // Reduced proportionally to the speed reduction
    
    // Initialize rotation parameters
    fieldOfView = 240.0;  // Field of view in degrees (default 240 degrees)
    turnRate = 2.0;       // Maximum rotation rate per frame
    previousDirection = ofVec3f(0, 0, 1); // Default forward direction
    
    // Initialize individualistic characteristics
    uniqueness = ofRandom(0.1, 0.9);
    size = ofRandom(0.8, 1.2);
    energyLevel = ofRandom(0.7, 1.3);
    
    // Generate a personal color with some randomness but still related to uniqueness
    float hue = ofRandom(60, 260); // Blue to purple range
    personalColor.setHsb(hue, 200 + uniqueness * 55, 200 + uniqueness * 55);
    
    // Random initial velocity
    velocity = ofVec3f(ofRandom(-1, 1), ofRandom(-1, 1), ofRandom(-1, 1));
    velocity.normalize();
    velocity *= ofRandom(minSpeed, maxSpeed);
    
    // Initial color based on velocity
    float speed = velocity.length() / maxSpeed;
    color = ofColor::fromHsb(100 + speed * 100, 200, 255);
    
    // Initialize force vectors for debugging
    separationForce = ofVec3f(0);
    alignmentForce = ofVec3f(0);
    cohesionForce = ofVec3f(0);
    seekForce = ofVec3f(0);
    wanderForce = ofVec3f(0);
    boundaryForce = ofVec3f(0);
}

ofVec3f Boid::separate(vector<Boid*> neighbors) {
    float desiredSeparation = separationRadius;
    ofVec3f steer(0, 0, 0);
    int count = 0;
    
    // Check each boid in the system
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check if other boid is in field of view
        if (!isInFieldOfView(other)) continue;
        
        // Calculate distance from current boid to neighbor
        float d = position.distance(other->position);
        
        // If the distance is greater than 0 and less than desired separation
        if ((d > 0) && (d < desiredSeparation)) {
            // Calculate vector pointing away from neighbor
            ofVec3f diff = position - other->position;
            diff.normalize();
            
            // Weight by distance (closer = stronger)
            diff /= d;
            
            // Add to steering vector
            steer += diff;
            count++;
        }
    }
    
    // Average of steering vectors
    if (count > 0) {
        steer /= count;
    }
    
    // If we have a steering force
    if (steer.length() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxSpeed;
        steer -= velocity;
        steer.limit(maxForce);
    }
    
    return steer;
}

ofVec3f Boid::align(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum all velocities and count number of neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check if other boid is in field of view
        if (!isInFieldOfView(other)) continue;
        
        sum += other->velocity;
        count++;
    }
    
    if (count > 0) {
        // Calculate average velocity
        sum /= count;
        
        // Implement Reynolds: Steering = Desired - Velocity
        sum.normalize();
        sum *= maxSpeed;
        ofVec3f steer = sum - velocity;
        steer.limit(maxForce);
        return steer;
    }
    else {
        // If no neighbors, maintain current velocity
        return ofVec3f(0, 0, 0);
    }
}

ofVec3f Boid::cohere(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum all positions and count number of neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check if other boid is in field of view
        if (!isInFieldOfView(other)) continue;
        
        sum += other->position;
        count++;
    }
    
    if (count > 0) {
        // Calculate average position (center of mass)
        sum /= count;
        
        // Create vector pointing from boid towards center of mass
        return seek(sum);
    }
    else {
        // If no neighbors, maintain current direction
        return ofVec3f(0, 0, 0);
    }
}

bool Boid::isInFieldOfView(Boid* other) {
    // If the field of view is 360 degrees, everything is visible
    if (fieldOfView >= 360.0f) return true;
    
    // Calculate vector to other boid
    ofVec3f toOther = other->position - position;
    
    // No point in checking if the other boid is at the same position
    if (toOther.length() <= 0.001f) return true;
    
    // Normalize vectors
    toOther.normalize();
    ofVec3f forward = velocity;
    forward.normalize();
    
    // Calculate the angle between the forward direction and the direction to the other boid
    float angle = acos(forward.dot(toOther)) * 180.0f / PI;
    
    // Check if angle is within field of view (half angle on each side)
    return angle <= fieldOfView / 2.0f;
}

ofVec3f Boid::seek(ofVec3f target) {
    // Create desired velocity
    ofVec3f desired = target - position;
    
    // Scale to maximum speed
    float d = desired.length();
    desired.normalize();
    
    // If we're close to target, slow down (arrival behavior)
    if (d < 4.0) {
        float m = ofMap(d, 0, 4.0, 0, maxSpeed);
        desired *= m;
    }
    else {
        desired *= maxSpeed;
    }
    
    // Reynolds: Steering = Desired - Velocity
    ofVec3f steer = desired - velocity;
    steer.limit(maxForce);
    
    return steer;
}

ofVec3f Boid::wander() {
    // Create a random wandering force based on energy level
    float wanderStrength = 0.3 * energyLevel * uniqueness;
    ofVec3f steer(
        ofRandom(-wanderStrength, wanderStrength),
        ofRandom(-wanderStrength, wanderStrength),
        ofRandom(-wanderStrength, wanderStrength)
    );
    
    return steer;
}

void Boid::flock(vector<Boid*> neighbors) {
    // Calculate each steering force
    separationForce = separate(neighbors);
    alignmentForce = align(neighbors);
    cohesionForce = cohere(neighbors);
    wanderForce = wander();
    
    // Weight the forces by the boid's individual characteristics
    // More unique boids have more wandering and less group following
    separationForce *= separationWeight * (1.0 - uniqueness * 0.3);
    alignmentForce *= alignmentWeight * (1.0 - uniqueness * 0.5);
    cohesionForce *= cohesionWeight * (1.0 - uniqueness * 0.5);
    wanderForce *= uniqueness * energyLevel;
    
    // Apply forces
    applyForce(separationForce);
    applyForce(alignmentForce);
    applyForce(cohesionForce);
    applyForce(wanderForce);
}

void Boid::applyForce(ofVec3f force) {
    // Simple physics: Force = Mass * Acceleration
    // We assume Mass = 1, so Force = Acceleration
    acceleration += force;
}

void Boid::integrate() {
    // Save current direction for turn rate limiting
    previousDirection = velocity.getNormalized();
    
    // Update velocity with accumulated acceleration
    velocity += acceleration;
    
    // Limit speed to max and min
    float speed = velocity.length();
    
    // Enforce maximum speed limit
    if (speed > maxSpeed) {
        velocity.normalize();
        velocity *= maxSpeed;
    }
    
    // Enforce minimum speed
    if (speed < minSpeed && speed > 0) {
        velocity.normalize();
        velocity *= minSpeed;
    }
    
    // Apply turn rate limitation
    if (speed > 0) {
        ofVec3f newDirection = velocity.getNormalized();
        
        // Calculate angle between previous and new direction
        float angle = acos(previousDirection.dot(newDirection)) * 180.0f / PI;
        
        // If turning too fast, limit the rotation
        if (angle > turnRate) {
            // Calculate rotation axis
            ofVec3f rotationAxis = previousDirection.getCrossed(newDirection);
            
            if (rotationAxis.length() > 0) {
                rotationAxis.normalize();
                
                // Create quaternion for limited rotation
                ofQuaternion limitedRotation;
                limitedRotation.makeRotate(turnRate, rotationAxis);
                
                // Apply limited rotation to previous direction
                ofVec3f limitedDirection = previousDirection;
                limitedDirection = limitedRotation * limitedDirection;
                
                // Set velocity direction to limited direction but preserve speed
                velocity = limitedDirection * speed;
            }
        }
    }
    
    // Update position with velocity
    position += velocity;
    
    // Reset acceleration to 0 for the next frame
    acceleration *= 0;
    
    // Update color based on velocity (faster = redder)
    float speedRatio = ofMap(velocity.length(), minSpeed, maxSpeed, 0, 1);
    
    // Blend between personal color and speed color
    ofColor speedColor = ofColor::fromHsb(100 + speedRatio * 100, 200, 255);
    color = personalColor.getLerped(speedColor, 0.3);
}

void Boid::draw(ofMesh* customMesh) {
    ofPushMatrix();
    ofTranslate(position);
    
    // Calculate rotation from velocity
    ofVec3f direction = velocity;
    
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
    
    // Draw the boid
    ofSetColor(color);
    
    // Scale the mesh to match boid size
    float scaleValue = 0.5 * size;
    ofScale(scaleValue, scaleValue, scaleValue);
    
    if (customMesh != nullptr && customMesh->getNumVertices() > 0) {
        // Use the provided custom mesh
        customMesh->draw();
    } else {
        // Draw the default cone
        float baseSize = 0.2 * size;
        float height = 0.6 * size;
        ofDrawCone(0, 0, 0, baseSize, height);
    }
    
    ofPopMatrix();
}

void Boid::drawDebug(bool showVelocity, bool showNeighborhood, bool showForces) {
    ofPushStyle();
    
    if (showNeighborhood) {
        // Draw neighborhood radius
        ofSetColor(100, 100, 255, 40);
        ofDrawSphere(position, neighborhoodRadius);
        
        // Draw separation radius
        ofSetColor(255, 100, 100, 50);
        ofDrawSphere(position, separationRadius);
        
        // Draw field of view visualization
        if (fieldOfView < 360.0f && velocity.length() > 0) {
            ofVec3f forward = velocity.getNormalized();
            
            // Draw a FOV arc (approximate with multiple lines)
            ofSetColor(200, 200, 0, 100);
            float radius = 1.0;
            int segments = 20;
            float halfFOV = fieldOfView / 2.0f * PI / 180.0f;
            
            // Create a rotation to align FOV with velocity direction
            ofQuaternion forwardRotation;
            ofVec3f upVector(0, 1, 0);
            forwardRotation.makeRotate(ofVec3f(0, 0, 1), forward);
            
            for (int i = 0; i < segments; i++) {
                float angle1 = -halfFOV + i * (2 * halfFOV) / (segments - 1);
                float angle2 = -halfFOV + (i + 1) * (2 * halfFOV) / (segments - 1);
                
                // Calculate points on FOV arc
                ofVec3f point1(sin(angle1) * radius, 0, cos(angle1) * radius);
                ofVec3f point2(sin(angle2) * radius, 0, cos(angle2) * radius);
                
                // Rotate to align with forward direction
                point1 = forwardRotation * point1;
                point2 = forwardRotation * point2;
                
                // Draw line segment
                ofDrawLine(position, position + point1);
                ofDrawLine(position + point1, position + point2);
            }
        }
    }
    
    if (showVelocity) {
        // Draw velocity vector
        ofSetColor(0, 255, 0);
        ofDrawLine(position, position + velocity * 2.0);
    }
    
    if (showForces) {
        // Draw force vectors (scaled for visibility)
        float forceScale = 3.0;
        
        ofSetColor(255, 0, 0);
        ofDrawLine(position, position + separationForce * forceScale);
        
        ofSetColor(0, 255, 0);
        ofDrawLine(position, position + alignmentForce * forceScale);
        
        ofSetColor(0, 0, 255);
        ofDrawLine(position, position + cohesionForce * forceScale);
        
        ofSetColor(255, 255, 0);
        ofDrawLine(position, position + wanderForce * forceScale);
        
        if (boundaryForce.length() > 0) {
            ofSetColor(255, 0, 255);
            ofDrawLine(position, position + boundaryForce * forceScale);
        }
    }
    
    ofPopStyle();
} 