#include "Boid.h"

Boid::Boid() {
    // Flocking parameters
    separationWeight = 1.5;
    alignmentWeight = 1.0;
    cohesionWeight = 1.0;
    neighborhoodRadius = 3.0;
    separationRadius = 2.0;
    maxSpeed = 0.15;
    minSpeed = 0.1;
    maxForce = 0.01;
    
    // Rotation parameters
    fieldOfView = 240.0;
    turnRate = 2.0;
    previousDirection = ofVec3f(0, 0, 1);
    
    // Individual characteristics
    uniqueness = ofRandom(0.1, 0.9);
    size = 100;
    energyLevel = ofRandom(0.7, 1.3);
    
    // Personal color
    float hue = ofRandom(60, 260); // Blue to purple
    personalColor.setHsb(hue, 200 + uniqueness * 55, 200 + uniqueness * 55);
    
    // Initial velocity
    velocity = ofVec3f(ofRandom(-.05, .05), ofRandom(-.05, .05), ofRandom(-.05, .05));
    velocity.normalize();
    velocity *= ofRandom(minSpeed, maxSpeed);
    
    // Initial color based on speed
    float speed = velocity.length() / maxSpeed;
    color = ofColor::fromHsb(100 + speed * 10, 200, 255);
    
    // Force vectors for debugging
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
    
    // Check each neighbor
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check field of view
        if (!isInFieldOfView(other)) continue;
        
        // Distance to neighbor
        float d = position.distance(other->position);
        
        // If within separation distance
        if ((d > 0) && (d < desiredSeparation)) {
            // Vector away from neighbor
            ofVec3f diff = position - other->position;
            diff.normalize();
            diff /= d; // Weight by distance
            
            steer += diff;
            count++;
        }
    }
    
    // Average steering vectors
    if (count > 0) {
        steer /= count;
    }
    
    // Steering force
    if (steer.length() > 0) {
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
    
    // Sum velocities and count neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check field of view
        if (!isInFieldOfView(other)) continue;
        
        sum += other->velocity;
        count++;
    }
    
    if (count > 0) {
        // Average velocity
        sum /= count;
        sum.normalize();
        sum *= maxSpeed;
        ofVec3f steer = sum - velocity;
        steer.limit(maxForce);
        return steer;
    } else {
        // No neighbors
        return ofVec3f(0, 0, 0);
    }
}

ofVec3f Boid::cohere(vector<Boid*> neighbors) {
    ofVec3f sum(0, 0, 0);
    int count = 0;
    
    // Sum positions and count neighbors
    for (int i = 0; i < neighbors.size(); i++) {
        Boid* other = neighbors[i];
        
        // Check field of view
        if (!isInFieldOfView(other)) continue;
        
        sum += other->position;
        count++;
    }
    
    if (count > 0) {
        // Average position
        sum /= count;
        return seek(sum);
    } else {
        // No neighbors
        return ofVec3f(0, 0, 0);
    }
}

bool Boid::isInFieldOfView(Boid* other) {
    // 360 degrees visibility
    if (fieldOfView >= 360.0f) return true;
    
    // Vector to other boid
    ofVec3f toOther = other->position - position;
    
    // Ignore if at same position
    if (toOther.length() <= 0.001f) return true;
    
    // Normalize vectors
    toOther.normalize();
    ofVec3f forward = velocity;
    forward.normalize();
    
    // Angle between directions
    float angle = acos(forward.dot(toOther)) * 180.0f / PI;
    
    // Check if within field of view
    return angle <= fieldOfView / 2.0f;
}

ofVec3f Boid::seek(ofVec3f target) {
    // Desired velocity
    ofVec3f desired = target - position;
    float d = desired.length();
    desired.normalize();
    
    // Slow down if close to target
    if (d < 4.0) {
        float m = ofMap(d, 0, 4.0, 0, maxSpeed);
        desired *= m;
    } else {
        desired *= maxSpeed;
    }
    
    // Steering force
    ofVec3f steer = desired - velocity;
    steer.limit(maxForce);
    
    return steer;
}

ofVec3f Boid::wander() {
    // Random wandering force
    float wanderStrength = 0.3 * energyLevel * uniqueness;
    ofVec3f steer(
        ofRandom(-wanderStrength, wanderStrength),
        ofRandom(-wanderStrength, wanderStrength),
        ofRandom(-wanderStrength, wanderStrength)
    );
    
    return steer;
}

void Boid::flock(vector<Boid*> neighbors) {
    // Calculate steering forces
    ofVec3f newSeparationForce = separate(neighbors);
    ofVec3f newAlignmentForce = align(neighbors);
    ofVec3f newCohesionForce = cohere(neighbors);
    ofVec3f newWanderForce = wander();
    
    // Smooth forces with interpolation (lerp) - add this section
    float smoothFactor = 0.2; // Lower = smoother, higher = more responsive
    separationForce = separationForce.getInterpolated(newSeparationForce, smoothFactor);
    alignmentForce = alignmentForce.getInterpolated(newAlignmentForce, smoothFactor);
    cohesionForce = cohesionForce.getInterpolated(newCohesionForce, smoothFactor);
    wanderForce = wanderForce.getInterpolated(newWanderForce, smoothFactor);
    
    // Weight forces by characteristics
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
    // Apply force as acceleration
    acceleration += force;
}

void Boid::integrate() {
    // Save direction for turn rate
    previousDirection = velocity.getNormalized();
    
    // Update velocity
    velocity += acceleration;
    
    // Limit speed
    float speed = velocity.length();
    
    // Max speed limit
    if (speed > maxSpeed) {
        velocity.normalize();
        velocity *= maxSpeed;
    }
    
    // Min speed limit
    if (speed < minSpeed && speed > 0) {
        velocity.normalize();
        velocity *= minSpeed;
    }
    
    // Turn rate limitation
    if (speed > 0) {
        ofVec3f newDirection = velocity.getNormalized();
        float angle = acos(previousDirection.dot(newDirection)) * 180.0f / PI;
        
        if (angle > turnRate) {
            ofVec3f rotationAxis = previousDirection.getCrossed(newDirection);
            
            if (rotationAxis.length() > 0) {
                rotationAxis.normalize();
                ofQuaternion limitedRotation;
                limitedRotation.makeRotate(turnRate, rotationAxis);
                ofVec3f limitedDirection = previousDirection;
                limitedDirection = limitedRotation * limitedDirection;
                velocity = limitedDirection * speed;
            }
        }
    }
    
    // Update position
    position += velocity;
    
    // Reset acceleration
    acceleration *= 0;
    
    // Update color based on speed
    float speedRatio = ofMap(velocity.length(), minSpeed, maxSpeed, 0, 1);
    ofColor speedColor = ofColor::fromHsb(100 + speedRatio * 100, 200, 255);
    color = personalColor.getLerped(speedColor, 0.3);
}

void Boid::draw(ofMesh* customMesh) {
    ofPushMatrix();
    ofTranslate(position);
    
    // Rotation from velocity
    ofVec3f direction = velocity;
    
    if (direction.length() > 0) {
        direction.normalize();
        ofQuaternion rotation;
        ofVec3f axis(0, 1, 0);
        rotation.makeRotate(axis, direction);
        ofMatrix4x4 mat;
        rotation.get(mat);
        glMultMatrixf(mat.getPtr());
    }
    
    // Draw boid
    ofSetColor(color);
    
    if (customMesh != nullptr && customMesh->getNumVertices() > 0) {
        float meshScaleFactor = 1 * size;
        ofScale(meshScaleFactor, meshScaleFactor, meshScaleFactor);
        customMesh->draw();
    } else {
        float baseSize = 0.2 * size;
        float height = 0.6 * size;
        ofScale(size, size, size);
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
        
        // Draw field of view
        if (fieldOfView < 360.0f && velocity.length() > 0) {
            ofVec3f forward = velocity.getNormalized();
            ofSetColor(200, 200, 0, 100);
            float radius = 1.0;
            int segments = 20;
            float halfFOV = fieldOfView / 2.0f * PI / 180.0f;
            ofQuaternion forwardRotation;
            ofVec3f upVector(0, 1, 0);
            forwardRotation.makeRotate(ofVec3f(0, 0, 1), forward);
            
            for (int i = 0; i < segments; i++) {
                float angle1 = -halfFOV + i * (2 * halfFOV) / (segments - 1);
                float angle2 = -halfFOV + (i + 1) * (2 * halfFOV) / (segments - 1);
                ofVec3f point1(sin(angle1) * radius, 0, cos(angle1) * radius);
                ofVec3f point2(sin(angle2) * radius, 0, cos(angle2) * radius);
                point1 = forwardRotation * point1;
                point2 = forwardRotation * point2;
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
        // Draw force vectors
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
