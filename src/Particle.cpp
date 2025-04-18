#include "Particle.h"


Particle::Particle() {

	// initialize particle with some reasonable values first;
	//
	velocity.set(0, 0, 0);
	acceleration.set(0, 0, 0);
	position.set(0, 0, 0);
	forces.set(0, 0, 0);
	lifespan = 5;
	birthtime = 0;
	radius = .1;
	damping = .95;
	mass = 1;
	color = ofColor::aquamarine;
}

void Particle::draw() {
	ofSetColor(color);

	float speed = velocity.length();
	
	// Adjust shape of sphere based on speed
	if (speed > 0.05) {
		// Save current transformation matrix
		ofPushMatrix();
		
		// Move to particle position
		ofTranslate(position);
		
		// Stretch factor
		float stretchFactor = ofMap(speed, 0, 20, 1.0, 5.0, true);
		
		// Get velocity directionz
		ofVec3f direction = velocity.getNormalized();
		
		// Calculate rotation to align with movement direction
		ofQuaternion rotation;
		ofVec3f axis(0, 1, 0);
		rotation.makeRotate(axis, direction);
		
		// Apply rotation
		ofMatrix4x4 mat;
		rotation.get(mat);
		glMultMatrixf(mat.getPtr());
		
		// Scale direction of movement
		ofScale(1.0, stretchFactor, 1.0);
		
		// Draw the sphere (scaled)
		ofDrawSphere(0, 0, 0, radius);
		
		// Restore transformation matrix
		ofPopMatrix();
	} 
	else {
		ofDrawSphere(position, radius);
	}
}

// write your own integrator here.. (hint: it's only 3 lines of code)
//
void Particle::integrate() {
    // interval for this step
    float dt = 1.0 / ofGetFrameRate();
    
    // update position based on velocity
    position += (velocity * dt);
    
    // update acceleration with accumulated forces
    ofVec3f accel = acceleration;
    accel += (forces * (1.0 / mass));
    velocity += accel * dt;
    
    // add damping
    velocity *= damping;
    
    // Limit maximum velocity (add this)
    float maxVelocity = 2.0; // Adjust this value as needed
    if (velocity.length() > maxVelocity) {
        velocity.normalize();
        velocity *= maxVelocity;
    }
    
    // clear forces
    forces.set(0, 0, 0);
}
//  return age in seconds
//
float Particle::age() {
	return (ofGetElapsedTimeMillis() - birthtime)/1000.0;
}


