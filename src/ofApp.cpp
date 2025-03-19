//  CS 134 - Flocking Simulation Project
//
//  Based on Craig Reynolds' Boids algorithm

#include "ofApp.h"

//--------------------------------------------------------------
//  Setup Application data
//
void ofApp::setup(){
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	ofSetVerticalSync(true);
	ofSetFrameRate(60);

	// Setup GUI
	gui.setup();
	
	// Flocking parameters
	gui.add(numBoids.setup("Num Boids", 100, 10, 500));
	gui.add(separationWeight.setup("Separation", 1.5, 0.0, 5.0));
	gui.add(alignmentWeight.setup("Alignment", 1.0, 0.0, 5.0));
	gui.add(cohesionWeight.setup("Cohesion", 1.0, 0.0, 5.0));
	gui.add(neighborhoodRadius.setup("Neighborhood Radius", 3.0, 1.0, 10.0));
	gui.add(separationRadius.setup("Separation Radius", 1.0, 0.5, 5.0));
	gui.add(maxSpeed.setup("Max Speed", 3.0, 1.0, 10.0));
	gui.add(maxForce.setup("Max Force", 0.5, 0.1, 2.0));
	gui.add(damping.setup("Damping", 0.99, 0.9, 1.0));
	gui.add(seekTarget.setup("Seek Target", false));
	
	// Legacy particle system parameters
	gui.add(velocity.setup("Initial Velocity", ofVec3f(0, 20, 0), ofVec3f(-25, -25, -25), ofVec3f(25, 25, 25)));	
	gui.add(lifespan.setup("Lifespan", 5.0, .1, 10.0));
	gui.add(rate.setup("Rate", 1.0, .5, 60.0));
	gui.add(gravity.setup("Gravity", 10, 1, 20));
	gui.add(radius.setup("Radius", .05, .01, .3));
	
	// Add turbulence parameters to GUI
	gui.add(turbulenceMinX.setup("Turb Min X", -1.0, -10.0, 0.0));
	gui.add(turbulenceMaxX.setup("Turb Max X", 1.0, 0.0, 10.0));
	gui.add(turbulenceMinY.setup("Turb Min Y", -1.0, -10.0, 0.0));
	gui.add(turbulenceMaxY.setup("Turb Max Y", 1.0, 0.0, 10.0));
	gui.add(turbulenceMinZ.setup("Turb Min Z", -1.0, -10.0, 0.0));
	gui.add(turbulenceMaxZ.setup("Turb Max Z", 1.0, 0.0, 10.0));
	
	bHide = false;
	
	// Initialize state variables
	simulationRunning = true;
	showFlockingSystem = true;
	showParticleSystem = false;

	// Initialize the flocking system with random boids
	for (int i = 0; i < numBoids; i++) {
		ofVec3f pos = ofVec3f(
			ofRandom(-5, 5),
			ofRandom(-5, 5),
			ofRandom(-5, 5)
		);
		
		ofVec3f vel = ofVec3f(
			ofRandom(-1, 1),
			ofRandom(-1, 1),
			ofRandom(-1, 1)
		);
		
		flockingSystem.addBoid(pos, vel);
	}
	
	// Legacy particle system setup
	if (showParticleSystem) {
		// Create a particle emitter with the particle system
		emitter = ParticleEmitter(&particleSystem);
		
		// Set initial emitter parameters
		emitter.setVelocity((ofVec3f)velocity);
		emitter.setLifespan(lifespan);
		emitter.setRate(rate);
		emitter.setParticleRadius(radius);
		
		// Gravity force
		gravityForce = new GravityForce(ofVec3f(0, -gravity, 0));
		particleSystem.addForce(gravityForce);
		
		// Turbulence force
		turbulenceForce = new TurbulenceForce(
			ofVec3f(turbulenceMinX, turbulenceMinY, turbulenceMinZ),
			ofVec3f(turbulenceMaxX, turbulenceMaxY, turbulenceMaxZ)
		);
		particleSystem.addForce(turbulenceForce);
		
		emitter.start();
	}
}

//--------------------------------------------------------------
//
void ofApp::update() {
	// Update flocking system parameters from GUI
	flockingSystem.separationWeight = separationWeight;
	flockingSystem.alignmentWeight = alignmentWeight;
	flockingSystem.cohesionWeight = cohesionWeight;
	flockingSystem.neighborhoodRadius = neighborhoodRadius;
	flockingSystem.separationRadius = separationRadius;
	flockingSystem.maxSpeed = maxSpeed;
	flockingSystem.maxForce = maxForce;
	flockingSystem.seekTarget = seekTarget;
	
	// If number of boids changed, adjust the flock size
	if (flockingSystem.boids.size() != numBoids) {
		// Add or remove boids as needed
		if (flockingSystem.boids.size() < numBoids) {
			// Add more boids
			int boidsToAdd = numBoids - flockingSystem.boids.size();
			for (int i = 0; i < boidsToAdd; i++) {
				ofVec3f pos = ofVec3f(
					ofRandom(-5, 5),
					ofRandom(-5, 5),
					ofRandom(-5, 5)
				);
				
				ofVec3f vel = ofVec3f(
					ofRandom(-1, 1),
					ofRandom(-1, 1),
					ofRandom(-1, 1)
				);
				
				flockingSystem.addBoid(pos, vel);
			}
		}
		else {
			// Remove excess boids
			while (flockingSystem.boids.size() > numBoids) {
				flockingSystem.removeBoid(flockingSystem.boids.size() - 1);
			}
		}
	}
	
	// Update the flocking system if simulation is running
	if (simulationRunning && showFlockingSystem) {
		flockingSystem.update();
	}
	
	// Legacy particle system update
	if (showParticleSystem) {
		// Update emitter parameters from GUI values
		emitter.setVelocity((ofVec3f)velocity);
		emitter.setLifespan(lifespan);
		emitter.setRate(rate);
		emitter.setParticleRadius(radius);
		
		// Update gravity force
		gravityForce->gravity = ofVec3f(0, -gravity, 0);
		
		// Update turbulence force parameters
		turbulenceForce->set(
			ofVec3f(turbulenceMinX, turbulenceMinY, turbulenceMinZ),
			ofVec3f(turbulenceMaxX, turbulenceMaxY, turbulenceMaxZ)
		);
		
		// Update the emitter
		emitter.update();
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetBackgroundColor(ofColor::black);

	// draw the GUI
	if (!bHide) gui.draw();

	// begin drawing in the camera
	//
	cam.begin();

	// draw a grid
	//
	ofPushMatrix();
	ofRotateDeg(90, 0, 0, 1);
	ofSetLineWidth(1);
	ofSetColor(ofColor::dimGrey);
	ofDrawGridPlane();
	ofPopMatrix();

	// Draw the flocking system
	if (showFlockingSystem) {
		flockingSystem.draw();
	}
	
	// Draw the legacy particle emitter and system
	if (showParticleSystem) {
		emitter.draw();
	}

	//  end drawing in the camera
	// 
	cam.end();

	// draw screen data
	//
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	
	if (showFlockingSystem) {
		str += "\nBoids: " + std::to_string(flockingSystem.boids.size());
	}
	
	if (showParticleSystem) {
		str += "\nParticles: " + std::to_string(particleSystem.particles.size());
	}
	
	str += "\n\nControls:";
	str += "\n  Space: Play/Pause Simulation";
	str += "\n  'c': Toggle Camera Control";
	str += "\n  'f': Toggle Fullscreen";
	str += "\n  'h': Hide/Show GUI";
	str += "\n  '1': Toggle Flocking System";
	str += "\n  '2': Toggle Particle System";
	str += "\n  Left Click: Set Target Position";
	
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 210, 15);
}

// Get the mouse position in 3D world space
ofVec3f ofApp::getMouseWorldPosition() {
	// Get the mouse position in screen coordinates
	float mouseX = ofGetMouseX();
	float mouseY = ofGetMouseY();
	
	// Convert to normalized device coordinates (-1 to 1)
	float ndcX = (mouseX / ofGetWidth()) * 2.0f - 1.0f;
	float ndcY = -((mouseY / ofGetHeight()) * 2.0f - 1.0f);
	
	// Create a ray from the camera position
	glm::vec4 rayStart = glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
	glm::vec4 rayEnd = glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
	
	// Transform ray to world space
	glm::mat4 invMatrix = glm::inverse(cam.getModelViewProjectionMatrix());
	glm::vec4 rayStartWorld = invMatrix * rayStart;
	glm::vec4 rayEndWorld = invMatrix * rayEnd;
	
	// Normalize the vectors by w component
	rayStartWorld /= rayStartWorld.w;
	rayEndWorld /= rayEndWorld.w;
	
	// Calculate ray direction
	glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld) - glm::vec3(rayStartWorld));
	
	// Get camera position in world space
	ofVec3f camPos = cam.getPosition();
	
	// Find intersection with the Z=0 plane
	// P = origin + t * direction, where t is determined by setting z = 0
	// For simplicity, we'll use z=0 as our ground plane
	float t = -camPos.z / rayDir.z;
	glm::vec3 worldPos = glm::vec3(camPos.x, camPos.y, camPos.z) + t * rayDir;
	
	return ofVec3f(worldPos.x, worldPos.y, worldPos.z);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	switch (key) {
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'h':
		bHide = !bHide;
		break;
	case ' ':
		// Toggle simulation running state
		simulationRunning = !simulationRunning;
		break;
	case '1':
		// Toggle flocking system
		showFlockingSystem = !showFlockingSystem;
		break;
	case '2':
		// Toggle particle system
		showParticleSystem = !showParticleSystem;
		if (showParticleSystem && !emitter.started) {
			emitter.start();
		}
		else if (!showParticleSystem && emitter.started) {
			emitter.stop();
		}
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	// Left mouse button - set target position for seeking
	if (button == 0 && seekTarget) {
		// Set target position based on mouse click
		// This is approximate since we're working in 3D space with a 2D input
		ofVec3f worldPos = getMouseWorldPosition();
		flockingSystem.target = worldPos;
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
