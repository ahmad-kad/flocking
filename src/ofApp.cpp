//  CS 134 - Project 2 - 3D Flocking Simulation
//
//  Kevin M. Smith - CS 134 - SJSU CS

#include "ofApp.h"

//--------------------------------------------------------------
//  Setup Application data
//
void ofApp::setup(){
	// Set up the camera
	cam.setDistance(15);
	cam.setNearClip(0.01);
	cam.setFarClip(1000);
	
	// Setup flocking parameters GUI
	flockingGui.setup("Flocking Parameters");
	flockingGui.add(separationWeight.setup("Separation", 1.5, 0.0, 5.0));
	flockingGui.add(alignmentWeight.setup("Alignment", 1.0, 0.0, 5.0));
	flockingGui.add(cohesionWeight.setup("Cohesion", 1.0, 0.0, 5.0));
	flockingGui.add(maxSpeed.setup("Max Speed", 0.15, 0.05, 1.0));
	flockingGui.add(minSpeed.setup("Min Speed", 0.1, 0.05, 0.5));
	flockingGui.add(maxForce.setup("Max Force", 0.05, 0.01, 0.5));
	flockingGui.add(neighborhoodRadius.setup("Neighborhood", 3.0, 0.5, 10.0));
	flockingGui.add(separationRadius.setup("Sep. Radius", 2.0, 0.5, 5.0));
	flockingGui.add(boundaryWeight.setup("Boundary Force", 1.5, 0.0, 5.0));
	flockingGui.add(boidCount.setup("Boid Count", 30, 1, 200));
	flockingGui.add(targetEnabled.setup("Enable Target", false));
	flockingGui.add(targetWeight.setup("Target Weight", 1.0, 0.0, 5.0));
	
	// Setup boid movement parameters
	flockingGui.add(fieldOfView.setup("Field of View", 240, 90, 360));
	flockingGui.add(turnRate.setup("Turn Rate", 1.5, 0.5, 5.0));
	
	// Setup global system parameters GUI
	globalGui.setup("Global Parameters");
	globalGui.add(flockMode.setup("Flock Mode", 0, 0, 3));
	globalGui.add(individualismFactor.setup("Individualism", 0.2, 0.0, 1.0));
	globalGui.add(systemChaos.setup("System Chaos", 0.05, 0.0, 0.5));
	globalGui.add(boidsVariability.setup("Boid Variability", 0.2, 0.0, 0.8));
	globalGui.add(colorBasedFlocking.setup("Color Flocking", true));
	globalGui.add(colorInfluence.setup("Color Influence", 0.2, 0.0, 1.0));
	globalGui.add(colorSimilarityThreshold.setup("Color Threshold", 0.3, 0.0, 1.0));
	
	// Setup environment parameters GUI
	environmentGui.setup("Environment");
	environmentGui.add(backgroundColor.set("Background", ofColor(30, 30, 50)));
	environmentGui.add(resetButton.setup("Reset Simulation"));
	environmentGui.add(spawnBoidsButton.setup("Spawn Boids"));
	environmentGui.add(spawnCount.setup("Spawn Count", 10, 1, 100));
	environmentGui.add(loadMeshButton.setup("Load Boid Mesh"));
	
	// Setup debug visualization GUI
	debugGui.setup("Debug View");
	debugGui.add(showDebug.setup("Show Debug", false));
	debugGui.add(showVelocities.setup("Show Velocities", false));
	debugGui.add(showNeighborhoods.setup("Show Neighborhoods", false));
	debugGui.add(showForces.setup("Show Forces", false));
	debugGui.add(showGrid.setup("Show Grid", false));
	
    // Setup presets GUI
    presetsGui.setup("Presets");
    
    // Get preset names from config manager
    presetNames = configManager.getPresetNames();
    selectedPresetIndex = 0;
    
    // Add preset UI elements
    presetsGui.add(presetLabel.setup("Saved Presets", ""));
    
    // Add preset name parameter
    if (presetNames.size() > 0) {
        presetNameParam.set("Preset Name", presetNames[0]);
    } else {
        presetNameParam.set("Preset Name", "Default");
    }
    presetsGui.add(presetNameParam);
    
    // Add preset buttons
    presetsGui.add(loadPresetButton.setup("Load Selected"));
    presetsGui.add(savePresetButton.setup("Save Current"));
    
	// Setup event listeners
	resetButton.addListener(this, &ofApp::resetSimulation);
	spawnBoidsButton.addListener(this, &ofApp::spawnBoids);
	loadMeshButton.addListener(this, &ofApp::loadBoidMesh);
    loadPresetButton.addListener(this, &ofApp::loadPreset);
    savePresetButton.addListener(this, &ofApp::savePreset);
	
	bHide = false;
	simulationPaused = false;
	customMeshLoaded = false;
	meshPath = "No mesh loaded";
	
	// Initialize flocking system
	flockSystem.addBoids(boidCount, ofVec3f(0, 0, 0), 5.0);
	
	// Initialize target parameters
	targetPosition = ofVec3f(0, 0, 0);
	targetMoving = false;
	targetPathRadius = 8.0;
	targetTime = 0;
    
    // Position GUI panels
    positionGUIPanels();
    
    // Enable color-based flocking by default
    flockSystem.setColorBasedFlocking(true, 0.2, 1.0);
}

//--------------------------------------------------------------
//
void ofApp::update() {
	// Don't update if simulation is paused
	if (simulationPaused) return;
	
    // Update flocking parameters from GUI
    flockSystem.setParameters(separationWeight, alignmentWeight, cohesionWeight);
    flockSystem.setMaxSpeed(maxSpeed);
    flockSystem.setMinSpeed(minSpeed);
    flockSystem.setMaxForce(maxForce);
    flockSystem.setNeighborhoodRadius(neighborhoodRadius);
    flockSystem.setSeparationRadius(separationRadius);
    flockSystem.boundaryForceWeight = boundaryWeight;
    
    // Update boid movement parameters
    updateBoidParameters();
    
    // Update global parameters
    flockSystem.setIndividualismFactor(individualismFactor);
    flockSystem.setSystemChaos(systemChaos);
    flockSystem.setBoidsVariability(boidsVariability);
    updateFlockMode();
    
    // Update debug visualization settings
    flockSystem.showDebug = showDebug;
    flockSystem.showVelocities = showVelocities;
    flockSystem.showNeighborhoods = showNeighborhoods;
    flockSystem.showForces = showForces;
    flockSystem.showGrid = showGrid;
    
    // Check if we need to add or remove boids
    int currentCount = flockSystem.getCount();
    if (boidCount > currentCount) {
        // Add boids
        flockSystem.addBoids(boidCount - currentCount, ofVec3f(0, 0, 0), 5.0);
    }
    
    // Update target
    if (targetEnabled) {
        // If using moving target
        if (targetMoving) {
            targetTime += ofGetLastFrameTime() * 0.5;
            
            // Move target in circular path
            float x = cos(targetTime) * targetPathRadius;
            float z = sin(targetTime) * targetPathRadius;
            targetPosition = ofVec3f(x, 0, z);
        }
        
        // Set target for flock to follow
        flockSystem.setTarget(targetPosition);
        flockSystem.targetWeight = targetWeight;
    }
    else {
        flockSystem.hasTarget = false;
    }
    
    // Handle strayed boids - remove those that go too far and replace them
    vector<Boid*> boidsToRemove;
    float maxDistance = 30.0; // Maximum allowed distance from origin
    
    for (auto boid : flockSystem.getAllBoids()) {
        if (boid->position.length() > maxDistance) {
            boidsToRemove.push_back(boid);
        }
    }
    
    // If any boids need to be removed, delete them and spawn replacements
    if (!boidsToRemove.empty()) {
        for (auto boid : boidsToRemove) {
            flockSystem.removeBoid(boid);
        }
        
        // Spawn replacement boids
        flockSystem.addBoids(boidsToRemove.size(), ofVec3f(0, 0, 0), 3.0);
    }
    
    // Update the flocking system
    flockSystem.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	// Set background color from GUI
	ofBackground(backgroundColor);

	// Draw the GUI panels if not hidden
	if (!bHide) {
		flockingGui.draw();
		globalGui.draw();
		environmentGui.draw();
		debugGui.draw();
        presetsGui.draw();
	}

	// Begin drawing in the camera
	cam.begin();

	// Draw a grid
	ofPushMatrix();
	ofRotateDeg(90, 0, 0, 1);
	ofSetLineWidth(1);
	ofSetColor(ofColor::dimGrey);
	ofDrawGridPlane();
	ofPopMatrix();

	// Draw the flocking system - use the boid mesh from the model if available
	if (customMeshLoaded) {
		flockSystem.draw(&boidMesh);
	} else {
		flockSystem.draw(nullptr);
	}
	
	// Draw target if enabled
	if (targetEnabled) {
		ofPushStyle();
		ofSetColor(ofColor::red);
		ofDrawSphere(targetPosition, 0.3);
		ofPopStyle();
	}

	// End drawing in the camera 
	cam.end();

	// Draw screen data
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	str += "\nBoids: " + std::to_string(flockSystem.getCount());
	str += "\nMode: " + getFlockModeDescription(flockMode);
	str += simulationPaused ? "\nSIMULATION PAUSED" : "";
	str += colorBasedFlocking ? "\nCOLOR FLOCKING ENABLED" : "";
	
	if (customMeshLoaded) {
		str += "\nMesh: " + ofFilePath::getFileName(meshPath);
	} else {
		str += "\nMesh: Default cone";
	}
	
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 250, 15);
    
    // Draw preset selection if not hidden
    if (!bHide && presetNames.size() > 0) {
        // Draw preset selector
        ofPushStyle();
        ofSetColor(ofColor::white);
        
        int x = presetsGui.getPosition().x;
        int y = presetsGui.getPosition().y + presetsGui.getHeight() + 10;
        
        ofDrawBitmapString("Select Preset:", x, y);
        
        for (int i = 0; i < presetNames.size(); i++) {
            if (i == selectedPresetIndex) {
                ofSetColor(ofColor::yellow);
            } else {
                ofSetColor(ofColor::white);
            }
            
            ofDrawBitmapString(ofToString(i + 1) + ": " + presetNames[i], x, y + 15 * (i + 1));
        }
        
        ofPopStyle();
    }
    
    // Draw help text
    if (!bHide) {
        ofSetColor(ofColor::white);
        string helpText = "Controls:\n";
        helpText += "h: Toggle GUI\n";
        helpText += "f: Toggle fullscreen\n";
        helpText += "c: Toggle camera controls\n";
        helpText += "SPACE: Pause/Play simulation\n";
        helpText += "r: Reset simulation\n";
        helpText += "b: Spawn 10 new boids at origin\n";
        helpText += "t: Place target at mouse position\n";
        helpText += "m: Toggle moving target\n";
        helpText += "d: Toggle debug view\n";
        helpText += "e: Toggle color-based flocking\n";
        helpText += "1-4: Change flock mode";
        
        ofDrawBitmapString(helpText, 10, ofGetHeight() - 150);
    }
}

//--------------------------------------------------------------
void ofApp::resetSimulation() {
	// Clear all boids
	flockSystem.clear();
	
	// Add new boids based on current count setting
	flockSystem.addBoids(boidCount, ofVec3f(0, 0, 0), 5.0);
	
	// Reset target
	targetPosition = ofVec3f(0, 0, 0);
	targetMoving = false;
}

//--------------------------------------------------------------
void ofApp::spawnBoids() {
	// Add specified number of boids at position
	flockSystem.addBoids(spawnCount, ofVec3f(0, 0, 0), 3.0);
	
	// Update the boid count slider
	boidCount = flockSystem.getCount();
}

//--------------------------------------------------------------
void ofApp::loadBoidMesh() {
	// Open file dialog
	ofFileDialogResult result = ofSystemLoadDialog("Select 3D model file", false, ofFilePath::getCurrentExeDir());
	if (result.bSuccess) {
		string path = result.getPath();
		loadMeshFromPath(path);
	}
}

//--------------------------------------------------------------
void ofApp::loadMeshFromPath(const string& path) {
    // Reset the mesh state
    customMeshLoaded = false;
    
    // Load mesh file using Assimp
    if (ofFile::doesFileExist(path)) {
        ofLogNotice() << "Attempting to load model: " << path;
        
        try {
            // Load the model using Assimp which supports multiple formats (obj, ply, fbx, etc.)
            // Use load() instead of loadModel() which is deprecated
            bool loaded = boidModel.load(path, true); // true = optimize mesh
            
            if (loaded) {
                ofLogNotice() << "Successfully loaded model with " << boidModel.getMeshCount() << " meshes";
                
                // For compatibility with existing code, extract the first mesh
                if (boidModel.getMeshCount() > 0) {
                    // Get the first mesh from the model
                    boidMesh = boidModel.getMesh(0);
                    
                    // Normalize and center the model
                    boidModel.setScale(100,1,1); // Scale down the model
                    // boidModel.setScaleNormalization(true); // Normalize size
                    
                    meshPath = path;
                    customMeshLoaded = true;
                    ofLogNotice() << "Successfully loaded model: " << path;
                } else {
                    ofLogError() << "Model loaded but contains no meshes: " << path;
                    createDefaultMesh();
                }
            } else {
                ofLogError() << "Failed to load model: " << path;
                createDefaultMesh();
            }
        } catch (const std::exception& e) {
            ofLogError() << "Exception loading model: " << e.what();
            createDefaultMesh();
        } catch (...) {
            ofLogError() << "Unknown error loading model";
            createDefaultMesh();
        }
    } else {
        ofLogError() << "File does not exist: " << path;
    }
}

// Helper method to create a default cone mesh when loading fails
void ofApp::createDefaultMesh() {
    boidMesh = ofMesh::cone(0.2, 0.6, 12, 1);
    meshPath = "Default cone (loading failed)";
    customMeshLoaded = true;
}

//--------------------------------------------------------------
void ofApp::loadPreset() {
    // Get selected preset
    FlockingPreset preset;
    
    if (configManager.loadPresetByIndex(selectedPresetIndex, preset)) {
        // Update UI with preset values
        updateUIFromPreset(preset);
    }
}

//--------------------------------------------------------------
void ofApp::savePreset() {
    // Create preset from current UI values
    FlockingPreset preset;
    updatePresetFromUI(preset);
    
    // Use name from parameter
    string name = presetNameParam.get();
    preset.name = name;
    
    // Save preset
    configManager.savePreset(name, preset);
    
    // Update preset names
    presetNames = configManager.getPresetNames();
    
    // Select the saved preset
    for (int i = 0; i < presetNames.size(); i++) {
        if (presetNames[i] == name) {
            selectedPresetIndex = i;
            break;
        }
    }
}

//--------------------------------------------------------------
void ofApp::updateUIFromPreset(const FlockingPreset& preset) {
    // Update basic parameters
    separationWeight = preset.separationWeight;
    alignmentWeight = preset.alignmentWeight;
    cohesionWeight = preset.cohesionWeight;
    maxSpeed = preset.maxSpeed;
    minSpeed = preset.minSpeed;
    maxForce = preset.maxForce;
    neighborhoodRadius = preset.neighborhoodRadius;
    separationRadius = preset.separationRadius;
    boundaryWeight = preset.boundaryWeight;
    
    // Update global parameters
    flockMode = preset.flockMode;
    individualismFactor = preset.individualismFactor;
    systemChaos = preset.systemChaos;
    boidsVariability = preset.boidsVariability;
    
    // Update environment parameters
    backgroundColor = preset.backgroundColor;
}

//--------------------------------------------------------------
void ofApp::updatePresetFromUI(FlockingPreset& preset) {
    // Copy basic parameters
    preset.separationWeight = separationWeight;
    preset.alignmentWeight = alignmentWeight;
    preset.cohesionWeight = cohesionWeight;
    preset.maxSpeed = maxSpeed;
    preset.minSpeed = minSpeed;
    preset.maxForce = maxForce;
    preset.neighborhoodRadius = neighborhoodRadius;
    preset.separationRadius = separationRadius;
    preset.boundaryWeight = boundaryWeight;
    
    // Copy global parameters
    preset.flockMode = flockMode;
    preset.individualismFactor = individualismFactor;
    preset.systemChaos = systemChaos;
    preset.boidsVariability = boidsVariability;
    
    // Copy environment parameters
    preset.backgroundColor = backgroundColor;
}

//--------------------------------------------------------------
void ofApp::positionGUIPanels() {
    int guiWidth = 200;
    int padding = 10;
    
    // Left side panels - top to bottom
    flockingGui.setPosition(padding, padding);
    debugGui.setPosition(padding, flockingGui.getPosition().y + flockingGui.getHeight() + padding);
    
    // Right side panels - top to bottom
    globalGui.setPosition(ofGetWidth() - guiWidth - padding, padding);
    environmentGui.setPosition(ofGetWidth() - guiWidth - padding, globalGui.getPosition().y + globalGui.getHeight() + padding);
    
    // Middle panels - left to right
    presetsGui.setPosition(flockingGui.getPosition().x + flockingGui.getWidth() + padding, padding);
}

//--------------------------------------------------------------
void ofApp::updateFlockMode() {
    if (flockMode == 0) { // Calm
        flockSystem.setSeparationRadius(2.0);
        flockSystem.setNeighborhoodRadius(3.0);
        flockSystem.setBounds(ofVec3f(-20, -20, -20), ofVec3f(20, 20, 20));
        flockSystem.boundaryForceWeight = 1.0;
        flockSystem.boundaryDistance = 5.0;
        flockSystem.setSystemChaos(0.1);
        flockSystem.setBoidsVariability(0.3);
        flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0); // Fixed parameter order and full color influence
    }
    else if (flockMode == 1) { // Excited
        flockSystem.setSeparationRadius(1.5);
        flockSystem.setNeighborhoodRadius(5.0);
        flockSystem.setBounds(ofVec3f(-30, -30, -30), ofVec3f(30, 30, 30));
        flockSystem.boundaryForceWeight = 1.5;
        flockSystem.boundaryDistance = 5.0;
        flockSystem.setSystemChaos(0.3);
        flockSystem.setBoidsVariability(0.5);
        flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0); // Fixed parameter order and full color influence
    }
    else if (flockMode == 2) { // Chaotic
        flockSystem.setSeparationRadius(1.0);
        flockSystem.setNeighborhoodRadius(8.0);
        flockSystem.setBounds(ofVec3f(-40, -40, -40), ofVec3f(40, 40, 40));
        flockSystem.boundaryForceWeight = 0.8;
        flockSystem.boundaryDistance = 5.0;
        flockSystem.setSystemChaos(0.6);
        flockSystem.setBoidsVariability(0.8);
        flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0); // Fixed parameter order and full color influence
    }
}

//--------------------------------------------------------------
string ofApp::getFlockModeDescription(int mode) {
    switch (mode) {
        case 0:
            return "Calm";
        case 1:
            return "Excited";
        case 2:
            return "Chaotic";
        case 3:
            return "Predator-Prey";
        default:
            return "Unknown";
    }
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
		// Toggle simulation pause/play
		simulationPaused = !simulationPaused;
		break;
	case 'r':
	case 'R':
		// Reset simulation
		resetSimulation();
		break;
	case 'b':
	case 'B':
		// Spawn new boids
		spawnBoids();
		break;
	case 'd':
	case 'D':
		// Toggle debug view
		showDebug = !showDebug;
		break;
    case 'e':
    case 'E':
        // Toggle color-based flocking
        colorBasedFlocking = !colorBasedFlocking;
        break;
	case '1':
	case '2':
	case '3':
	case '4':
        // Check if we're in preset selection mode (GUI not hidden and presets shown)
        if (!bHide && key >= '1' && key <= '0' + presetNames.size()) {
            // Select preset
            selectedPresetIndex = key - '1';
            
            // Update preset name in UI
            presetNameParam = presetNames[selectedPresetIndex];
        } else {
            // Change flock mode instead
            flockMode = key - '1'; // Map 1-4 to 0-3
        }
		break;
	case 't':
		{
			// Place target at current mouse position (projected into 3D space)
			// Get the current mouse position
			glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(ofGetMouseX(), ofGetMouseY(), 0));
			glm::vec3 mouseDirection = glm::normalize(mouseWorld - cam.getPosition());
			
			// Calculate intersection with XZ plane (y=0)
			float t = -cam.getPosition().y / mouseDirection.y;
			glm::vec3 planeIntersection = cam.getPosition() + mouseDirection * t;
			
			// Set target position
			targetPosition = ofVec3f(planeIntersection.x, 0, planeIntersection.z);
			targetEnabled = true;
			targetMoving = false;
		}
		break;
	case 'm':
		{
			// Toggle moving target
			targetMoving = !targetMoving;
			if (targetMoving) {
				targetEnabled = true;
			}
		}
		break;
    case 'l':
        // Load selected preset
        loadPreset();
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
    // Check if clicking on a preset in the list
    if (!bHide && presetNames.size() > 0) {
        int presetsX = presetsGui.getPosition().x;
        int presetsY = presetsGui.getPosition().y + presetsGui.getHeight() + 10;
        
        // Check if mouse is in the preset selection area
        if (x >= presetsX && x < presetsX + 200) {
            for (int i = 0; i < presetNames.size(); i++) {
                int itemY = presetsY + 15 * (i + 1);
                if (y >= itemY - 12 && y < itemY + 3) {
                    // Selected this preset
                    selectedPresetIndex = i;
                    presetNameParam = presetNames[selectedPresetIndex];
                    break;
                }
            }
        }
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
	// Reposition GUI panels
    positionGUIPanels();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
	// If files are dragged onto the window, try to load the first one as a mesh
	if (!dragInfo.files.empty()) {
		string path = dragInfo.files[0];
		loadMeshFromPath(path);
	}
}

// Add new method to update boid parameters
void ofApp::updateBoidParameters() {
    // Update field of view and turn rate for all boids
    for (auto boid : flockSystem.getAllBoids()) {
        boid->fieldOfView = fieldOfView;
        boid->turnRate = turnRate;
    }
}
