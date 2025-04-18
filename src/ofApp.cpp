//  CS 134 - Project 2 - 3D Flocking Simulation
//  Kevin M. Smith - CS 134 - SJSU CS

#include "ofApp.h"

//--------------------------------------------------------------
//  Setup application data
//
void ofApp::setup(){
	// Set shader pointers
	ofSetVerticalSync(true);
	ofEnableDepthTest();
	ofSetFrameRate(60);

	// Initialize UI 
	bHide = true; // Hide UI by default
	
	// Set up lights
	ofEnableSmoothing();
	ofEnableLighting();
	ofEnableDepthTest();
	
	// Configure GUI
	flockingGui.setup("Flocking", "flocking_settings.xml");
	flockingGui.add(separationWeight.setup("Separation", 1.5f, 0.0f, 5.0f));
	flockingGui.add(alignmentWeight.setup("Alignment", 1.0f, 0.0f, 5.0f));
	flockingGui.add(cohesionWeight.setup("Cohesion", 1.0f, 0.0f, 5.0f));
	flockingGui.add(maxSpeed.setup("Max Speed", 1.0f, 0.1f, 5.0f)); // Reduced max speed
	flockingGui.add(minSpeed.setup("Min Speed", 0.2f, 0.0f, 1.0f)); // Reduced min speed
	flockingGui.add(maxForce.setup("Max Force", 0.1f, 0.01f, 1.0f));
	flockingGui.add(neighborhoodRadius.setup("Neighborhood Radius", 10.0f, 1.0f, 50.0f));
	flockingGui.add(separationRadius.setup("Separation Radius", 5.0f, 1.0f, 10.0f));
	flockingGui.add(boundaryWeight.setup("Boundary Weight", 1.0f, 0.0f, 5.0f));
	flockingGui.add(boidCount.setup("Boid Count", 35, 1, 200));
	flockingGui.add(targetEnabled.setup("Enable Target", false));
	flockingGui.add(targetWeight.setup("Target Weight", 1.0, 0.0, 5.0));
	
	// Boid movement parameters
	flockingGui.add(fieldOfView.setup("Field of View", 240, 90, 360));
	flockingGui.add(turnRate.setup("Turn Rate", 0.5, 0.2, 2.0));
	
	// Global system parameters GUI
	globalGui.setup("Global Parameters");
	globalGui.add(flockMode.setup("Flock Mode", 0, 0, 3));
	globalGui.add(individualismFactor.setup("Individualism", 0.2, 0.0, 1.0));
	globalGui.add(systemChaos.setup("System Chaos", 0.05, 0.0, 0.5));
	globalGui.add(boidsVariability.setup("Boid Variability", 0.2, 0.0, 0.8));
	globalGui.add(colorBasedFlocking.setup("Color Flocking", true));
	globalGui.add(colorInfluence.setup("Color Influence", 0.2, 0.0, 1.0));
	globalGui.add(colorSimilarityThreshold.setup("Color Threshold", 0.3, 0.0, 1.0));
	
	// Environment parameters GUI
	environmentGui.setup("Environment");
	environmentGui.add(backgroundColor.set("Background", ofColor(30, 30, 50)));
	environmentGui.add(resetButton.setup("Reset Simulation"));
	environmentGui.add(spawnBoidsButton.setup("Spawn Boids"));
	environmentGui.add(spawnCount.setup("Spawn Count", 10, 1, 100));
	environmentGui.add(loadMeshButton.setup("Load Boid Mesh"));
	environmentGui.add(showTerrainDebug.setup("Show Terrain Debug", false));
	
	// Debug visualization GUI
	debugGui.setup("Debug View");
	debugGui.add(showDebug.setup("Show Debug", false));
	debugGui.add(showVelocities.setup("Show Velocities", false));
	debugGui.add(showNeighborhoods.setup("Show Neighborhoods", false));
	debugGui.add(showForces.setup("Show Forces", false));
	debugGui.add(showGrid.setup("Show Grid", false));
	debugGui.add(showFood.setup("Show Food", true));
	
	// Food system GUI
	foodSystemGui.setup("Food System");
	foodSystemGui.add(plantFoodCount.setup("Plant Food", 50, 0, 200));
	foodSystemGui.add(insectSwarmCount.setup("Insect Swarms", 30, 0, 100));
	foodSystemGui.add(plantRegenerationRate.setup("Plant Regen Rate", 1.0, 0.1, 3.0));
	foodSystemGui.add(insectRegenerationRate.setup("Insect Regen Rate", 1.5, 0.1, 3.0));
	foodSystemGui.add(resetFoodButton.setup("Reset Food System"));
	
	// Presets GUI
	presetsGui.setup("Presets");
	
	// Get preset names
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
	
	// Event listeners
	resetButton.addListener(this, &ofApp::resetSimulation);
	spawnBoidsButton.addListener(this, &ofApp::spawnBoids);
	loadMeshButton.addListener(this, &ofApp::loadBoidMesh);
	loadPresetButton.addListener(this, &ofApp::loadPreset);
	savePresetButton.addListener(this, &ofApp::savePreset);
	resetFoodButton.addListener(this, &ofApp::resetFoodSystem);
	
	// Camera setup
	cam.setDistance(15);
	cam.setNearClip(0.01);
	cam.setFarClip(1000);
	
	// Initialize flocking system
	flockSystem.addBoids(boidCount, ofVec3f(0, 0, 0), 5.0);
	
	// Initialize terrain system
	terrainSystem.setup(256, 256, 0.5);
	terrainSystem.generateTerrain(10.0f, 0.5f);
	
	// Set terrain in flock system
	flockSystem.setTerrainSystem(&terrainSystem);
	
	// Initialize food manager with terrain reference
	foodManager.setTerrain(&terrainSystem);
	
	// Setup food system with initial food
	resetFoodSystem();
	
	// Initialize target parameters
	targetPosition = ofVec3f(0, 0, 0);
	targetMoving = false;
	targetPathRadius = 8.0;
	targetTime = 0;
	
	// Legend GUI
	legendGui.setup("Legend");
	legendGui.add(legendTitle.setup("Entity Types", ""));
	legendGui.add(showLegend.setup("Show Legend", true));
	
	// Position GUI panels
	positionGUIPanels();
	
	// Enable color-based flocking by default
	flockSystem.setColorBasedFlocking(true, 0.2, 1.0);
}

//--------------------------------------------------------------
//
void ofApp::update() {
	// Skip update if paused
	if (simulationPaused) return;
	
	// Update flocking parameters
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
	
	// Update debug settings
	flockSystem.showDebug = showDebug;
	flockSystem.showVelocities = showVelocities;
	flockSystem.showNeighborhoods = showNeighborhoods;
	flockSystem.showForces = showForces;
	flockSystem.showGrid = showGrid;
	
	// Check for boid count adjustments
	int currentCount = flockSystem.getCount();
	if (boidCount > currentCount) {
		// Add boids
		flockSystem.addBoids(boidCount - currentCount, ofVec3f(0, 0, 0), 5.0);
	}
	
	// Update target
	if (targetEnabled) {
		// Move target if enabled
		if (targetMoving) {
			targetTime += ofGetLastFrameTime() * 0.5;
			
			// Circular target movement
			float x = cos(targetTime) * targetPathRadius;
			float z = sin(targetTime) * targetPathRadius;
			
			// Set target height based on terrain
			float y = terrainSystem.getHeightAt(x, z) + 2.0f; // Hover above terrain
			targetPosition = ofVec3f(x, y, z);
		}
		
		// Set target for flock
		flockSystem.setTarget(targetPosition);
		flockSystem.targetWeight = targetWeight;
	}
	else {
		flockSystem.hasTarget = false;
	}
	
	// Handle strayed boids
	vector<Boid*> boidsToRemove;
	float maxDistance = 30.0; // Max distance from origin
	
	for (auto boid : flockSystem.getAllBoids()) {
		if (boid->position.length() > maxDistance) {
			boidsToRemove.push_back(boid);
		}
		
		// Update boid height based on terrain if it's a terrestrial creature
		if (boid->movementType == MovementType::TERRESTRIAL) {
			float terrainHeight = terrainSystem.getHeightAt(boid->position.x, boid->position.z);
			
			// Softly adjust terrestrial boids to terrain height
			float targetHeight = terrainHeight + (boid->size * 0.5f);
			boid->position.y = ofLerp(boid->position.y, targetHeight, 0.3f);
			
			// Modify velocity to reduce vertical component (help keep them moving horizontally)
			if (abs(boid->velocity.y) > 0.2f) {
				boid->velocity.y *= 0.8f;
			}
		} else if (boid->movementType == MovementType::AERIAL) {
			// For aerial creatures, enforce minimum height above terrain
			float terrainHeight = terrainSystem.getHeightAt(boid->position.x, boid->position.z);
			float minHeight = terrainHeight + 2.0f;
			
			if (boid->position.y < minHeight) {
				// If too close to ground, add upward force
				boid->position.y = ofLerp(boid->position.y, minHeight + 1.0f, 0.1f);
				if (boid->velocity.y < 0) {
					boid->velocity.y *= -0.5f; // Reverse downward motion
				}
			}
		}
		
		// Search for food if needed
		if (boid->lifecycle.hungerLevel > 0.5f) {
			// Only search for food at intervals to reduce computational load
			if (ofRandom(1.0) < 0.05) {
				std::vector<FoodSource*> accessibleFood = foodManager.findAccessibleFood(boid, neighborhoodRadius * 2.0f);
				
				if (!accessibleFood.empty()) {
					// Get the closest food
					FoodSource* closestFood = nullptr;
					float minDist = FLT_MAX;
					
					for (auto* food : accessibleFood) {
						float dist = boid->position.distance(food->position);
						if (dist < minDist) {
							minDist = dist;
							closestFood = food;
						}
					}
					
					// If close enough, consume the food
					if (minDist < boid->radius * 2.0f) {
						foodManager.consumeFood(boid, closestFood);
					}
					else {
						// Otherwise, set it as the target for seeking behavior
						boid->currentFood = closestFood;
						boid->behaviorState = BehaviorState::FEEDING;
					}
				}
			}
		}
		else {
			// Reset feeding state if no longer hungry
			if (boid->behaviorState == BehaviorState::FEEDING) {
				boid->behaviorState = BehaviorState::NORMAL;
				boid->currentFood = nullptr;
			}
		}
	}
	
	// Remove and replace boids if needed
	if (!boidsToRemove.empty()) {
		for (auto boid : boidsToRemove) {
			flockSystem.removeBoid(boid);
		}
		
		// Spawn replacements
		flockSystem.addBoids(boidsToRemove.size(), ofVec3f(0, 0, 0), 3.0);
	}
	
	// Update food system parameters
	foodManager.plantRegenerationRate = plantRegenerationRate;
	foodManager.insectRegenerationRate = insectRegenerationRate;
	
	// Update food system
	foodManager.update(ofGetLastFrameTime());
	
	// Update the flocking system
	flockSystem.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	// Set background color
	ofBackground(backgroundColor);

	// Draw GUI panels if visible
	if (!bHide) {
		flockingGui.draw();
		globalGui.draw();
		environmentGui.draw();
		debugGui.draw();
		presetsGui.draw();
		foodSystemGui.draw();
		legendGui.draw();
	}

	// Begin camera drawing
	cam.begin();
	ofEnableDepthTest();

	// Draw terrain
	terrainSystem.draw();
	if (showTerrainDebug) {
		terrainSystem.drawDebug();
	}

	// Draw food
	if (showFood) {
		foodManager.draw();
	}

	// Draw flocking system
	if (customMeshLoaded) {
		flockSystem.draw(&boidMesh);
	} else {
		flockSystem.draw();
	}
	
	// Draw target if enabled
	if (targetEnabled) {
		ofPushStyle();
		ofSetColor(255, 100, 100);
		ofDrawSphere(targetPosition, 0.5);
		ofSetColor(255, 100, 100, 100);
		ofDrawSphere(targetPosition, 1.0);
		ofPopStyle();
	}

	ofDisableDepthTest();
	cam.end();

	// Draw the legend if enabled
	if (showLegend) {
		drawLegend();
	}

	// Always draw minimal status info
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	str += "\nBoids: " + std::to_string(flockSystem.getCount());
	str += "\nMode: " + getFlockModeDescription(flockMode);
	str += simulationPaused ? "\nSIMULATION PAUSED" : "";
	
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 250, 15);
	
	// Draw preset selection if visible
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
	
	// Always show minimal help text if UI is hidden
	if (bHide) {
		ofSetColor(ofColor::white);
		string minimalHelp = "Press 'h' to show UI";
		ofDrawBitmapString(minimalHelp, 10, 20);
	}
	
	// Draw complete help text if showing UI
	else {
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
	// Clear boids
	flockSystem.clear();
	
	// Add new boids
	flockSystem.addBoids(boidCount, ofVec3f(0, 0, 0), 5.0);
	
	// Reset food system
	resetFoodSystem();
	
	// Reset terrain
	terrainSystem.generateTerrain(10.0f, 0.5f);
	
	// Reset target
	targetPosition = ofVec3f(0, 0, 0);
	targetTime = 0;
	
	// Apply speed adjustments to the newly created boids
	for (auto boid : flockSystem.getAllBoids()) {
		// Adjust speeds based on movement type
		if (boid->movementType == MovementType::AERIAL) {
			boid->maxSpeed *= 0.6f; // Slower for aerial to prevent excessive speed
			boid->minSpeed *= 0.7f;
		} else if (boid->movementType == MovementType::TERRESTRIAL) {
			boid->maxSpeed *= 0.8f; // Slightly slower for terrestrial
			boid->minSpeed *= 0.8f;
		}
		
		// Adjust speeds based on trophic level
		if (boid->trophicLevel == TrophicLevel::APEX_PREDATOR) {
			// Apex predators are a bit faster
			boid->maxSpeed *= 1.1f;
		} else if (boid->trophicLevel == TrophicLevel::PREY) {
			// Prey are faster when fleeing but default to normal speed
			boid->maxSpeed *= 0.9f;
		}
		
		// Ensure valid position above terrain
		if (terrainSystem.isInitialized()) {
			float terrainHeight = terrainSystem.getHeightAt(boid->position.x, boid->position.z);
			
			// Check if boid is underground
			if (boid->position.y < terrainHeight + 0.5f) {
				// Move boid above ground based on movement type
				if (boid->movementType == MovementType::AERIAL) {
					boid->position.y = terrainHeight + 3.0f + ofRandom(2.0f); // Aerial above ground
				} else {
					boid->position.y = terrainHeight + 0.5f + ofRandom(0.5f); // Just above ground
				}
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::spawnBoids() {
	// Add specified number of boids
	flockSystem.addBoids(spawnCount, ofVec3f(0, 0, 0), 3.0);
	
	// Update boid count slider
	boidCount = flockSystem.getCount();
	
	// Adjust speeds and parameters for all boids
	for (auto boid : flockSystem.getAllBoids()) {
		// Adjust speeds based on movement type
		if (boid->movementType == MovementType::AERIAL) {
			boid->maxSpeed *= 0.6f; // Slower for aerial to prevent excessive speed
			boid->minSpeed *= 0.7f;
		} else if (boid->movementType == MovementType::TERRESTRIAL) {
			boid->maxSpeed *= 0.8f; // Slightly slower for terrestrial
			boid->minSpeed *= 0.8f;
		}
		
		// Adjust speeds based on trophic level
		if (boid->trophicLevel == TrophicLevel::APEX_PREDATOR) {
			// Apex predators are a bit faster
			boid->maxSpeed *= 1.1f;
		} else if (boid->trophicLevel == TrophicLevel::PREY) {
			// Prey are faster when fleeing but default to normal speed
			boid->maxSpeed *= 0.9f;
		}
		
		// Ensure valid position above terrain
		if (terrainSystem.isInitialized()) {
			float terrainHeight = terrainSystem.getHeightAt(boid->position.x, boid->position.z);
			
			// Check if boid is underground
			if (boid->position.y < terrainHeight + 0.5f) {
				// Move boid above ground based on movement type
				if (boid->movementType == MovementType::AERIAL) {
					boid->position.y = terrainHeight + 3.0f + ofRandom(2.0f); // Aerial above ground
				} else {
					boid->position.y = terrainHeight + 0.5f + ofRandom(0.5f); // Just above ground
				}
			}
		}
	}
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
	// Reset mesh state
	customMeshLoaded = false;
	
	// Load mesh file
	if (ofFile::doesFileExist(path)) {
		ofLogNotice() << "Attempting to load model: " << path;
		
		try {
			// Load model using Assimp
			bool loaded = boidModel.load(path, ofxAssimpModelLoader::OPTIMIZE_DEFAULT); // Fix deprecated load call
			
			if (loaded) {
				ofLogNotice() << "Successfully loaded model with " << boidModel.getMeshCount() << " meshes";
				
				// Extract the first mesh
				if (boidModel.getMeshCount() > 0) {
					boidMesh = boidModel.getMesh(0);
					
					// Scale the mesh vertices directly
					for (int i = 0; i < boidMesh.getNumVertices(); i++) {
						ofVec3f vertex = boidMesh.getVertex(i);
						vertex *= 10.0; // Apply a large scale factor
						boidMesh.setVertex(i, vertex);
					}
					
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
	// Update parameters
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
	// Copy parameters
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
	
	// Left side panels
	flockingGui.setPosition(padding, padding);
	debugGui.setPosition(padding, flockingGui.getPosition().y + flockingGui.getHeight() + padding);
	legendGui.setPosition(padding, debugGui.getPosition().y + debugGui.getHeight() + padding);
	
	// Right side panels
	globalGui.setPosition(ofGetWidth() - guiWidth - padding, padding);
	environmentGui.setPosition(ofGetWidth() - guiWidth - padding, globalGui.getPosition().y + globalGui.getHeight() + padding);
	
	// Middle panels
	presetsGui.setPosition(flockingGui.getPosition().x + flockingGui.getWidth() + padding, padding);
	foodSystemGui.setPosition(presetsGui.getPosition().x + presetsGui.getWidth() + padding, padding);
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
		flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0);
	}
	else if (flockMode == 1) { // Excited
		flockSystem.setSeparationRadius(1.5);
		flockSystem.setNeighborhoodRadius(5.0);
		flockSystem.setBounds(ofVec3f(-30, -30, -30), ofVec3f(30, 30, 30));
		flockSystem.boundaryForceWeight = 1.5;
		flockSystem.boundaryDistance = 5.0;
		flockSystem.setSystemChaos(0.3);
		flockSystem.setBoidsVariability(0.5);
		flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0);
	}
	else if (flockMode == 2) { // Chaotic
		flockSystem.setSeparationRadius(1.0);
		flockSystem.setNeighborhoodRadius(8.0);
		flockSystem.setBounds(ofVec3f(-40, -40, -40), ofVec3f(40, 40, 40));
		flockSystem.boundaryForceWeight = 0.8;
		flockSystem.boundaryDistance = 5.0;
		flockSystem.setSystemChaos(0.6);
		flockSystem.setBoidsVariability(0.8);
		flockSystem.setColorBasedFlocking(colorBasedFlocking, colorSimilarityThreshold, 1.0);
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
	// Toggle UI with 'h' key
	if (key == 'h') {
		bHide = !bHide;
	}
	
	// Toggle mouse input mode with 'm'
	else if (key == 'm') {
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
	}
	
	// Toggle fullscreen with 'f'
	else if (key == 'f') {
		ofToggleFullscreen();
	}
	
	// Pause/play with spacebar
	else if (key == ' ') {
		simulationPaused = !simulationPaused;
	}
	
	// Reset simulation with 'r'
	else if (key == 'r') {
		resetSimulation();
	}
	
	// Spawn new boids with 'b'
	else if (key == 'b') {
		spawnBoids();
	}
	
	// Toggle debug view with 'd'
	else if (key == 'd') {
		showDebug = !showDebug;
	}
	
	// Toggle debug view with 't'
	else if (key == 't') {
		showTerrainDebug = !showTerrainDebug;
	}
	
	// Toggle color-based flocking with 'e'
	else if (key == 'e') {
		colorBasedFlocking = !colorBasedFlocking;
	}
	
	// Select presets with number keys
	else if (key >= '1' && key <= '9') {
		int presetIndex = key - '1';
		if (!bHide && presetIndex < presetNames.size()) {
			selectedPresetIndex = presetIndex;
			presetNameParam = presetNames[selectedPresetIndex];
			loadPreset();
		} else {
			// Change flock mode
			if (presetIndex < 4) {
				flockMode = presetIndex; // Map 1-4 to 0-3
			}
		}
	}
	
	// Place target at mouse position
	else if (key == 'p') {
		glm::vec3 mouseWorld = cam.screenToWorld(glm::vec3(ofGetMouseX(), ofGetMouseY(), 0));
		glm::vec3 mouseDirection = glm::normalize(mouseWorld - cam.getPosition());
		
		// Calculate intersection with XZ plane
		float t = -cam.getPosition().y / mouseDirection.y;
		glm::vec3 planeIntersection = cam.getPosition() + mouseDirection * t;
		
		// Set target position
		targetPosition = ofVec3f(planeIntersection.x, 0, planeIntersection.z);
		targetEnabled = true;
		targetMoving = false;
	}
	
	// Toggle moving target
	else if (key == 'o') {
		targetMoving = !targetMoving;
		if (targetMoving) {
			targetEnabled = true;
		}
	}
	
	// Load selected preset
	else if (key == 'l') {
		loadPreset();
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
	// Check if clicking on a preset
	if (!bHide && presetNames.size() > 0) {
		int presetsX = presetsGui.getPosition().x;
		int presetsY = presetsGui.getPosition().y + presetsGui.getHeight() + 10;
		
		// Check if mouse is in preset selection area
		if (x >= presetsX && x < presetsX + 200) {
			for (int i = 0; i < presetNames.size(); i++) {
				int itemY = presetsY + 15 * (i + 1);
				if (y >= itemY - 12 && y < itemY + 3) {
					// Selected preset
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
	// Load first file as mesh if dragged
	if (!dragInfo.files.empty()) {
		string path = dragInfo.files[0];
		loadMeshFromPath(path);
	}
}

// Update boid parameters
void ofApp::updateBoidParameters() {
	// Update field of view and turn rate for all boids
	for (auto boid : flockSystem.getAllBoids()) {
		boid->fieldOfView = fieldOfView;
		boid->turnRate = turnRate;
	}
}

void ofApp::resetFoodSystem() {
	// Get the simulation bounds from the flock system
	ofVec3f boundsMin, boundsMax;
	flockSystem.getBounds(boundsMin, boundsMax);
	
	// Set bounds for food system
	foodManager.setBounds(boundsMin, boundsMax);
	
	// Clear and reinitialize food sources
	foodManager.setup(plantFoodCount, insectSwarmCount);
}

// Add after the draw method
void ofApp::drawLegend() {
	if (!showLegend) return;
	
	ofPushStyle();
	ofFill();
	
	float x = 10;
	float y = ofGetHeight() - 220;
	float width = 200;
	float height = 210;
	float lineHeight = 20;
	float shapeSize = 10;
	float textPadding = 20;
	
	// Draw background
	ofSetColor(0, 0, 0, 180);
	ofDrawRectangle(x, y, width, height);
	
	// Title
	ofSetColor(255);
	ofDrawBitmapString("ENTITY LEGEND", x + 10, y + 20);
	y += 35;
	
	// Boid Types by Trophic Level
	ofDrawBitmapString("Boid Types:", x + 10, y);
	y += lineHeight;
	
	// Apex Predator
	ofSetColor(200, 50, 50);
	ofDrawCone(x + 10, y, 0, shapeSize * 0.8f, shapeSize * 2.0f);
	ofSetColor(255);
	ofDrawBitmapString("Apex Predator", x + textPadding, y + 5);
	y += lineHeight;
	
	// Mid Predator
	ofSetColor(200, 100, 0);
	ofDrawCone(x + 10, y, 0, shapeSize * 0.8f, shapeSize * 2.0f);
	ofSetColor(255);
	ofDrawBitmapString("Mid Predator", x + textPadding, y + 5);
	y += lineHeight;
	
	// Apex Prey
	ofSetColor(100, 200, 255);
	ofDrawSphere(x + 10, y, shapeSize);
	ofSetColor(255);
	ofDrawBitmapString("Apex Prey", x + textPadding, y + 5);
	y += lineHeight;
	
	// Regular Prey
	ofSetColor(0, 200, 100);
	ofDrawSphere(x + 10, y, shapeSize);
	ofSetColor(255);
	ofDrawBitmapString("Regular Prey", x + textPadding, y + 5);
	y += lineHeight + 5;
	
	// Food Types
	ofDrawBitmapString("Food Types:", x + 10, y);
	y += lineHeight;
	
	// Plant
	ofSetColor(0, 150, 0);
	ofDrawCone(x + 10, y, 0, shapeSize, shapeSize * 2);
	ofSetColor(255);
	ofDrawBitmapString("Plant Food", x + textPadding, y + 5);
	y += lineHeight;
	
	// Insects
	ofSetColor(200, 200, 0);
	ofDrawSphere(x + 10, y, shapeSize * 0.7);
	ofSetColor(255);
	ofDrawBitmapString("Insect Swarm", x + textPadding, y + 5);
	y += lineHeight;
	
	// Behavior states
	ofDrawBitmapString("Behavior:", x + 10, y);
	y += lineHeight;
	
	// Hunting
	ofSetColor(255, 0, 0);
	ofDrawSphere(x + 10, y, shapeSize * 0.7);
	ofSetColor(255);
	ofDrawBitmapString("Hunting", x + textPadding, y + 5);
	y += lineHeight;
	
	// Fleeing
	ofSetColor(255, 255, 0);
	ofDrawSphere(x + 10, y, shapeSize * 0.7);
	ofSetColor(255);
	ofDrawBitmapString("Fleeing", x + textPadding, y + 5);
	
	ofPopStyle();
}
