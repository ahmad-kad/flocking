#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "FlockSystem.h"
#include "ConfigManager.h"
#include "ofxAssimpModelLoader.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofEasyCam    cam;

		// GUI panels
		bool bHide;
		ofxPanel flockingGui;
		ofxPanel globalGui;
		ofxPanel environmentGui;
		ofxPanel debugGui;
		ofxPanel presetsGui;
		
		// Flocking system
		FlockSystem flockSystem;
		bool simulationPaused;
		
		// Basic flocking parameters
		ofxFloatSlider separationWeight;
		ofxFloatSlider alignmentWeight;
		ofxFloatSlider cohesionWeight;
		ofxFloatSlider maxSpeed;
		ofxFloatSlider minSpeed;
		ofxFloatSlider maxForce;
		ofxFloatSlider neighborhoodRadius;
		ofxFloatSlider separationRadius;
		ofxFloatSlider boundaryWeight;
		ofxIntSlider   boidCount;
		ofxToggle      targetEnabled;
		ofxFloatSlider targetWeight;
		
		// Boid movement parameters
		ofxFloatSlider fieldOfView;
		ofxFloatSlider turnRate;
		
		// Global system parameters
		ofxFloatSlider individualismFactor;
		ofxFloatSlider systemChaos;
		ofxFloatSlider boidsVariability;
		ofxIntSlider flockMode;
		
		// Color-based flocking
		ofxToggle colorBasedFlocking;
		ofxFloatSlider colorInfluence;
		ofxFloatSlider colorSimilarityThreshold;
		
		// Debug visualization
		ofxToggle showDebug;
		ofxToggle showVelocities;
		ofxToggle showNeighborhoods;
		ofxToggle showForces;
		ofxToggle showGrid;
		
		// Environment parameters
		ofParameter<ofColor> backgroundColor;
		ofxButton resetButton;
		ofxButton spawnBoidsButton;
		ofxIntSlider spawnCount;
		ofxButton loadMeshButton;
		string meshPath;
		ofxAssimpModelLoader boidModel;
		ofMesh boidMesh;
		bool customMeshLoaded;
		
		// Target
		ofVec3f targetPosition;
		bool targetMoving;
		float targetPathRadius;
		float targetTime;
        
        // Preset management
        ConfigManager configManager;
        ofParameter<string> presetNameParam;
        ofxLabel presetLabel;
        vector<string> presetNames;
        int selectedPresetIndex;
        ofxButton loadPresetButton;
        ofxButton savePresetButton;

		// Helper methods
		void resetSimulation();
		void spawnBoids();
		void loadBoidMesh();
		void loadMeshFromPath(const string& path);
		void createDefaultMesh();
		void updateFlockMode();
		string getFlockModeDescription(int mode);
		void updateBoidParameters();
        
        // Preset methods
        void loadPreset();
        void savePreset();
        void updateUIFromPreset(const FlockingPreset& preset);
        void updatePresetFromUI(FlockingPreset& preset);
        void positionGUIPanels();
};
