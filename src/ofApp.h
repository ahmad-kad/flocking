#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Particle.h"
#include "ParticleEmitter.h"
#include "Boid.h"

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

		// some simple sliders to play with parameters
		//
		bool bHide;
		ofxPanel gui;
		
		// Legacy particle system parameters
		ofxFloatSlider gravity;
		ofxFloatSlider damping;
		ofxFloatSlider radius;
		ofxVec3Slider velocity;
		ofxFloatSlider lifespan;
		ofxFloatSlider rate;
		
		// Flocking behavior parameters
		ofxFloatSlider separationWeight;
		ofxFloatSlider alignmentWeight;
		ofxFloatSlider cohesionWeight;
		ofxFloatSlider neighborhoodRadius;
		ofxFloatSlider separationRadius;
		ofxFloatSlider maxSpeed;
		ofxFloatSlider maxForce;
		ofxToggle seekTarget;
		ofxIntSlider numBoids;
		
		// Flocking system
		FlockingSystem flockingSystem;
		
		// Simulation control
		bool simulationRunning;
		
		// Mouse interaction
		ofVec3f getMouseWorldPosition();
		
		// Legacy particle system (can be removed if no longer needed)
		ParticleEmitter emitter;
		ParticleSystem particleSystem;
		GravityForce* gravityForce;
		TurbulenceForce* turbulenceForce;
		
		// Turbulence force parameters
		ofxFloatSlider turbulenceMinX;
		ofxFloatSlider turbulenceMaxX;
		ofxFloatSlider turbulenceMinY;
		ofxFloatSlider turbulenceMaxY;
		ofxFloatSlider turbulenceMinZ;
		ofxFloatSlider turbulenceMaxZ;
		
		// UI states
		bool showParticleSystem;
		bool showFlockingSystem;
};
