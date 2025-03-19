#pragma once

#include "ofMain.h"
#include "ofxGui.h"

// Structure to hold a complete set of simulation parameters
struct FlockingPreset {
    string name;
    
    // Basic flocking parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float maxSpeed;
    float minSpeed;
    float maxForce;
    float neighborhoodRadius;
    float separationRadius;
    float boundaryWeight;
    
    // Global system parameters
    float individualismFactor;
    float systemChaos;
    float boidsVariability;
    int flockMode;
    
    // Environment parameters
    ofColor backgroundColor;
};

class ConfigManager {
public:
    ConfigManager();
    
    // Load and save presets
    void savePreset(const string& name, const FlockingPreset& preset);
    bool loadPreset(const string& name, FlockingPreset& preset);
    bool loadPresetByIndex(int index, FlockingPreset& preset);
    
    // Get preset names for GUI
    vector<string> getPresetNames();
    
    // Save/load all presets to/from disk
    void savePresetsToFile(const string& filename = "presets.json");
    void loadPresetsFromFile(const string& filename = "presets.json");
    
    // Create default presets
    void createDefaultPresets();
    
private:
    // List of saved presets
    vector<FlockingPreset> presets;
}; 