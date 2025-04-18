#pragma once

#include "ofMain.h"
#include "ofxGui.h"

// Structure for simulation parameters
struct FlockingPreset {
    string name;
    
    // Flocking parameters
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float maxSpeed;
    float minSpeed;
    float maxForce;
    float neighborhoodRadius;
    float separationRadius;
    float boundaryWeight;
    
    // System parameters
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
    
    // Save and load presets
    void savePreset(const string& name, const FlockingPreset& preset);
    bool loadPreset(const string& name, FlockingPreset& preset);
    bool loadPresetByIndex(int index, FlockingPreset& preset);
    
    // Get preset names
    vector<string> getPresetNames();
    
    // Save/load presets to/from disk
    void savePresetsToFile(const string& filename = "presets.json");
    void loadPresetsFromFile(const string& filename = "presets.json");
    
    // Create default presets
    void createDefaultPresets();
    
private:
    // Saved presets
    vector<FlockingPreset> presets;
}; 