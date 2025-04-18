#include "ConfigManager.h"

ConfigManager::ConfigManager() {
    // Load presets or create defaults
    if (!ofFile::doesFileExist(ofToDataPath("presets.json"))) {
        createDefaultPresets();
        savePresetsToFile();
    } else {
        loadPresetsFromFile();
    }
}

void ConfigManager::savePreset(const string& name, const FlockingPreset& preset) {
    // Check for existing preset
    for (int i = 0; i < presets.size(); i++) {
        if (presets[i].name == name) {
            // Update preset
            presets[i] = preset;
            savePresetsToFile();
            return;
        }
    }
    
    // Add new preset
    FlockingPreset newPreset = preset;
    newPreset.name = name;
    presets.push_back(newPreset);
    savePresetsToFile();
}

bool ConfigManager::loadPreset(const string& name, FlockingPreset& preset) {
    for (const auto& p : presets) {
        if (p.name == name) {
            preset = p;
            return true;
        }
    }
    return false;
}

bool ConfigManager::loadPresetByIndex(int index, FlockingPreset& preset) {
    if (index >= 0 && index < presets.size()) {
        preset = presets[index];
        return true;
    }
    return false;
}

vector<string> ConfigManager::getPresetNames() {
    vector<string> names;
    for (const auto& preset : presets) {
        names.push_back(preset.name);
    }
    return names;
}

void ConfigManager::savePresetsToFile(const string& filename) {
    // Create JSON object
    ofJson json;
    
    // Add presets
    for (int i = 0; i < presets.size(); i++) {
        ofJson presetJson;
        presetJson["name"] = presets[i].name;
        
        // Basic parameters
        presetJson["separationWeight"] = presets[i].separationWeight;
        presetJson["alignmentWeight"] = presets[i].alignmentWeight;
        presetJson["cohesionWeight"] = presets[i].cohesionWeight;
        presetJson["maxSpeed"] = presets[i].maxSpeed;
        presetJson["minSpeed"] = presets[i].minSpeed;
        presetJson["maxForce"] = presets[i].maxForce;
        presetJson["neighborhoodRadius"] = presets[i].neighborhoodRadius;
        presetJson["separationRadius"] = presets[i].separationRadius;
        presetJson["boundaryWeight"] = presets[i].boundaryWeight;
        
        // Global parameters
        presetJson["individualismFactor"] = presets[i].individualismFactor;
        presetJson["systemChaos"] = presets[i].systemChaos;
        presetJson["boidsVariability"] = presets[i].boidsVariability;
        presetJson["flockMode"] = presets[i].flockMode;
        
        // Environment parameters
        presetJson["backgroundColor"] = {
            {"r", presets[i].backgroundColor.r},
            {"g", presets[i].backgroundColor.g},
            {"b", presets[i].backgroundColor.b}
        };
        
        json.push_back(presetJson);
    }
    
    // Save to file
    ofSavePrettyJson(ofToDataPath(filename), json);
}

void ConfigManager::loadPresetsFromFile(const string& filename) {
    // Clear presets
    presets.clear();
    
    // Check if file exists
    if (!ofFile::doesFileExist(ofToDataPath(filename))) {
        return;
    }
    
    // Load JSON
    ofJson json = ofLoadJson(ofToDataPath(filename));
    
    // Parse presets
    for (auto& element : json) {
        FlockingPreset preset;
        preset.name = element["name"];
        
        // Basic parameters
        preset.separationWeight = element["separationWeight"];
        preset.alignmentWeight = element["alignmentWeight"];
        preset.cohesionWeight = element["cohesionWeight"];
        preset.maxSpeed = element["maxSpeed"];
        preset.minSpeed = element["minSpeed"];
        preset.maxForce = element["maxForce"];
        preset.neighborhoodRadius = element["neighborhoodRadius"];
        preset.separationRadius = element["separationRadius"];
        preset.boundaryWeight = element["boundaryWeight"];
        
        // Global parameters
        preset.individualismFactor = element["individualismFactor"];
        preset.systemChaos = element["systemChaos"];
        preset.boidsVariability = element["boidsVariability"];
        preset.flockMode = element["flockMode"];
        
        // Environment parameters
        preset.backgroundColor = ofColor(
            element["backgroundColor"]["r"],
            element["backgroundColor"]["g"],
            element["backgroundColor"]["b"]
        );
        
        presets.push_back(preset);
    }
}

void ConfigManager::createDefaultPresets() {
    presets.clear();
    
    // Default preset
    FlockingPreset defaultPreset;
    defaultPreset.name = "Default";
    defaultPreset.separationWeight = 1.5;
    defaultPreset.alignmentWeight = 1.0;
    defaultPreset.cohesionWeight = 1.0;
    defaultPreset.maxSpeed = 0.1;
    defaultPreset.minSpeed = 0.05;
    defaultPreset.maxForce = 0.05;
    defaultPreset.neighborhoodRadius = 3.0;
    defaultPreset.separationRadius = 1.5;
    defaultPreset.boundaryWeight = 1.0;
    defaultPreset.individualismFactor = 0.5;
    defaultPreset.systemChaos = 0.0;
    defaultPreset.boidsVariability = 0.5;
    defaultPreset.flockMode = 0; // Normal
    defaultPreset.backgroundColor = ofColor(0, 0, 0);
    presets.push_back(defaultPreset);
    
    // Scattered preset
    FlockingPreset scatteredPreset = defaultPreset;
    scatteredPreset.name = "Scattered";
    scatteredPreset.separationWeight = 2.5;
    scatteredPreset.cohesionWeight = 0.5;
    scatteredPreset.individualismFactor = 0.8;
    scatteredPreset.flockMode = 1; // Scattered
    presets.push_back(scatteredPreset);
    
    // Tight formation preset
    FlockingPreset tightPreset = defaultPreset;
    tightPreset.name = "Tight Formation";
    tightPreset.separationWeight = 0.7;
    tightPreset.cohesionWeight = 2.0;
    tightPreset.individualismFactor = 0.2;
    tightPreset.flockMode = 2; // Tight
    presets.push_back(tightPreset);
    
    // Predator-Prey preset
    FlockingPreset predatorPreyPreset = defaultPreset;
    predatorPreyPreset.name = "Predator-Prey";
    predatorPreyPreset.maxSpeed = 0.15;
    predatorPreyPreset.individualismFactor = 0.7;
    predatorPreyPreset.boidsVariability = 0.9;
    predatorPreyPreset.flockMode = 3; // Predator-Prey
    presets.push_back(predatorPreyPreset);
    
    // Chaotic preset
    FlockingPreset chaoticPreset = defaultPreset;
    chaoticPreset.name = "Chaotic";
    chaoticPreset.systemChaos = 0.8;
    chaoticPreset.maxForce = 1.0;
    chaoticPreset.individualismFactor = 0.6;
    chaoticPreset.backgroundColor = ofColor(20, 20, 40);
    presets.push_back(chaoticPreset);
} 