#pragma once

#include "ofMain.h"

class Genetics {
public:
    // Physical attributes (all initialized to 1.0 = baseline)
    float size = 1.0f;              // Body size factor (0.5-3.0)
    float speed = 1.0f;             // Speed multiplier (0.5-3.0)
    float strength = 1.0f;          // Attack strength multiplier (0.5-5.0)
    float stamina = 1.0f;           // Energy efficiency (0.5-3.0)
    float perception = 1.0f;        // Detection range multiplier (0.5-3.0)
    float stealth = 1.0f;           // Visibility reduction (0.2-3.0)
    float aggressiveness = 0.5f;    // Attack tendency (0.1-1.0)
    float intelligence = 1.0f;      // Decision quality (0.5-3.0)
    
    // Visual attributes
    ofColor colorVariation = ofColor(0, 0, 0);
    
    // Mutation markers
    bool isMutant = false;          // Special rare mutation
    bool isApex = false;            // "Boss" level individual
    float mutationStrength = 0.0f;  // Mutation magnitude (0.0-3.0)
    
    // Constructor
    Genetics();
    
    // Create offspring genetics from two parents
    Genetics createOffspring(const Genetics& partner) const;
    
    // Apply random mutation with given probability
    void mutate(float mutationChance, float bossChance);
    
    // Apply specific mutation to create boss entity
    void makeBossEntity();
}; 