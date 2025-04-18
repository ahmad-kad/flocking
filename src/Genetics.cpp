#include "Genetics.h"

Genetics::Genetics() {
    // Initialize with random small variation (+/- 10%)
    size = ofRandom(0.9f, 1.1f);
    speed = ofRandom(0.9f, 1.1f);
    strength = ofRandom(0.9f, 1.1f);
    stamina = ofRandom(0.9f, 1.1f);
    perception = ofRandom(0.9f, 1.1f);
    stealth = ofRandom(0.9f, 1.1f);
    aggressiveness = ofRandom(0.4f, 0.6f);
    intelligence = ofRandom(0.9f, 1.1f);
    
    // Random small color variation
    colorVariation = ofColor(
        ofRandom(-20, 20),
        ofRandom(-20, 20),
        ofRandom(-20, 20)
    );
}

Genetics Genetics::createOffspring(const Genetics& partner) const {
    Genetics offspring;
    
    // Inherit traits from both parents with 50% chance each
    // Plus small randomization for natural variation
    offspring.size = ofRandom(1.0) < 0.5f ? size : partner.size;
    offspring.size *= ofRandom(0.95f, 1.05f);  // Small variation
    
    offspring.speed = ofRandom(1.0) < 0.5f ? speed : partner.speed;
    offspring.speed *= ofRandom(0.95f, 1.05f);
    
    offspring.strength = ofRandom(1.0) < 0.5f ? strength : partner.strength;
    offspring.strength *= ofRandom(0.95f, 1.05f);
    
    offspring.stamina = ofRandom(1.0) < 0.5f ? stamina : partner.stamina;
    offspring.stamina *= ofRandom(0.95f, 1.05f);
    
    offspring.perception = ofRandom(1.0) < 0.5f ? perception : partner.perception;
    offspring.perception *= ofRandom(0.95f, 1.05f);
    
    offspring.stealth = ofRandom(1.0) < 0.5f ? stealth : partner.stealth;
    offspring.stealth *= ofRandom(0.95f, 1.05f);
    
    offspring.aggressiveness = ofRandom(1.0) < 0.5f ? aggressiveness : partner.aggressiveness;
    offspring.aggressiveness *= ofRandom(0.95f, 1.05f);
    
    offspring.intelligence = ofRandom(1.0) < 0.5f ? intelligence : partner.intelligence;
    offspring.intelligence *= ofRandom(0.95f, 1.05f);
    
    // Mix color variations
    offspring.colorVariation = ofColor(
        (colorVariation.r + partner.colorVariation.r) / 2 + ofRandom(-10, 10),
        (colorVariation.g + partner.colorVariation.g) / 2 + ofRandom(-10, 10),
        (colorVariation.b + partner.colorVariation.b) / 2 + ofRandom(-10, 10)
    );
    
    return offspring;
}

void Genetics::mutate(float mutationChance, float bossChance) {
    // Check if mutation occurs
    if (ofRandom(1.0) < mutationChance) {
        isMutant = true;
        
        // Determine mutation strength (higher = more significant changes)
        mutationStrength = ofRandom(0.5f, 2.0f);
        
        // Randomly select 1-3 traits to mutate
        int traitsToMutate = ofRandom(1, 4);
        
        for (int i = 0; i < traitsToMutate; i++) {
            int trait = ofRandom(8); // 8 possible traits to mutate
            float mutationAmount = ofRandom(-0.3f, 0.3f) * mutationStrength;
            
            switch (trait) {
                case 0: size = ofClamp(size + mutationAmount, 0.5f, 3.0f); break;
                case 1: speed = ofClamp(speed + mutationAmount, 0.5f, 3.0f); break;
                case 2: strength = ofClamp(strength + mutationAmount, 0.5f, 5.0f); break;
                case 3: stamina = ofClamp(stamina + mutationAmount, 0.5f, 3.0f); break;
                case 4: perception = ofClamp(perception + mutationAmount, 0.5f, 3.0f); break;
                case 5: stealth = ofClamp(stealth + mutationAmount, 0.2f, 3.0f); break;
                case 6: aggressiveness = ofClamp(aggressiveness + mutationAmount, 0.1f, 1.0f); break;
                case 7: intelligence = ofClamp(intelligence + mutationAmount, 0.5f, 3.0f); break;
            }
        }
        
        // More dramatic color mutation
        colorVariation = ofColor(
            colorVariation.r + ofRandom(-40, 40) * mutationStrength,
            colorVariation.g + ofRandom(-40, 40) * mutationStrength,
            colorVariation.b + ofRandom(-40, 40) * mutationStrength
        );
        
        // Check for boss mutation (rare, powerful mutation)
        if (ofRandom(1.0) < bossChance) {
            makeBossEntity();
        }
    }
}

void Genetics::makeBossEntity() {
    isApex = true;
    mutationStrength = 3.0f;
    isMutant = true;
    
    // Boss entities are significantly enhanced
    size = ofRandom(2.0f, 3.0f);
    strength = ofRandom(3.0f, 5.0f);
    speed = ofRandom(1.5f, 2.5f);
    stamina = ofRandom(2.0f, 3.0f);
    perception = ofRandom(2.0f, 3.0f);
    intelligence = ofRandom(2.0f, 3.0f);
    
    // Very distinctive coloration - bright, saturated colors
    colorVariation = ofColor(
        ofRandom(100, 255),
        ofRandom(100, 255),
        ofRandom(100, 255)
    );
} 