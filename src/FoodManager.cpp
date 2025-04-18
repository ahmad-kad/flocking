#include "FoodManager.h"
#include "Boid.h"

FoodManager::FoodManager(TerrainSystem* terrain) : terrain(terrain) {
}

void FoodManager::setup(int plantFoodCount, int insectSwarmCount) {
    // Clear existing food
    foodSources.clear();
    
    // Create initial food sources
    createPlantFood(plantFoodCount);
    createInsectSwarms(insectSwarmCount);
}

void FoodManager::update(float deltaTime) {
    // Update all food sources
    for (auto& food : foodSources) {
        food->update(deltaTime);
    }
    
    // Potentially add new food based on regeneration rates
    if (ofRandom(1.0) < plantRegenerationRate * deltaTime) {
        createPlantFood(1);
    }
    
    if (ofRandom(1.0) < insectRegenerationRate * deltaTime) {
        createInsectSwarms(1);
    }
    
    // Remove decayed carrion
    for (auto it = foodSources.begin(); it != foodSources.end();) {
        if ((*it)->type == FoodSourceType::CARRION) {
            // Check if carrion should decay
            float timeSinceCreation = ofGetElapsedTimef() - (*it)->lastConsumedTime;
            if (timeSinceCreation > 60.0f) {  // 60 seconds lifetime for carrion
                it = foodSources.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void FoodManager::draw() const {
    for (const auto& food : foodSources) {
        food->draw();
    }
}

void FoodManager::createPlantFood(int count) {
    for (int i = 0; i < count; i++) {
        // Create a random position within bounds
        ofVec3f pos = ofVec3f(
            ofRandom(boundsMin.x, boundsMax.x),
            0,  // Plants are at ground level
            ofRandom(boundsMin.z, boundsMax.z)
        );
        
        // Create plant food
        auto plant = std::make_unique<FoodSource>(
            pos, 
            FoodSourceType::PLANT, 
            ofRandom(15.0f, 25.0f),  // Nutrition value
            ofRandom(1.0f, 2.0f)     // Size
        );
        
        foodSources.push_back(std::move(plant));
    }
}

void FoodManager::createInsectSwarms(int count) {
    for (int i = 0; i < count; i++) {
        // Create a random position within bounds (insects are in the air)
        ofVec3f pos = ofVec3f(
            ofRandom(boundsMin.x, boundsMax.x),
            ofRandom(5.0f, 20.0f),  // Insects fly above ground
            ofRandom(boundsMin.z, boundsMax.z)
        );
        
        // Create insect swarm
        auto insects = std::make_unique<FoodSource>(
            pos, 
            FoodSourceType::INSECT_SWARM, 
            ofRandom(10.0f, 15.0f),  // Less nutrition but easier to catch
            ofRandom(2.0f, 3.0f)     // Size of swarm
        );
        
        foodSources.push_back(std::move(insects));
    }
}

void FoodManager::addCarrion(const ofVec3f& position, float size, MovementType sourceType) {
    // Create carrion at the position of a dead animal
    auto carrion = std::make_unique<FoodSource>(
        position, 
        FoodSourceType::CARRION, 
        size * 20.0f,  // Nutrition based on size of dead animal
        size
    );
    
    // Adjust accessibility based on source
    if (sourceType == MovementType::AERIAL) {
        carrion->accessibleAerial = true;
        carrion->accessibleTerrestrial = false;
    }
    
    foodSources.push_back(std::move(carrion));
}

std::vector<FoodSource*> FoodManager::findAccessibleFood(Boid* boid, float searchRadius) {
    std::vector<FoodSource*> accessibleFood;
    
    for (auto& food : foodSources) {
        // Check if food is within search radius
        float distance = boid->position.distance(food->position);
        if (distance <= searchRadius && food->isAccessibleTo(boid)) {
            accessibleFood.push_back(food.get());
        }
    }
    
    return accessibleFood;
}

bool FoodManager::consumeFood(Boid* boid, FoodSource* food) {
    if (!food || !food->available) return false;
    
    // Check distance to food
    float distance = boid->position.distance(food->position);
    if (distance > boid->radius * 2) return false;
    
    // Consume the food and return the result
    float nutrition = food->consume(boid);
    
    return nutrition > 0;
}

int FoodManager::getPlantFoodCount() const {
    int count = 0;
    for (const auto& food : foodSources) {
        if (food->type == FoodSourceType::PLANT && food->available) {
            count++;
        }
    }
    return count;
}

int FoodManager::getInsectSwarmCount() const {
    int count = 0;
    for (const auto& food : foodSources) {
        if (food->type == FoodSourceType::INSECT_SWARM && food->available) {
            count++;
        }
    }
    return count;
}

int FoodManager::getCarrionCount() const {
    int count = 0;
    for (const auto& food : foodSources) {
        if (food->type == FoodSourceType::CARRION && food->available) {
            count++;
        }
    }
    return count;
}

float FoodManager::getTotalAvailableNutrition() const {
    float total = 0.0f;
    for (const auto& food : foodSources) {
        if (food->available) {
            total += food->nutritionalValue;
        }
    }
    return total;
}

void FoodManager::setBounds(ofVec3f min, ofVec3f max) {
    boundsMin = min;
    boundsMax = max;
} 