# Enhanced Predator-Prey Ecosystem: Complete Design Document

## 1. System Architecture Overview

```
┌─────────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│                     │  │                  │  │                  │
│    FlockSystem      │◄►│   TerrainSystem  │◄►│   FoodSystem     │
│                     │  │                  │  │                  │
└─────────┬───────────┘  └──────────────────┘  └─────────┬────────┘
          │                                              │
          ▼                                              ▼
┌─────────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│                     │  │                  │  │                  │
│  OctreeSpatial      │◄►│   EventSystem    │◄►│   UISystem       │
│                     │  │                  │  │                  │
└─────────────────────┘  └──────────────────┘  └──────────────────┘
```

## 2. Core Classes Definition

### 2.1 Trophic Levels and Movement Types

```cpp
// Strict hierarchy of predator and prey relationships
enum class TrophicLevel {
    APEX_PREDATOR,   // Top predator, hunts mid-predators and prey
    MID_PREDATOR,    // Intermediate predator, hunts prey
    PREY,            // Standard prey species
    APEX_PREY        // Special prey that cannot be killed
};

// Domain-specific movement types
enum class MovementType {
    AERIAL,          // Flying creatures
    TERRESTRIAL      // Ground-based creatures
};
```

### 2.2 Species Definition

```cpp
class Species {
public:
    // Identification
    std::string name;
    TrophicLevel trophicLevel;
    MovementType movementType;
    
    // Combat capabilities
    float attackStrength = 1.0f;    // Base damage (0.0-10.0)
    float defenseStrength = 1.0f;   // Damage reduction (0.0-10.0)
    float detectionRange = 10.0f;   // Vision radius
    float stealthFactor = 0.5f;     // Hide ability (0.0-1.0)
    
    // Movement parameters
    float maxSpeed = 2.0f;          // Maximum units/second
    float acceleration = 0.5f;      // Units/second²
    float maneuverability = 0.7f;   // Turning rate (0.0-1.0)
    
    // Flocking parameters
    float separationWeight = 1.5f;  // Force multiplier
    float alignmentWeight = 1.0f;   // Force multiplier
    float cohesionWeight = 1.0f;    // Force multiplier
    
    // Lifecycle parameters
    float maxEnergy = 100.0f;       // Maximum energy capacity
    float energyConsumptionRate = 0.1f; // Units/second
    float maxAge = 100.0f;          // Simulation seconds
    float reproductionRate = 0.005f; // Chance per second
    
    // Prey and predator relationships
    std::vector<std::string> preySpecies;
    std::vector<std::string> predatorSpecies;
    
    // Visual representation
    ofColor baseColor = ofColor(128, 128, 128);
    
    // Constructor with required parameters
    Species(const std::string& name, TrophicLevel level, MovementType moveType);
    
    // Check if this species can hunt another
    bool canHunt(const std::string& otherSpecies) const;
    
    // Check if this species is hunted by another
    bool isHuntedBy(const std::string& otherSpecies) const;
};
```

### 2.3 Genetics System

```cpp
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
    
    // Create offspring genetics from two parents
    Genetics createOffspring(const Genetics& partner) const;
    
    // Apply random mutation with given probability
    void mutate(float mutationChance, float bossChance);
    
    // Apply specific mutation to create boss entity
    void makeBossEntity();
};
```

### 2.4 Boid Behaviors and States

```cpp
// Explicit behavior states for state machine
enum class BehaviorState {
    NORMAL,         // Standard flocking
    HUNTING,        // Actively pursuing prey
    FLEEING,        // Running from predators
    HIDING,         // Using cover to avoid detection
    FIGHTING,       // Defending against attack
    FEEDING,        // Consuming food
    MATING,         // Reproductive behavior
    TERRITORIAL,    // Defending territory
    RESTING         // Conserving energy
};

// Specific defensive tactics
enum class DefenseTactic {
    FLEE,           // Run away at maximum speed
    HIDE,           // Seek cover and reduce visibility
    GROUP_DEFENSE,  // Stay with flock for safety
    FIGHT_BACK      // Counter-attack predator
};
```

### 2.5 Life Cycle System

```cpp
class LifeCycle {
public:
    // Age tracking
    float age = 0.0f;               // Current age in seconds
    float maxAge = 100.0f;          // Maximum lifespan
    float maturity = 0.0f;          // Development level (0.0-1.0)
    
    // Reproduction
    float reproductiveReadiness = 0.0f; // Readiness to mate (0.0-1.0)
    float gestationProgress = 0.0f;  // Pregnancy progress (0.0-1.0)
    bool isPregnant = false;         // Currently pregnant
    
    // Energy and health
    float energy = 100.0f;          // Current energy
    float maxEnergy = 100.0f;       // Maximum energy capacity
    float hungerLevel = 0.0f;       // Hunger status (0.0-1.0)
    float lastMealTime = 0.0f;      // Simulation time of last feeding
    
    // Vital status
    bool isAlive = true;            // Currently alive
    float health = 1.0f;            // Current health (0.0-1.0)
    float stress = 0.0f;            // Current stress level (0.0-1.0)
    
    // Update lifecycle state
    void update(float deltaTime, float baseConsumption);
    
    // Feed to restore energy
    bool feed(float nutritionValue);
    
    // Take damage from attack
    bool takeDamage(float amount);
    
    // Start pregnancy
    void conceive();
    
    // Check if reproduction is possible
    bool canReproduce() const;
};
```

### 2.6 Enhanced Boid Class

```cpp
class Boid : public Particle {
public:
    // Species and genetic information
    std::string speciesName;
    TrophicLevel trophicLevel;
    MovementType movementType;
    Genetics genetics;
    
    // State tracking
    BehaviorState behaviorState = BehaviorState::NORMAL;
    DefenseTactic defenseTactic = DefenseTactic::FLEE;
    LifeCycle lifecycle;
    
    // Current targets
    Boid* currentTarget = nullptr;     // Current prey being hunted
    Boid* currentThreat = nullptr;     // Current predator threat
    FoodSource* currentFood = nullptr; // Current food source
    
    // Territory
    ofVec3f territoryCenter;
    float territoryRadius = 10.0f;
    
    // Interaction cooldowns
    float attackCooldown = 0.0f;
    float lastAttackTime = 0.0f;
    float lastDefenseTime = 0.0f;
    
    // Defensive capabilities
    float hideEffectiveness = 0.0f;    // Current hiding (0.0-1.0)
    ofVec3f lastKnownSafePosition;     // Retreat position
    
    // Core flocking parameters
    float separationWeight = 1.5f;
    float alignmentWeight = 1.0f;
    float cohesionWeight = 1.0f;
    float maxSpeed = 2.0f;
    float minSpeed = 0.5f;
    float maxForce = 0.5f;
    
    // Predator-prey behaviors
    ofVec3f huntAsPredator(const std::vector<Boid*>& potentialPrey, TerrainSystem* terrain);
    bool attackTarget(Boid* target);
    void respondToAttack(Boid* attacker);
    void chooseBestDefenseTactic(Boid* attacker);
    
    // Defense behaviors
    ofVec3f executeDefenseTactic(TerrainSystem* terrain);
    ofVec3f executeFleeBehavior();
    ofVec3f executeHideBehavior(TerrainSystem* terrain);
    ofVec3f executeGroupDefenseBehavior();
    ofVec3f executeFightBehavior();
    
    // Competitor behaviors
    ofVec3f competitionBehavior(const std::vector<Boid*>& otherPredators);
    
    // Flocking behaviors
    ofVec3f separate(const std::vector<Boid*>& neighbors);
    ofVec3f align(const std::vector<Boid*>& neighbors);
    ofVec3f cohere(const std::vector<Boid*>& neighbors);
    
    // Movement helpers
    ofVec3f seek(const ofVec3f& target);
    ofVec3f flee(const ofVec3f& target);
    void applyForce(const ofVec3f& force);
    
    // Environment interaction helpers
    ofVec3f findNearestCover(TerrainSystem* terrain);
    float findDistanceToNearestCover(TerrainSystem* terrain);
    ofVec3f findFlockCenter(float radius);
    int countNearbyFlockmates(float radius);
    
    // Update method
    void update(float deltaTime, TerrainSystem* terrain, Octree* octree);
    
    // Rendering
    void draw();
    void drawDebug(bool showVelocity, bool showNeighborhood, bool showForces);
};
```

## 3. Food System

### 3.1 Food Source Types

```cpp
enum class FoodSourceType {
    PLANT,              // For herbivores (grass, fruit)
    CARRION,            // For scavengers and omnivores
    INSECT_SWARM,       // Small abundant food source
    WATER_SOURCE        // Hydration (not nutritional)
};
```

### 3.2 Food Source Base Class

```cpp
class FoodSource {
public:
    // Position and value
    ofVec3f position;
    float nutritionalValue = 20.0f;
    bool available = true;
    float respawnTime = 30.0f;
    float lastConsumedTime = 0.0f;
    float size = 1.0f;
    FoodSourceType type;
    
    // Visibility parameters
    float detectionDifficulty = 0.3f;  // How hard to spot (0.0-1.0)
    float seasonalAbundance = 0.8f;    // Seasonal availability (0.0-1.0)
    
    // Domain accessibility flags
    bool accessibleAerial = true;
    bool accessibleTerrestrial = true;
    
    // Constructor
    FoodSource(const ofVec3f& pos, FoodSourceType type, float nutrition, float size);
    
    // Virtual methods
    virtual bool isAccessibleTo(const Boid* boid) const;
    virtual void update(float deltaTime);
    virtual float consume(Boid* consumer);
    virtual void draw() const;
    
    // Destructor
    virtual ~FoodSource() = default;
};
```

### 3.3 Food Manager System

```cpp
class FoodManager {
public:
    // Food source storage
    std::vector<std::unique_ptr<FoodSource>> foodSources;
    TerrainSystem* terrain = nullptr;
    
    // Food generation parameters
    float plantRegenerationRate = 1.0f;
    float insectRegenerationRate = 1.5f;
    float carrionDecayRate = 0.5f;
    
    // Constructor
    FoodManager(TerrainSystem* terrain);
    
    // System methods
    void setup(int plantFoodCount, int insectSwarmCount);
    void update(float deltaTime);
    void draw() const;
    
    // Food management
    void createPlantFood(int count);
    void createInsectSwarms(int count);
    void addCarrion(const ofVec3f& position, float size, MovementType sourceType);
    
    // Food discovery
    std::vector<FoodSource*> findAccessibleFood(Boid* boid, float searchRadius);
    bool consumeFood(Boid* boid, FoodSource* food);
    
    // Statistics
    int getPlantFoodCount() const;
    int getInsectSwarmCount() const;
    int getCarrionCount() const;
    float getTotalAvailableNutrition() const;
};
```

## 4. Octree Spatial Partitioning

```cpp
class OctreeNode {
public:
    // Node structure
    ofVec3f center;
    float halfSize;
    std::vector<Boid*> boids;
    OctreeNode* children[8] = {nullptr};
    bool isLeaf = true;
    
    // Constants
    static const int MAX_BOIDS_PER_NODE = 8;
    static const int MIN_NODE_SIZE = 2;
    
    // Constructor/destructor
    OctreeNode(const ofVec3f& center, float halfSize);
    ~OctreeNode();
    
    // Core operations
    bool contains(const ofVec3f& point) const;
    void insert(Boid* boid);
    void split();
    int getOctant(const ofVec3f& point) const;
    
    // Query operations
    void queryRange(const ofVec3f& point, float radius, std::vector<Boid*>& result) const;
    void queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel, std::vector<Boid*>& result) const;
    void queryPotentialPrey(Boid* predator, float radius, std::vector<Boid*>& result) const;
    void queryPotentialThreats(Boid* prey, float radius, std::vector<Boid*>& result) const;
    void queryCompetitors(Boid* boid, float radius, std::vector<Boid*>& result) const;
    
    // Helper methods
    bool sphereIntersectsNode(const ofVec3f& point, float radius) const;
    float calculateVisibility(Boid* observer, Boid* target) const;
    
    // Debug
    void debugDraw() const;
};

class Octree {
public:
    // Root node and bounds
    OctreeNode* root = nullptr;
    ofVec3f worldMin = ofVec3f(-100, -50, -100);
    ofVec3f worldMax = ofVec3f(100, 50, 100);
    
    // Constructor/destructor
    Octree();
    ~Octree();
    
    // Core operations
    void build(const std::vector<Boid*>& boids);
    bool isInBounds(const ofVec3f& point) const;
    
    // Query operations
    std::vector<Boid*> queryRange(const ofVec3f& point, float radius) const;
    std::vector<Boid*> queryByType(const ofVec3f& point, float radius, TrophicLevel targetLevel) const;
    std::vector<Boid*> queryPotentialPrey(Boid* predator, float radius) const;
    std::vector<Boid*> queryPotentialThreats(Boid* prey, float radius) const;
    std::vector<Boid*> queryCompetitors(Boid* boid, float radius) const;
    
    // Debug
    void debugDraw() const;
};
```

## 5. Event System

```cpp
enum class EventType {
    PREDATION,           // Predator kills prey
    MUTATION,            // Mutation occurs
    BOSS_SPAWNED,        // Boss-level mutation
    BIRTH,               // New offspring
    DEATH_AGE,           // Death from old age
    DEATH_HUNGER,        // Death from starvation
    DEATH_PREDATION,     // Death from predator
    TERRITORY_CLAIM,     // Territory established
    FOOD_CONSUMED,       // Food source consumed
    ECOSYSTEM_IMBALANCE, // Population imbalance detected
    APEX_PREY_DETECTED   // Special apex prey spawned/detected
};

struct Event {
    // Event data
    EventType type;
    Boid* source = nullptr;
    Boid* target = nullptr;
    ofVec3f location;
    float timestamp;
    std::string description;
    
    // Constructor
    Event(EventType type, const std::string& desc, ofVec3f loc = ofVec3f(0))
        : type(type), description(desc), location(loc), 
          timestamp(ofGetElapsedTimef()) {}
};

// Event callback definition
using EventCallback = std::function<void(const Event&)>;

class EventManager {
public:
    // Subscribe to events
    void subscribe(EventType type, EventCallback callback);
    
    // Dispatch events to subscribers
    void dispatchEvent(const Event& event);
    
    // Event history
    const std::deque<Event>& getRecentEvents(int maxCount = 100) const;
    void clearEvents();
    
    // Filter events
    std::vector<Event> getEventsByType(EventType type, int maxCount = 100) const;
    std::vector<Event> getEventsBySource(Boid* source, int maxCount = 100) const;
    
private:
    // Event subscribers
    std::map<EventType, std::vector<EventCallback>> eventSubscribers;
    
    // Event history
    std::deque<Event> recentEvents;
    static const int MAX_RECENT_EVENTS = 100;
};
```

## 6. Terrain System

```cpp
enum class TerrainType {
    OPEN_FIELD,
    FOREST,
    WATER,
    MOUNTAIN,
    CAVE
};

class TerrainSystem {
public:
    // Terrain cell structure
    struct TerrainCell {
        float height = 0.0f;
        float slope = 0.0f;
        float vegetation = 0.0f;
        TerrainType type = TerrainType::OPEN_FIELD;
        
        // Movement modifiers
        float aerialSpeedModifier = 1.0f;
        float terrestrialSpeedModifier = 1.0f;
        
        // Visibility modifier
        float visibilityFactor = 1.0f;
        
        // Food abundance factor
        float foodFactor = 1.0f;
    };
    
    // Terrain data
    std::vector<std::vector<TerrainCell>> cells;
    int width = 256;
    int height = 256;
    float cellSize = 1.0f;
    
    // Height parameters
    float maxHeight = 20.0f;
    float waterLevel = 2.0f;
    
    // Setup and generation
    void setup(int w, int h, float cellSize);
    void generateTerrain(float heightScale = 10.0f, float roughness = 0.5f);
    
    // Terrain queries
    TerrainCell& getCellAt(float worldX, float worldZ);
    float getHeightAt(float worldX, float worldZ);
    ofVec3f getNormalAt(float worldX, float worldZ);
    float getVegetationDensity(const ofVec3f& position);
    float getSpeedModifier(const ofVec3f& position, MovementType moveType);
    float getVisibilityModifier(const ofVec3f& position);
    
    // Cover and navigation
    ofVec3f findNearestCover(const ofVec3f& position, float maxDistance = 20.0f);
    ofVec3f findNearestWater(const ofVec3f& position, float maxDistance = 20.0f);
    float getCoverDensity(const ofVec3f& position, float radius = 5.0f);
    
    // Rendering
    void draw() const;
    void drawDebug() const;
    
    // Accessors
    int getWidth() const { return width; }
    int getDepth() const { return height; }
    float getHeightScale() const { return maxHeight; }
    float getRoughness() const { return roughnessFactor; }
    
private:
    // Generation parameters
    float roughnessFactor = 0.5f;
    int octaves = 6;
    
    // Helper functions
    ofColor getTerrainColor(const TerrainCell& cell) const;
    void updateDerivedValues();
};
```

## 7. ImGui Visualization System

```cpp
class EcosystemUI {
public:
    // Panel visibility
    bool showOverview = true;
    bool showSpeciesPanel = true;
    bool showPredatorPreyPanel = true;
    bool showFoodWebPanel = true;
    bool showMutationsPanel = false;
    bool showDebugPanel = false;
    bool showTerrainPanel = false;
    bool showPresets = false;
    bool showEvents = false;
    
    // Selection state
    Boid* selectedBoid = nullptr;
    
    // System references
    FlockSystem* flockSystem = nullptr;
    FoodManager* foodManager = nullptr;
    TerrainSystem* terrainSystem = nullptr;
    EventManager* eventManager = nullptr;
    
    // Statistics cache for performance
    struct EcosystemStats {
        // Population counts
        std::map<std::string, int> populationBySpecies;
        std::map<std::string, int> predatorCount;
        std::map<std::string, int> preyCount;
        std::map<std::string, int> apexPreyCount;
        
        // Health metrics
        std::map<std::string, float> avgEnergy;
        std::map<std::string, float> avgHealth;
        
        // Mutation tracking
        std::map<std::string, int> mutantCount;
        std::map<std::string, int> bossCount;
        
        // Predation statistics
        std::map<std::string, int> kills;
        std::map<std::string, int> deaths;
        
        // Totals
        int totalPopulation = 0;
        int totalPredators = 0;
        int totalPrey = 0;
        int totalApexPrey = 0;
        
        // Ecosystem events
        std::map<std::string, std::map<std::string, int>> predationEvents;
        
        // Food statistics
        int availablePlantFood = 0;
        int availableInsectFood = 0;
        int availableCarrion = 0;
        float totalAvailableNutrition = 0.0f;
    };
    
    EcosystemStats stats;
    
    // Constructor
    EcosystemUI(FlockSystem* fs, FoodManager* fm, TerrainSystem* ts);
    
    // Update and draw
    void updateStats();
    void draw();
    
    // Panel drawing methods
    void drawOverviewPanel();
    void drawSpeciesPanel();
    void drawPredatorPreyPanel();
    void drawFoodWebPanel();
    void drawMutationsPanel();
    void drawDebugPanel();
    void drawTerrainPanel();
    void drawPresetsPanel();
    void drawEventsPanel();
    void drawBoidInspector();
    
    // Helper methods
    void drawSpeciesDetails(const std::string& speciesName);
    void drawPredatorPreyHeatmap();
    void drawBoidChart(const std::string& title, 
                      const std::map<std::string, float>& data,
                      float maxValue = 1.0f);
    
    // Preset management
    void loadPreset(int index);
    void saveCurrentAsPreset(const char* name, const char* description);
    void importPresetFromFile(const std::string& path);
    void exportPresetToFile(const std::string& name, 
                          const std::string& description,
                          const std::string& path);
};
```

## 8. Preset System

```cpp
struct EcosystemPreset {
    // Basic info
    std::string name;
    std::string description;
    
    // Terrain settings
    float terrainHeight = 20.0f;
    float terrainRoughness = 0.5f;
    float vegetationDensity = 0.6f;
    
    // Species configuration
    struct SpeciesConfig {
        std::string name;
        int initialCount;
        TrophicLevel trophicLevel;
        MovementType movementType;
        
        // Species parameters
        float maxSpeed;
        float maxEnergy;
        float attackStrength;
        float defenseStrength;
        
        // Appearance
        ofColor baseColor;
        
        // Predator-prey relationships
        std::vector<std::string> preySpecies;
    };
    
    std::vector<SpeciesConfig> speciesConfigs;
    
    // Food system settings
    int plantFoodCount = 60;
    int insectSwarmCount = 20;
    float foodRegenerationRate = 1.0f;
    
    // Mutation settings
    float mutationChance = 0.005f;
    float bossChance = 0.001f;
    
    // File operations
    void saveToFile(const std::string& filename);
    static EcosystemPreset loadFromFile(const std::string& filename);
};

class PresetManager {
public:
    // Preset storage
    std::vector<EcosystemPreset> presets;
    
    // System references
    FlockSystem* flockSystem;
    TerrainSystem* terrainSystem;
    FoodManager* foodManager;
    
    // Constructor
    PresetManager(FlockSystem* fs, TerrainSystem* ts, FoodManager* fm);
    
    // Preset operations
    void loadPreset(int index);
    void loadPresetFromFile(const std::string& filename);
    void saveCurrentAsPreset(const std::string& name, const std::string& description);
    void savePresetToFile(int index, const std::string& filename);
    
    // Create default presets
    void createDefaultPresets();
    
private:
    // Apply specific preset
    void applyPreset(const EcosystemPreset& preset);
    
    // Create preset from current state
    EcosystemPreset createPresetFromCurrentState(const std::string& name, 
                                              const std::string& description);
};
```

## 9. Integration Framework

```cpp
class FlockSystem {
public:
    // Core components
    std::vector<Boid*> boids;
    std::vector<Species> speciesRegistry;
    TerrainSystem* terrain = nullptr;
    Octree* octree = nullptr;
    FoodManager* foodManager = nullptr;
    EventManager* eventManager = nullptr;
    
    // Global simulation parameters
    float individualismFactor = 0.5f;
    float systemChaos = 0.0f;
    float mutationChance = 0.005f;
    float bossMutationChance = 0.001f;
    float predatorPreyBalanceFactor = 1.0f;
    
    // System status
    int totalPredationEvents = 0;
    int totalBirths = 0;
    int totalDeaths = 0;
    int totalMutations = 0;
    int totalBossMutations = 0;
    
    // Statistics tracking
    std::map<std::string, int> killCountsBySpecies;
    std::map<std::string, int> deathCountsBySpecies;
    std::map<std::string, std::map<std::string, int>> predationEvents;
    
    // Constructor/destructor
    FlockSystem();
    ~FlockSystem();
    
    // System setup
    void setTerrainSystem(TerrainSystem* ts) { terrain = ts; }
    void setOctree(Octree* oct) { octree = oct; }
    void setFoodManager(FoodManager* fm) { foodManager = fm; }
    void setEventManager(EventManager* em) { eventManager = em; }
    
    // Species management
    void registerSpecies(const Species& species);
    Species* getSpeciesByName(const std::string& name);
    std::vector<std::string> getAllSpeciesNames() const;
    ofColor getSpeciesColor(const std::string& speciesName) const;
    
    // Update and draw
    void update(float deltaTime);
    void draw(bool debug = false);
    
    // Boid management
    void addBoid(Boid* boid);
    void addBoidsBySpecies(const std::string& speciesName, int count);
    void removeBoid(Boid* boid);
    void clear();
    
    // Flock queries
    std::vector<Boid*> getAllBoids() { return boids; }
    int getCount() const { return boids.size(); }
    Boid* getRandomBoidOfSpecies(const std::string& speciesName);
    
    // Statistics access
    std::map<std::string, int> getKillCountsBySpecies() const { return killCountsBySpecies; }
    std::map<std::string, int> getDeathCountsBySpecies() const { return deathCountsBySpecies; }
    std::map<std::string, std::map<std::string, int>> getPredationEvents() const { return predationEvents; }
    
private:
    // Internal methods
    void updateBoidBehaviors(Boid* boid, float deltaTime);
    void handlePredatorPreyInteractions();
    void handleCompetitorInteractions();
    void handleReproduction(float deltaTime);
    void removeDeadBoids();
    void spawnNewOffspring();
    
    // Safety check for ecosystem balance
    void ensureEcosystemBalance();
    
    // Process specific events
    void handlePredationEvent(Boid* predator, Boid* prey);
    void handleBirthEvent(Boid* parent, Boid* offspring);
    void handleDeathEvent(Boid* boid, const std::string& cause);
    void handleMutationEvent(Boid* boid, bool isBoss);
};

class ofApp : public ofBaseApp {
public:
    // Core systems
    FlockSystem flockSystem;
    TerrainSystem terrainSystem;
    FoodManager foodManager;
    Octree spatialPartitioning;
    EventManager eventManager;
    EcosystemUI ui;
    PresetManager presetManager;
    
    // Application state
    bool paused = false;
    bool showDebug = false;
    float timeScale = 1.0f;
    
    // Setup, update, draw
    void setup() override;
    void update() override;
    void draw() override;
    
    // Input handlers
    void keyPressed(int key) override;
    void mousePressed(int x, int y, int button) override;
    
    // Initialize systems with dependencies
    void initSystems();
    
    // Reset simulation
    void resetSimulation();
    void loadDefaultEcosystem();
};
```

## 10. Implementation Plan

### Phase 1: Core Systems (2 weeks)
1. Implement terrain generation and rendering
2. Create octree spatial partitioning
3. Build basic boid movement and flocking
4. Implement species registry
5. Integrate core systems in main app

### Phase 2: Predator-Prey System (2 weeks)
1. Implement trophic levels and species relationships
2. Create hunting and defensive behaviors
3. Add prey evasion tactics
4. Build basic health and damage system
5. Test basic predator-prey interactions

### Phase 3: Life Cycle & Food System (2 weeks)
1. Implement energy and lifecycle management
2. Add food sources and consumption
3. Create reproduction system
4. Implement death from various causes
5. Build ecosystem balance monitoring

### Phase 4: Mutations and Special Behaviors (1 week)
1. Implement genetics system
2. Create mutation generation
3. Add "boss" entities with special properties
4. Implement apex prey species
5. Test special interactions

### Phase 5: Event System and UI (2 weeks)
1. Implement event system
2. Create core ImGui panels
3. Build ecosystem visualization
4. Add preset management
5. Polish visualization and interaction

### Phase 6: Testing and Balancing (1 week)
1. Create balanced ecosystem presets
2. Test long-term ecosystem stability
3. Optimize performance bottlenecks
4. Fix any remaining bugs
5. Document all features and systems

## 11. Performance Optimization Strategies

1. **Octree Spatial Queries**: Use octree for all spatial queries instead of brute force searches
2. **Frustum Culling**: Only draw boids visible to the camera
3. **Multi-threading**: Process boid behaviors in parallel worker threads
4. **Instanced Rendering**: Use instanced mesh rendering for boids of the same species
5. **Selective Updates**: Update distant boids at lower frequency
6. **GPU Acceleration**: Consider moving core flocking calculations to GPU via compute shaders
7. **Custom Memory Pool**: Implement object pooling for frequently created/destroyed entities
8. **Spatial Hashing**: Add spatial hash table for additional fast spatial lookups

## 12. Expected Outcomes

This enhanced ecosystem simulation will demonstrate:

1. **Emergent Behavior**: Complex patterns emerging from simple rules
2. **Evolution**: Genetic traits evolving through selection pressure
3. **Dynamic Balance**: Self-regulating predator-prey populations
4. **Adaptation**: Species adapting to terrain and resources
5. **Mutations**: Rare special entities influencing ecosystem dynamics
6. **Visual Complexity**: Clear visual representation of interaction patterns

The complete, unambiguous design provides a foundation for a sophisticated predator-prey simulation with terrestrial and aerial dynamics, specialized predator-prey behaviors, and an evolutionary system including mutations and "boss" entities.