#pragma once

#include "ofMain.h"
#include "EcosystemTypes.h"
#include <deque>
#include <map>
#include <functional>

// Forward declaration
class Boid;

// Event data structure
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
    // Constructor
    EventManager();
    
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