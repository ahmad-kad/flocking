#include "EventManager.h"
#include "Boid.h"

EventManager::EventManager() {
    // Initialize event system
}

void EventManager::subscribe(EventType type, EventCallback callback) {
    eventSubscribers[type].push_back(callback);
}

void EventManager::dispatchEvent(const Event& event) {
    // Add to recent events
    recentEvents.push_front(event);
    
    // Trim if too many events
    if (recentEvents.size() > MAX_RECENT_EVENTS) {
        recentEvents.pop_back();
    }
    
    // Notify subscribers
    if (eventSubscribers.find(event.type) != eventSubscribers.end()) {
        for (auto& callback : eventSubscribers[event.type]) {
            callback(event);
        }
    }
}

const std::deque<Event>& EventManager::getRecentEvents(int maxCount) const {
    return recentEvents; // Caller will take only what they need
}

void EventManager::clearEvents() {
    recentEvents.clear();
}

std::vector<Event> EventManager::getEventsByType(EventType type, int maxCount) const {
    std::vector<Event> result;
    
    for (const auto& event : recentEvents) {
        if (event.type == type) {
            result.push_back(event);
            if (result.size() >= maxCount) break;
        }
    }
    
    return result;
}

std::vector<Event> EventManager::getEventsBySource(Boid* source, int maxCount) const {
    std::vector<Event> result;
    
    for (const auto& event : recentEvents) {
        if (event.source == source) {
            result.push_back(event);
            if (result.size() >= maxCount) break;
        }
    }
    
    return result;
} 