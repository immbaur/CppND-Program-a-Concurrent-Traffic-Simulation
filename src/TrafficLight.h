#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Vehicle;


template <class T>
class MessageQueue
{
public:
    void send(T &&msg);
    T receive();

private:
    std::mutex _mutex;
    std::condition_variable _condition;
    std::deque<T> _queue;
};

enum TrafficLightPhase
{
    red,
    green
};


class TrafficLight : public TrafficObject
{
public:
    // constructor / desctructor
    TrafficLight();

    // getters / setters
    TrafficLightPhase getCurrentPhase() const;

    // typical behaviour methods
    void waitForGreen();
    void simulate() override;
    
private:
    // typical behaviour methods
    void cycleThroughPhases();

    // FP.4b : create a private member of type MessageQueue for messages of type TrafficLightPhase 
    // and use it within the infinite loop to push each new TrafficLightPhase into it by calling 
    // send in conjunction with move semantics.
    MessageQueue<TrafficLightPhase> _messageQueue;
    std::condition_variable _condition;
    std::mutex _mutex;
    TrafficLightPhase _currentPhase;
};

#endif