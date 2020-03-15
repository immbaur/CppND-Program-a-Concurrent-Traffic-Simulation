#include <iostream>
#include <random>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */


template <typename T>
T MessageQueue<T>::receive()
{
    std::unique_lock<std::mutex> lock(_mutex);
    _condition.wait(lock, [this]{ return !_queue.empty(); });
    // get last element from queue
    T message = std::move(_queue.back());
    // remove all elements from queue, only the last phase matters
    _queue.clear();

    return message; // will not be copied due to return value optimization (RVO) in C++
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _queue.push_back(std::move(msg));
    _condition.notify_one();
}



/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto message = _messageQueue.receive();
        if(message == TrafficLightPhase::green)
            return;
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase() const
{
    //std::cout << "getCurrentPhase #" << _currentPhase << std::endl;
    return _currentPhase;
}

void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    std::unique_lock<std::mutex> lck(_mtx);
    std::cout << "TrafficLight #" << _id << "::cycleThroughPhases: thread id = " << std::this_thread::get_id() << std::endl;
    lck.unlock();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(4000, 6000);    

    double cycleDuration = dist(gen); // duration of a single simulation cycle in ms
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    // init stop watch
    lastUpdate = std::chrono::system_clock::now();

    while(true){

        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration){
            
            //lock the phase resource and toggle phase
            std::lock_guard<std::mutex> lock(_mutex);
            _currentPhase = _currentPhase == TrafficLightPhase::green ? TrafficLightPhase::red : TrafficLightPhase::green;

            _messageQueue.send(std::move(_currentPhase));

            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();
            // set new random duration value
            cycleDuration = dist(gen);

            //lck.lock();
            //print cycle duration
            //std::cout << cycleDuration << std::endl;
            //lck.unlock();
        }
    }
}