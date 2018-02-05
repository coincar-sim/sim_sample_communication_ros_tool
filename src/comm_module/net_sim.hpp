#pragma once

#include <cstdlib>
#include <random>
#include <string>
#include <ros/ros.h>


class NetSim {

public:
    NetSim();

    struct Delay {
        double mean;
        double dev;
    };

    // setter and getters

    /**
     * Return the state of the time-model of the network.
     * \return State.
     */
    int getStateSender() {
        return state_;
    }

    /**
     * Sets the state of the time-model of the network.
     * \param state State.
     */
    void setStateSender(const int state) {
        state_ = state;
    }


    /**
     * Returns the probability with which messages are dropped.
     *\return Probability between 0 and 1.
     */
    double getDropProbability() {
        return dropProbability_;
    }

    void setDropProbability(const double);

    // other methods
    Delay getDelayParametersGauss();
    double getDelayTime();
    bool dropMessage();

private:
    // class variables
    int state_;
    double dropProbability_;
};
