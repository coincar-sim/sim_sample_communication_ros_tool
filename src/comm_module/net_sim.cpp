#include <comm_module/net_sim.hpp>

/**
 * Constructor which initializes certain class-variables
 */
NetSim::NetSim() : state_(0), dropProbability_(0) {
}

/** Determines the parameters of the gauss distribution according to the state. Default value of the
 * state
 * is 0.
 * \return Standard-deviation and mean-value of the gauss distribution.
 * \todo { implement the function for the mean-value which depends on the vehicle-density in the
 * covered area }
 */
NetSim::Delay NetSim::getDelayParametersGauss() {
    struct Delay retVal;

    // Delay is represented by a gauss-distribution
    // mean: function of the vehicle-density in the covered area
    // deviation: arbitrary
    // current delay-model: see wiki

    switch (state_) {
    case 0:
        retVal.mean = 0;
        retVal.dev = 0;
        break;

    case 1:
        retVal.mean = rand() / static_cast<double>(RAND_MAX) + 0.5; // put your functions there....
        retVal.dev = rand() / static_cast<double>(RAND_MAX) * 0.02; // .. and there
        break;

    case 2:
        retVal.mean =
            rand() / static_cast<double>(RAND_MAX) * 1.5 + 1.5; // put your functions there....
        retVal.dev = rand() / static_cast<double>(RAND_MAX) * 0.18 + 0.02; // .. and there
        break;

    case 3:
        retVal.mean =
            rand() / static_cast<double>(RAND_MAX) * 1.5 + 3.5; // put your functions there....
        retVal.dev = rand() / static_cast<double>(RAND_MAX) * 0.8 + 0.2; // .. and there
        break;

    default:
        retVal.mean = 0;
        retVal.dev = 0;
        break;
    }

    return retVal;
}

/**
 * Retrieves the actual value of the time-delay according to the state.
 * \return Delay in seconds
 */
double NetSim::getDelayTime() {
    Delay del = getDelayParametersGauss();

    std::normal_distribution<double> time_distribution(del.mean, del.dev);
    std::default_random_engine generator;

    return time_distribution(generator);
}


/**
 * Sets the probability that messages are dropped. Needs to be between 0 (0%) and 1 (100%).
 * \param dropProbability Probability that shall be between 0 and 1
 *
 */
void NetSim::setDropProbability(const double dropProbability) {
    if (dropProbability >= 0 && dropProbability <= 1)
        dropProbability_ = dropProbability;
}

/**
 * Determines if message shall be dropped.
 * \return True means that message shall be dropped. False implies to not drop the message.
 */
bool NetSim::dropMessage() {
    double value = rand() / static_cast<double>(RAND_MAX);
    ROS_DEBUG_THROTTLE(1, "Value: %s", std::to_string(value).c_str());
    if (value <= dropProbability_)
        return true;
    else
        return false;
}
