#ifndef LOCALIZATION_MCL_H
#define LOCALIZATION_MCL_H

#include <vector>
#include <cmath>
#include <stdexcept>

#include "Utils.h"
#include "Data.h"

namespace robotics_mcl {

#define ONE_OVER_SQRT_2PI 0.39894228040143267793994605993438

class MCL {
public:
    MCL(Data* data_p, const Coord& initialPose, const std::vector<double>& noise
            , const std::vector<double>& z, double sigma, int sampInterval);

    bool run(std::vector<Coord>& belief);

    /*
     *  Implementation of the odometry motion model. 
     *  -Return the new position of "particle" based on the previous
     *   pose and the new pose of the turtlebot.
    */
    Coord motionModel(const Coord& particle);

    /*
     *  Implementation of the likelihood field sensor model. 
     *  -Return the weight of the new position of a particle based on
     *   scan ranges and the map.
    */
    double sensorModel(const Coord& newEstimatedPose);

    // Return the new belief.
    std::vector<Coord> resample();

    double prob(double a, double b) {
        return (ONE_OVER_SQRT_2PI / b) * exp(-0.5 * pow(a / b, 2));
    }

private:
    Data* data_p;
    Coord prevPose_m;
    std::vector<double> noise_m;
    std::vector<std::pair<Coord, double>> belief_m;
    // zhit, zrand, and zmax;
    std::vector<double> z_m;
    double sigma_m;
    double wmax_m;
    int sampCount_m;
    int sampInterval_m;
};

}

#endif