#include "MCL.h"

namespace robotics_mcl {

MCL::MCL(Data* d_p, const Coord& initPose, const std::vector<double>& noise
        , const std::vector<double>& z, double s, int sampInterval)
        : data_p(d_p), prevPose_m(initPose), noise_m(noise), z_m(z), sigma_m(s)
        , sampCount_m(0), sampInterval_m(sampInterval) { 
    if(noise_m.size() != 4)
        throw std::runtime_error("Error: noise vector must be of size 4 (a1, a2, a3, a4).");
    if(z_m.size() != 3)
        throw std::runtime_error("Error: z vector must be of size 3 (zhit, zrand, zmax).");
}

bool MCL::run(std::vector<Coord>& belief) {
    ROS_INFO("Sampling: %d", sampCount_m);
    if(prevPose_m == data_p->getPose()) {
        return false;
    } else if(sampCount_m < sampInterval_m) {
        ++sampCount_m;
        return false;
    } else {
        sampCount_m = 0;
        auto eta = 0.0;
        wmax_m = 0.0;
        belief_m = {};
        for(auto i = 0; i < belief.size(); ++i) {
            auto xt = motionModel(belief[i]);
            std::cout << "xt-1:" << belief[i] << "xt:" << xt << std::endl;
            belief[i] = xt;
            // std::cout << "xt:" << xt << std::endl;
            auto wt = sensorModel(xt);
            wmax_m = std::max(wmax_m, wt);
            eta += wt;
            belief_m.push_back(std::make_pair(xt, wt));
        }
        wmax_m /= eta;

        // Normalization of the weights.
        for(auto i = 0; i < belief_m.size(); ++i) {
            belief_m[i].second /= eta;
            // std::cout << belief_m[i].second << " ";
        }
        std::cout << std::endl;
        prevPose_m = data_p->getPose();

        return true;
    }
}

Coord MCL::motionModel(const Coord& particle) {
    // Odometry motion information (u_t-1).
    auto xbar = prevPose_m.x;
    auto ybar = prevPose_m.y;
    auto thetabar = prevPose_m.w;
    // Odometry motion information (u_t).
    auto xbarPrime = data_p->getPose().x;
    auto ybarPrime = data_p->getPose().y;
    auto thetabarPrime = data_p->getPose().w;
    // Pose x_t-1.
    auto x = particle.x;
    auto y = particle.y;
    auto theta = particle.w;
    // Noise coefficients.
    auto a1 = noise_m[0];
    auto a2 = noise_m[1];
    auto a3 = noise_m[2];
    auto a4 = noise_m[3];
    // Calculate the relative motion based on what our odometry told us.
    auto deltaRot1 = atan2(ybarPrime - ybar, xbarPrime - xbar) - thetabar;
    auto deltaTrans = sqrt(pow(xbar - xbarPrime, 2) + pow(ybar - ybarPrime, 2));
    auto deltaRot2 = thetabarPrime - thetabar - deltaRot1;
    // Calculate what the relative motion would be if we were to end up at
    // the hypothesized successor pose, xtPrev.
    auto deltahatRot1 = deltaRot1 + prob(deltaRot1, a1*deltaRot1 + a2*deltaTrans);
    auto deltahatTrans = deltaTrans + prob(deltaTrans, a3*deltaTrans + a4*(deltaRot1 + deltaRot2));
    auto deltahatRot2 = deltaRot2 + prob(deltaRot2, a1*deltaRot2 + a2*deltaTrans);

    auto xPrime = x + deltahatTrans*cos(theta + deltahatRot1);
    auto yPrime = y + deltahatTrans*sin(theta + deltahatRot1);
    auto thetaPrime = theta + deltahatRot1 + deltahatRot2;

    return Coord(xPrime, yPrime, thetaPrime);
}

double MCL::sensorModel(const Coord& newEstimatedPose) {
    auto x = newEstimatedPose.x;
    auto y = newEstimatedPose.y;
    auto theta = newEstimatedPose.w;

    auto xSens = data_p->getSensorPose().x;
    auto ySens = data_p->getSensorPose().y;
    auto thetaSens = data_p->getSensorPose().w;

    auto w = 1.0;
    for(const auto ztk : data_p->getScanRanges()) {
        // Essa checagem já é feita no momento de aquisição dos dados do laser.
        // if(ztk > data_p->getScan().minRange 
        //         && ztk < data_p->getScan().maxRange) {
        auto xzt = x + xSens * cos(theta) - ySens * sin(theta) + ztk * cos(theta + thetaSens);
        auto yzt = y + ySens * cos(theta) - xSens * sin(theta) + ztk * sin(theta + thetaSens);
        auto d = 100000000.0;

        for(auto i = 0; i < data_p->getMap().x.size(); ++i) {
            auto x = data_p->getMap().x[i];
            auto y = data_p->getMap().y[i];
            d = std::min(d, pow(xzt - x, 2) + pow(yzt - y, 2));
        }

        w *= (z_m[0] * prob(pow(d, 2), pow(sigma_m, 2)) + z_m[1] / z_m[2]);
        // }
    }

    return w;
}

std::vector<Coord> MCL::resample() {
    std::vector<Coord> newBelief;

    ROS_INFO("resample");
    for(auto& p : belief_m) {
        auto beta = (rand() % 101) * (wmax_m / 100.0);
        if(p.second > beta) {
            newBelief.push_back(p.first);
            std::cout <<  p.first << " " << p.second << std::endl;
        }
    }


    ROS_INFO("new belief size: %d", (int)newBelief.size());
    return newBelief;
}

}