#include "xgc2_world_lidar/observation_contract.h"

#include <cmath>
#include <cstdio>
#include <exception>
#include <iostream>
#include <limits>
using namespace xgc2_world_lidar;
int main() try {
    int n = 0, failed = 0;
    auto check = [&](bool value) {
        ++n;
        if (!value) {
            ++failed;
            std::cerr << "failed " << n << '\n';
        }
    };
    Pose body;
    check(validObservationPose(body, "world", "world"));
    check(!validObservationPose(body, "map", "world"));
    body.orientation.coeffs().setZero();
    check(!validObservationPose(body, "world", "world"));
    body.orientation = Eigen::Quaterniond::Identity();
    body.position.x() = std::numeric_limits<double>::quiet_NaN();
    check(!validObservationPose(body, "world", "world"));
    check(freshObservationTime(0, 0, 0.5));
    check(freshObservationTime(0, 0.5, 0.5));
    check(!freshObservationTime(0, 0.6, 0.5));
    check(!freshObservationTime(10, 1, 0.5));
    check(!freshObservationTime(1, 10, 0.5));
    check(!freshObservationTime(-1, 0, 0.5));
    check(!freshObservationTime(1, 1, 0));
    check(!freshObservationTime(1, 1, std::numeric_limits<double>::infinity()));
    body = mountingPose({10, 20, 30, 0, 0, std::acos(-1.0) / 2});
    Pose sensor = sensorWorldPose(body, mountingPose({1, 0, 2, 0, 0, std::acos(-1.0) / 2}));
    check((sensor.position - Eigen::Vector3d(10, 21, 32)).norm() < 1e-12);
    check((sensor.orientation * Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitX()).norm() <
          1e-12);
    for (const auto& mount :
         {std::vector<double>{0, 0},
          std::vector<double>{0, 0, 0, 0, 0, std::numeric_limits<double>::infinity()}}) {
        bool rejected = false;
        try {
            mountingPose(mount);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        check(rejected);
    }
    std::cout << n << " checks, " << failed << " failures\n";
    return failed ? 1 : 0;
}

catch (const std::exception& error) {
    std::fprintf(stderr, "observation contract exception: %s\n", error.what());
    return 1;
}
