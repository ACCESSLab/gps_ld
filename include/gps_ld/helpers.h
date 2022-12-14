
#ifndef HELPERS_H_
#define HELPERS_H_

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
// #include <opencv2/core.hpp>
#include <math.h>

const double R_EARTH = 6371000;

struct GpsPt{
    double lat;
    double lon;
};


class Utils{
    public:
        double to_rad(const double angle);
        double to_deg(const double angle);
        double haversine_dist(const GpsPt pt1, const GpsPt pt2);
        double bearing_angle(GpsPt pt1, GpsPt pt2);
        double poly_eval(const Eigen::VectorXd &coeffs, double x);
        Eigen::VectorXd poly_fit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

};

#endif  // HELPERS_H_




