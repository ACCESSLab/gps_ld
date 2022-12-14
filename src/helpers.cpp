
#include "helpers.h"

double Utils::to_rad(const double angle){
    return angle*M_PI/180.0;
}

double Utils::to_deg(const double angle){
    return angle*180.0/M_PI;
}

double Utils::haversine_dist(const GpsPt pt1, const GpsPt pt2){
    double lat_delta = to_rad(pt2.lat - pt1.lat);
    double lon_delta = to_rad(pt2.lon - pt1.lon);
    double lat1 = to_rad(pt1.lat);
    double lat2 = to_rad(pt2.lat);

    double a = sin(lat_delta/2)*sin(lat_delta/2) + 
               cos(lat1)*cos(lat2)*sin(lon_delta/2)*sin(lon_delta/2);
    double c = 2*atan2(sqrt(a),sqrt(1-a));

    double dist= R_EARTH*c;

    return dist;
}

double Utils::bearing_angle(GpsPt pt1, GpsPt pt2){
    double y = sin(to_rad(pt2.lon-pt1.lon))*cos(to_rad(pt2.lat));
    double x = cos(to_rad(pt1.lat))*sin(to_rad(pt2.lat)) - 
               sin(to_rad(pt1.lat))*cos(to_rad(pt2.lat))*cos(to_rad(pt2.lon-pt1.lon));
    double brng = to_deg(atan2(y,x));
    brng = fmod((brng + 360),360);
    return brng;
}


// From:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Utils::poly_fit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order){

    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < order; ++i) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);

    return result;
}
//--------------------------------------------------------------------------
double Utils::poly_eval(const Eigen::VectorXd &coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
