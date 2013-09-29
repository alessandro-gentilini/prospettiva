/*
    Prospettiva

    This code is written by Alessandro Gentilini, September 2013.
*/

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen/Dense"

void save_ply( const char* fname, const std::vector<Eigen::Vector3d> &points )
{
    const size_t n = points.size();

    std::ofstream oss(fname);
    oss << "ply" << "\n";
    oss << "format ascii 1.0" << "\n";
    oss << "element vertex " << n << "\n";
    oss << "property float x" << "\n";
    oss << "property float y" << "\n";
    oss << "property float z" << "\n";
    oss << "end_header" << "\n";

    for ( size_t i = 0; i < n; i++ )
    {
        oss << points[i].transpose() << "\n";
    }
}

Eigen::Vector2d pinhole_perspective( const Eigen::Vector3d &P_W, const Eigen::Matrix3d &R, const Eigen::Vector3d T, double f )
{
    Eigen::Vector3d P_C = R * P_W + T;
    const double a = f / P_C(2);
    return Eigen::Vector2d(a * P_C(0), a * P_C(1));
}

Eigen::Vector2d telecentric_perspective( const Eigen::Vector3d &P_W, const Eigen::Matrix3d &R, const Eigen::Vector3d T, double )
{
    Eigen::Vector3d P_C = R * P_W + T;
    return Eigen::Vector2d(P_C(0), P_C(1));
}

Eigen::Vector2d radial_distorsion( const Eigen::Vector2d &p, double kappa = 0 )
{
    const double &u = p(0);
    const double &v = p(1);
    double a = 2 / (1 + sqrt(1 - 4 * kappa * (u * u + v * v)));
    return Eigen::Vector2d(a * u, a * v);
}

Eigen::Vector2d to_pixel(const Eigen::Vector2d &p, double s_y, double s_x, double c_x, double c_y)
{
    const double &u = p(0);
    const double &v = p(1);
    return Eigen::Vector2d(v / s_y + c_y, u / s_x + c_x);
}

int main()
{
    double rho = 15;
    const size_t n = 100;
    const double angular_step = 2 * M_PI / n;
    double phi = 0;

    std::vector< Eigen::Vector3d > circle;
    for ( size_t i = 0; i < n; i++ )
    {
        circle.push_back(Eigen::Vector3d(rho * cos(phi), rho * sin(phi), 0));
        phi += angular_step;
    }

    double theta = M_PI/2;
    Eigen::Matrix3d rot;
    rot(0, 0) = cos(theta);
    rot(0, 1) = 0;
    rot(0, 2) = sin(theta);

    rot(1, 0) = 0;
    rot(1, 1) = 1;
    rot(1, 2) = 0;

    rot(2, 0) = -sin(theta);
    rot(2, 1) = 0;
    rot(2, 2) = cos(theta);

    std::vector< Eigen::Vector3d > rotated;
    for ( size_t i = 0; i < circle.size(); i++)
    {
        rotated.push_back(rot * circle[i]);
    }

    std::vector< Eigen::Vector3d > all;
    all.reserve(circle.size() + rotated.size());

    all.insert(all.end(), circle.begin(), circle.end() );
    all.insert(all.end(), rotated.begin(), rotated.end() );

    save_ply("circles.ply",all);

    double alpha = 0;
    Eigen::Matrix3d R_alpha;
    R_alpha(0, 0) = 1;
    R_alpha(0, 1) = 0;
    R_alpha(0, 2) = 0;

    R_alpha(1, 0) = 0;
    R_alpha(1, 1) = cos(alpha);
    R_alpha(1, 2) = -sin(alpha);

    R_alpha(2, 0) = 0;
    R_alpha(2, 1) = sin(alpha);
    R_alpha(2, 2) = cos(alpha);

    double beta = 0;
    Eigen::Matrix3d R_beta;
    R_beta(0, 0) = cos(beta);
    R_beta(0, 1) = 0;
    R_beta(0, 2) = sin(beta);

    R_beta(1, 0) = 0;
    R_beta(1, 1) = 1;
    R_beta(1, 2) = 0;

    R_beta(2, 0) = -sin(beta);
    R_beta(2, 1) = 0;
    R_beta(2, 2) = cos(beta);

    double gamma = 0;
    Eigen::Matrix3d R_gamma;
    R_gamma(0, 0) = cos(gamma);
    R_gamma(0, 1) = -sin(gamma);
    R_gamma(0, 2) = 0;

    R_gamma(1, 0) = sin(gamma);
    R_gamma(1, 1) = cos(gamma);
    R_gamma(1, 2) = 0;

    R_gamma(2, 0) = 0;
    R_gamma(2, 1) = 0;
    R_gamma(2, 2) = 1;

    double focal = 50;
    double t_x = 5*focal;
    double t_y = 0;
    double t_z = 0;
    Eigen::Vector3d T(t_x, t_y, t_z);

    Eigen::Matrix3d R = R_alpha * R_beta * R_gamma;

    std::cout << "row col\n";
    for ( size_t i = 0; i < rotated.size(); i++)
    {
        const Eigen::Vector2d& p = to_pixel( radial_distorsion( pinhole_perspective(rotated[i],R,T,focal) ), 1, 1, 0, 0 );
        std::cout << p(0) << " " << p(1) << "\n";
    }

    return 0;
}
