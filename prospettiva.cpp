/*
    Prospettiva

    This code is written by Alessandro Gentilini, September 2013.
*/

#include <iostream>
#include <sstream>
#include <cmath>
#include <vector>
#include "Eigen/Dense"

void save_ply( const std::vector<Eigen::Vector3d>& points )
{
    const size_t n = points.size();

    std::ostringstream oss;
    oss << "ply" << "\n";
    oss << "format ascii 1.0" << "\n";
    oss << "element vertex " << n << "\n";
    oss << "property float x" << "\n";
    oss << "property float y" << "\n";
    oss << "property float z" << "\n";
    oss << "end_header" << "\n";
    std::cout << oss.str() << "\n";

    for ( size_t i = 0; i < n; i++ )
    {
        std::cout << points[i].transpose() << "\n";
    }    
}

int main()
{
    double rho = 1;
    const size_t n = 100;
    const double angular_step = 2 * M_PI / n;
    double phi = 0;

    std::vector< Eigen::Vector3d > circle;

    for ( size_t i = 0; i < n; i++ )
    {
        circle.push_back(Eigen::Vector3d(rho*cos(phi),rho*sin(phi),0));
        phi += angular_step;
    }

    double theta = M_PI/4;
    Eigen::Matrix3d rot;
    rot(0,0) = cos(theta);
    rot(0,1) = 0;
    rot(0,2) = sin(theta);

    rot(1,0) = 0;
    rot(1,1) = 1;
    rot(1,2) = 0;

    rot(2,0) = -sin(theta);
    rot(2,1) = 0;
    rot(2,2) = cos(theta);

    std::vector< Eigen::Vector3d > rotated;
    for ( size_t i = 0; i < circle.size(); i++) {
        rotated.push_back(rot*circle[i]);
    }
    
    std::vector< Eigen::Vector3d > all;
    all.reserve(circle.size()+rotated.size());

    all.insert(all.end(), circle.begin(), circle.end() );
    all.insert(all.end(), rotated.begin(), rotated.end() );

    save_ply(all);

    return 0;
}
