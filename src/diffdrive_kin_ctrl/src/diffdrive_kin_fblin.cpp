#include "diffdrive_kin_fblin.h"

#include <cmath>
#include <stdexcept>


diffdrive_kin_fblin::diffdrive_kin_fblin(double P_dist)
{
    this->P_dist  = P_dist;

    // initialize robot pose
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

diffdrive_kin_fblin::~diffdrive_kin_fblin()
{
    // destructor: do nothing
}

void diffdrive_kin_fblin::control_transformation(double vPx, double vPy, double& v, double& omega)
{
    v = vPx * std::cos(theta) + vPy * std::sin(theta);
    omega = (vPy * std::cos(theta) - vPx * std::sin(theta)) / P_dist;
}

void diffdrive_kin_fblin::output_transformation(double& xP, double& yP)
{
    xP = x + P_dist * std::cos(theta);
    yP = y + P_dist * std::sin(theta);
}

void diffdrive_kin_fblin::reference_transformation(double xref, double yref, double& xPref, double& yPref)
{
    xPref = xref + P_dist * std::cos(theta);
    yPref = yref + P_dist * std::sin(theta);
}
