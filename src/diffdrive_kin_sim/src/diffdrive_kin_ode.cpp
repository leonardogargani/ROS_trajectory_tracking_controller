#include "diffdrive_kin_ode.h"

#include <boost/math/special_functions/sign.hpp>


// constructor
diffdrive_kin_ode::diffdrive_kin_ode(double deltaT) : dt(deltaT), t(0.0), state(3), omega_r(0.0), omega_l(0.0)
{
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}


void diffdrive_kin_ode::setInitialState(double x0, double y0, double theta0)
{
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
}


void diffdrive_kin_ode::setReferenceCommands(double angular_velocity_r, double angular_velocity_l)
{
    omega_r = angular_velocity_r;
    omega_l = angular_velocity_l;
}


// integrate for one step ahead (updating time and steering)
void diffdrive_kin_ode::integrate()
{
    using namespace std::placeholders;
    stepper.do_step(std::bind(&diffdrive_kin_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);
    t += dt;
}


// compute (dx,dy,dtheta) starting from (w_r,w_l)
void diffdrive_kin_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // real state
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];

    // vehicle equations
    dstate[0] = (omega_r + omega_l) / 2 * r * std::cos(theta);      // dx
    dstate[1] = (omega_r + omega_l) / 2 * r * std::sin(theta);      // dy
    dstate[2] = (omega_r - omega_l) / d * r;                        // dtheta
}
