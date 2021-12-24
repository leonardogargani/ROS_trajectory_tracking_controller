#include "unicycle_kin_ode.h"

#include <boost/math/special_functions/sign.hpp>

unicycle_kin_ode::unicycle_kin_ode(double deltaT) : dt(deltaT), t(0.0), state(3), V(0.0), omega(0.0)
{
    // initial state values (state = [x,y,theta])
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void unicycle_kin_ode::setInitialState(double x0, double y0, double theta0)
{
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
}

void unicycle_kin_ode::setReferenceCommands(double velocity, double angular_velocity)
{
    V = velocity;
    omega = angular_velocity;
}

void unicycle_kin_ode::integrate()
{
    // integrate for one step ahead (updatig time and steering)
    using namespace std::placeholders;
    stepper.do_step(std::bind(&unicycle_kin_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    t += dt;
}

void unicycle_kin_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // actual state
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];

    // vehicle equations
    dstate[0] = V * std::cos(theta); // dx
    dstate[1] = V * std::sin(theta); // dy
    dstate[2] = omega;               // dtheta
}
