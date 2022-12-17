#ifndef DIFFDRIVE_ODE_H_
#define DIFFDRIVE_ODE_H_

#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;


class diffdrive_kin_ode
{
public:

    diffdrive_kin_ode(double deltaT);

    float d, r;

    void setInitialState(double x0, double y0, double theta0);
    void setReferenceCommands(double angular_velocity_r, double angular_velocity_l);

    void setRobotDimensions(float wheels_distance, float wheels_radius) { d = wheels_distance; r = wheels_radius; };

    void integrate();    
    void getPose(double &x, double &y, double &theta) { x = state[0]; y = state[1]; theta = state[2]; };
    void getCommands(double &angular_velocity_r, double &angular_velocity_l) { angular_velocity_r = omega_r; angular_velocity_l = omega_l; };
    void getTime(double &time) { time = t; };

private:
    double t, dt;
    double omega_r, omega_l;

    state_type state;
    runge_kutta_dopri5 <state_type> stepper;

    void vehicle_ode(const state_type &state, state_type &dstate, double t);
};


#endif /* DIFFDRIVE_ODE_H_ */
