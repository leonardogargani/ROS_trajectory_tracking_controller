#ifndef UNICYCLE_KIN_FBLIN
#define UNICYCLE_KIN_FBLIN


class unicycle_kin_fblin
{
    public:
        unicycle_kin_fblin(double P_dist);
        ~unicycle_kin_fblin();

        void set_unicycleState(double position_x, double position_y, double heading) { x = position_x; y = position_y; theta = heading; };

        void control_transformation(double vPx, double vPy, double& v, double& omega);
        void output_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    
    private:
        double P_dist;
        double x, y, theta;
};

#endif /* UNICYCLE_KIN_FBLIN */
