#ifndef DIFFDRIVE_KIN_FBLIN
#define DIFFDRIVE_KIN_FBLIN


class diffdrive_kin_fblin
{
    public:
        diffdrive_kin_fblin(double P_dist);
        ~diffdrive_kin_fblin();

        void set_diffdriveState(double position_x, double position_y, double heading) { x = position_x; y = position_y; theta = heading; };

        void control_transformation(double vPx, double vPy, double& v, double& omega);
        void output_transformation(double& xP, double& yP);
        void reference_transformation(double xref, double yref, double& xPref, double& yPref);
    
    private:
        double P_dist;
        double x, y, theta;
};

#endif /* DIFFDRIVE_KIN_FBLIN */
