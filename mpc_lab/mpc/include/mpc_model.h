#include <casadi/casadi.hpp>
#include "util.h"

class MPCSolver{
    private:
        // Config class containing all the parameters for the MPC controller
        Config config;
        // State variable - 4 x N matrix of x, y, theta, v
        casadi::MX X;
        // Control variable - 2 x N matrix of steering angle and velocity
        casadi::MX U;
        // Reference state variable - 3 x N matrix of x_ref, y_ref, theta_ref
        casadi::DM Xref;
        // Initial state variable - 3 x 1 matrix of x0, y0, theta0
        casadi::DM X0;
        // Variable vectors for x, y, theta, v, delta
        casadi::MX x, y, theta, v, delta;
        // Optimizer
        casadi::Opti opti;
        // x_dot = f(x, u)
        casadi::MX f(casadi::MX X, casadi::MX U);

        // Cost function matrices
        casadi::MX Q, R, QN, Cost;
        casadi::Slice all;

        // Waypoints
        Waypoints full_ref_states;
        //Car State
        CarState car_state;
        //Float dynamic variables
        float dt;
        // Function to find nearest waypoint
        int find_nearest_waypoint();
        // Function to get reference states within horizon
        void get_reference_states_within_horizon(int closest_index);

    public:
        // Constructor for MPCSolver given waypoints and config
        MPCSolver(Config config);

        // Default Constructor setting all values to a pre-tuned controller
        MPCSolver();

        // Function to update car state
        void update_car_state(CarState& car_state);

        // Function to get reference waypoints
        void get_waypoints(Waypoints waypoints);
        
        // Function to solve MPC
        tuple<double, double, vector<tuple<double, double>>> solve(CarState& car_state, float delta0, float v0);

};