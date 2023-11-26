#include "mpc_model.h"
using namespace casadi;

MPCSolver::MPCSolver(){
    this->opti = Opti();
    this->all = Slice();
}

MPCSolver::MPCSolver(Config config){
    Slice all;
    this->all = all;
    this->config = config;
}

void MPCSolver::get_waypoints(Waypoints waypoints){
    this->full_ref_states = waypoints;
    return;
}

void MPCSolver::update_car_state(CarState& car_state) {
    this->car_state = car_state;
    return;
}

int MPCSolver::find_nearest_waypoint() {
    float min_dist = 1000000;
    int min_index = 0;
    for(int i=0; i < int(this->full_ref_states.x.size()); i++) {
        float dist = sqrt(pow(this->full_ref_states.x[i]-this->car_state.x, 2) + pow(this->full_ref_states.y[i]-this->car_state.y, 2));
        if(dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    return min_index;
}

void MPCSolver::get_reference_states_within_horizon(int closest_index){
    Waypoints ref_states;
    int i = closest_index;
    // float lookahead_vel = max(this->car_state.v, float(config.v_min));
    while(int(ref_states.x.size()) < this->config.N+1) {
        ref_states.x.push_back(this->full_ref_states.x[i]);
        ref_states.y.push_back(this->full_ref_states.y[i]);
        float c_theta = this->full_ref_states.theta[i];
        if(c_theta - this->car_state.theta > 4.5){
            c_theta -= 2*M_PI;
        }
        else if(this->car_state.theta - c_theta > 4.5){
            c_theta += 2*M_PI;
        }
        ref_states.theta.push_back(c_theta);
        // i = min((i + int(lookahead_vel*this->config.dt/this->config.dtk)), int(this->full_ref_states.x.size()) - 1);
        i = i + int(this->full_ref_states.x.size())/this->config.N;
        i = min(i, int(this->full_ref_states.x.size()) - 1);
    }
    this->Xref = DM::vertcat({DM(ref_states.x).T(), DM(ref_states.y).T(), DM(ref_states.theta).T()});
    return;
    
}

MX MPCSolver::f(MX X, MX U) {
    MX x_dot = MX::cos(X(2))*U(1);
    MX y_dot = MX::sin(X(2))*U(1);
    //0.33 is wheelbase of F1/10
    MX theta_dot = U(1)*MX::tan(U(0))/this->config.wheelbase;
    MX X_dot = MX::vertcat({x_dot, y_dot, theta_dot});
    return X_dot;
}



tuple<double,  double, vector<tuple<double, double>>> MPCSolver::solve(CarState& car_state, float delta0, float v0) {
    // Cost function
    this->update_car_state(car_state);
    DM Q = DM::zeros(3,3);
    Q(0,0) = this->config.Q[0];
    Q(1,1) = this->config.Q[1];
    Q(2,2) = this->config.Q[2];
    DM R = DM::zeros(2,2);
    R(0,0) = this->config.R[0];
    R(1,1) = this->config.R[1];
    DM QN = DM::zeros(3,3);
    QN(0,0) = this->config.QN[0];
    QN(1,1) = this->config.QN[1];
    QN(2,2) = this->config.QN[2];
    this->X0 = DM::vertcat({this->car_state.x, this->car_state.y, this->car_state.theta});

    // Variables
    this->opti = Opti();
    this->X = this->opti.variable(3, this->config.N+1);
    this->U = this->opti.variable(2, this->config.N);
    this->x = this->X(0, this->all);
    this->y = this->X(1, this->all);
    this->theta = this->X(2, this->all);
    this->delta = this->U(0, this->all);
    this->v = this->U(1, this->all);

    // Updates Xref
    get_reference_states_within_horizon(find_nearest_waypoint());


    MX Cost = MX::zeros(1,1);
    // Path cost 
    for(int i=0; i < this->config.N; i++) {
        Cost = Cost + MX::mtimes(MX::mtimes((this->X(all,i)-this->Xref(all,i)).T(), Q), (this->X(all,i)-this->Xref(all,i))) + MX::mtimes(MX::mtimes(this->U(all,i).T(), R), this->U(all,i));
    }
    // Terminal cost
    Cost = Cost + MX::mtimes(MX::mtimes((this->X(all,this->config.N)-this->Xref(all,this->config.N)).T(), QN), (this->X(all,this->config.N)-this->Xref(all,this->config.N)));
    

    // Dynamic Constraints - RK4
    for(int i=0; i < this->config.N; i++) {
        MX k1 = this->f(this->X(all,i), this->U(all,i));
        MX k2 = this->f(this->X(all,i) + k1*this->config.dt/2, this->U(all,i));
        MX k3 = this->f(this->X(all,i) + k2*this->config.dt/2, this->U(all,i));
        MX k4 = this->f(this->X(all,i) + k3*this->config.dt, this->U(all,i));
        this->opti.subject_to(this->X(all,i+1) == this->X(all,i) + (k1 + 2*k2 + 2*k3 + k4)*(this->config.dt/6));
    }

    // Control constraints
    this->opti.subject_to(this->X(all,0) == this->X0);
    this->opti.subject_to(this->v >= this->config.v_min);
    this->opti.subject_to(this->v <= this->config.v_max);
    this->opti.subject_to(this->delta <= this->config.delta_max);
    this->opti.subject_to(this->delta >= this->config.delta_min);
    
    // Setting solver options
    casadi::Dict opts;
    opts["ipopt.print_level"] = 0;
    // opts["print_time"] = true;
    opts["ipopt.mu_strategy"] = "adaptive";
    // opts["ipopt.max_iter"] = 100;
    // opts["ipopt.tol"] = 1e-5;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["expand"] = true;

    
    // Warm solve with initial guess
    DM initial_X = DM::horzcat({this->X0, this->Xref(all, Slice(1, this->config.N + 1))});
    DM initial_U = DM::horzcat({DM::vertcat({delta0, v0}), DM::zeros(2, this->config.N-1)});
    this->opti.set_initial(this->X, initial_X);
    this->opti.set_initial(this->U, initial_U);

    // Solution step
    this->opti.solver("ipopt", opts);
    this->opti.minimize(Cost);
    OptiSol solution = this->opti.solve();
    auto sol = solution.value(this->U);

    // Formatting and sending solution to controller
    double vel = double(sol(1,0));
    double delta = double(sol(0,0));
    vector<tuple<double, double>> path;
    for(int i=0; i < this->config.N+1; i++) {
        path.push_back(make_tuple(double(solution.value(this->X(0,i))), double(solution.value(this->X(1,i)))));
    }
    return make_tuple(delta, vel, path);
}


