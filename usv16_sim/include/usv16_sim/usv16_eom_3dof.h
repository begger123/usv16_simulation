#ifndef USV16_EOM_3DOF_HEADER_INCLUDED
#define USV16_EOM_3DOF_HEADER_INCLUDED

/**
 * @file usv16_eom_3dof.h
 * @author Ivan Bertaska
 * @date 30 Jun 2016
 * @brief 3DOF Simulation of the USV16 with interchangeable dynamics function
 * 
 */

#include <array>


class Usv16Eom3Dof
{
public:

	typedef void (*dynamics_t)(const double *, const double*, double *);
	typedef void (*actuators_t)(const double *, double *);

	// typedef for 3 and 4 element long floating pt vector - different from Eigen
	typedef std::array<double,3> Vec3f64_t;
	typedef std::array<double,4> Vec4f64_t;

	// 3 x 3 long floating point matrix
	typedef std::array<std::array<double,3>,3> Mat3x3f64_t;
	
	// internal struct to handle the state of the sim
	struct state_t {

		/////////////////
		// Constructor //
		/////////////////

		state_t() : 	
			t{0.0}, pose{0.0, 0.0, 0.0}, vel{0.0, 0.0, 0.0},  acc{0.0, 0.0, 0.0} {}
		

		double 		t;	// time (s)
		Vec3f64_t pose;	// [North (m)	East (m)	Yaw (rad)]
		Vec3f64_t vel;	// [u (m/s)		v (m/s)		r (rad/s)]
		Vec3f64_t acc;	// [ud (m/s)	vd (m/s)	rd (rad/s)]

	};

	/////////////////
	// Constructor //
	/////////////////

	Usv16Eom3Dof();
		
	////////////////////
	// Public Methods //
	////////////////////

	/**
	 * @brief 		Simulates the dynamic model of the USV16 for exactly one timestep.
	 * @details 	Simulates the dynamic model of the USV16 for exactly one timestep.
	 * 
	 * @param u 	Actuator input array [Tp (N) ap (rad) Ts (N) as (rad)] 
	 * @return 		Simulation state after one timestep.
	 */
	state_t sim_one_time_step(const Vec4f64_t& u); // Dummy function for now

	void set_ts(const double& ts) {ts_ = ts;};
	void set_ini_state(const state_t &st){ini_st_ = st; prv_st_ = ini_st_;};
	void reset_ini_state(void){prv_st_ = ini_st_;};

	void set_actuators_function(const actuators_t act) { act_fcn_ptr_ = act; };
	void set_dynamics_function(const dynamics_t dyn) { dyn_fcn_ptr_ = dyn; };

	// getters
	state_t get_ini_state(void) const {return ini_st_; };
	state_t get_prv_state(void) const {return prv_st_; };
	double	get_ts(void) const {return ts_; };

	////////////////
	// Destructor //
	////////////////

	~Usv16Eom3Dof(){};

private:

	/////////////////////
	// Private Methods //
	/////////////////////

	Vec3f64_t actuators_(const Vec4f64_t& u) const;
	Vec3f64_t dynamics_(const Vec3f64_t& tau, const Vec3f64_t& vel) const;
	Vec3f64_t kinematics_(const state_t& state) const;
	Vec3f64_t wrap_(const Vec3f64_t& pose) const;

	Vec3f64_t add_(const Vec3f64_t& a, const Vec3f64_t& b) const;
	Vec3f64_t multiply_(const Vec3f64_t& v, const double& lf) const;
	Vec3f64_t multiply_(const Mat3x3f64_t& M, const Vec3f64_t& v) const;

	/////////////////////
	// Private Members //
	/////////////////////

	state_t 	ini_st_; 	// initial state
	state_t 	prv_st_; 	// previous state
	double		ts_;		// simulation timestep

	dynamics_t	dyn_fcn_ptr_;	// dynamics function pointer
	actuators_t	act_fcn_ptr_;	// actuator function pointer
};

#endif