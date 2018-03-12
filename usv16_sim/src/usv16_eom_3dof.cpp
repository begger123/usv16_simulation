#include "usv16_sim/usv16_eom_3dof.h"

#include <iostream>
#include <cmath> // <- trigonometric functions

using namespace std;

/////////////////
// Constructor //
/////////////////

Usv16Eom3Dof::Usv16Eom3Dof()
{
	// set to null
	act_fcn_ptr_ = nullptr;
	dyn_fcn_ptr_ = nullptr;
}

////////////////////
// Public Methods //
////////////////////

Usv16Eom3Dof::state_t Usv16Eom3Dof::sim_one_time_step(const Vec4f64_t& u)
{

	// simulate actuators
	const Vec3f64_t tau = actuators_(u);

	// perform dynamic calculations
	prv_st_.acc = dynamics_(tau,prv_st_.vel);
	prv_st_.vel = add_(multiply_(prv_st_.acc,ts_),prv_st_.vel);

	// perform kinematic calculations
	const Vec3f64_t eta_d = kinematics_(prv_st_);
	prv_st_.pose = wrap_(add_(multiply_(eta_d,ts_),prv_st_.pose));

	// update time
	prv_st_.t += ts_;

	return prv_st_;
}

/////////////////////
// Private Methods //
/////////////////////

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::actuators_(const Vec4f64_t& u) const
{
	const double lf_u[4] = {u[0],u[1],u[2],u[3]};
	double lf_tau[3];

	// sanity check
	if(act_fcn_ptr_ == nullptr)
		cerr << "Actuators function not set..." << endl;

	// use stored fucntion pointer
	act_fcn_ptr_(lf_u,lf_tau);

	return Vec3f64_t({lf_tau[0],lf_tau[1],lf_tau[2]});
}

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::dynamics_(const Vec3f64_t& tau, const Vec3f64_t& vel) const
{
	const double lf_tau[3] = {tau[0],tau[1],tau[2]};
	const double lf_vel[3] = {vel[0],vel[1],vel[2]};
	double lf_vel_d[3];

	// sanity check
	if(dyn_fcn_ptr_ == nullptr)
		cerr << "Dynamics function not set..." << endl;

	// use stored function pointer
	dyn_fcn_ptr_(lf_tau,lf_vel,lf_vel_d);

	return Vec3f64_t({lf_vel_d[0],lf_vel_d[1],lf_vel_d[2]});
}

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::kinematics_(const state_t& state) const
{
	const double psi = state.pose[2];
	
	// geo 2 bod rotation matrix
	const Mat3x3f64_t Jbg = {
					cos(psi),  -sin(psi), 	0.0,
					sin(psi), 	cos(psi),	0.0,
					0.0, 		0.0,		1.0
					};


	return multiply_(Jbg,state.vel);
}

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::wrap_(const Vec3f64_t& pose) const
{
	double psi = pose[2];
	psi = fmod(psi+2*M_PI,2*M_PI);

	return Vec3f64_t({pose[0],pose[1],psi});
}

////////////////////
// BLAS Functions //
////////////////////

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::add_(const Vec3f64_t& a, const Vec3f64_t& b) const
{
	Vec3f64_t r;

	for (int i = 0; i<3; ++i)
		r[i] = a[i]+b[i];

	return r;
}

Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::multiply_(const Vec3f64_t& v, const double& lf) const
{
	Vec3f64_t r;
	
	for (int i = 0; i < 3; ++i)
		r[i] = v[i]*lf;

	return r;
}


Usv16Eom3Dof::Vec3f64_t Usv16Eom3Dof::multiply_(const Mat3x3f64_t& M, const Vec3f64_t& v) const
{
	Vec3f64_t r;

	for (int i = 0; i < 3; ++i)
	{
		double lf = 0;
		for (int ii = 0; ii < 3; ++ii)
		{
			lf += M[i][ii]*v[ii];
		}

		r[i] = lf;
	}	

	return r;
}