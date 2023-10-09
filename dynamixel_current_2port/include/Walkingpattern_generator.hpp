#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
using namespace Eigen;
using namespace std;

class Trajectory {
private:
	//Com member
	double walkfreq;
	double walktime;
	int walktime_n;
	double freq;
	double step;
	int step_n;
	double del_t;
	double z_c;
	double g;
	double T_prev;
	int NL;
	int sim_n;
	Matrix3d A;
	Vector3d B;
	RowVector3d C;
	double Qe;
	Matrix3d Qx;
	Matrix4d Q_p;
	Matrix<double, 1, 1> R;
	Vector4d I_p;
	Vector4d B_p;
	Matrix<double, 4, 3> F_p;
	Matrix4d A_p;
	Matrix4d K_p;
	double Gi;
	Matrix<double, 1, 3> Gx;
	Matrix4d Ac_p;
	MatrixXd Gd;
	double sim_time;
	RowVectorXd xzmp_ref;
	RowVectorXd yzmp_ref;

	
	VectorXd zmp_ref;
	VectorXd zmp_ref_fifo;
	VectorXd u;
	VectorXd zmp;
	VectorXd zmp_ref_final;
	VectorXd ref_xCP;
	VectorXd ref_yCP;
	double zmp_err_int;
	double u_prev;
	VectorXd Ref_Xpos;
	VectorXd Ref_Ypos;
	bool check_DD = true;
	Matrix<double, 6, 1> Angle_trajectorty_turn;
	Matrix<double, 6, 1> Angle_trajectorty_back;
	//Footmember
	Matrix<double, 6, 1> XStep;
	Matrix<double, 6, 1> XStride;
	double L0 = 0.06;



public:

	Trajectory();
	void Change_Freq(double f);
	// for Com
	void Set_step(double a);
	void Set_distance(double Goal_distance);
	MatrixXd PreviewGd();
	double Return_Step_n();
	VectorXd Get_xCP();
	VectorXd Get_yCP();
	RowVectorXd Get_xZMP();
	RowVectorXd Get_yZMP();
	MatrixXd YComSimulation();
	MatrixXd XComSimulation();
	MatrixXd YComSimulation_Sidewalk(double a, double b, double c, double d, double e, double f);
	MatrixXd YComSimulation_Sidewalk_half(double a, double b, double c, double d);
	MatrixXd YComSimulation_Sidewalk6(double step);
	MatrixXd Huddle_Xcom();
	MatrixXd Huddle_Ycom();
	MatrixXd Huddle_Xcom1();
	MatrixXd Huddle_Ycom1();

	MatrixXd Equation_solver(double t0, double t1, double start, double end);
	double Step(double t);
	double Stride(double t);
	MatrixXd RF_xsimulation_straightwalk();
	MatrixXd LF_xsimulation_straightwalk();
	MatrixXd RF_zsimulation_straightwalk(double h);
	MatrixXd LF_zsimulation_straightwalk(double h);

	MatrixXd RF_ysimulation_leftwalk();
	MatrixXd LF_ysimulation_leftwalk();
	MatrixXd RF_zsimulation_leftwalk();
	MatrixXd LF_zsimulation_leftwalk();
	MatrixXd RF_ysimulation_rightwalk();
	MatrixXd LF_ysimulation_rightwalk();
	MatrixXd RF_zsimulation_rightwalk();
	MatrixXd LF_zsimulation_rightwalk();

	MatrixXd RF_ysimulation_leftwalk_halfstep();
	MatrixXd LF_ysimulation_leftwalk_halfstep();
	MatrixXd RF_zsimulation_leftwalk_halfstep();
	MatrixXd LF_zsimulation_leftwalk_halfstep();
	MatrixXd RF_ysimulation_rightwalk_halfstep();
	MatrixXd LF_ysimulation_rightwalk_halfstep();
	MatrixXd RF_zsimulation_rightwalk_halfstep();
	MatrixXd LF_zsimulation_rightwalk_halfstep();

	MatrixXd RF_ysimulation_leftwalk6();
	MatrixXd LF_ysimulation_leftwalk6();
	MatrixXd RF_zsimulation_leftwalk6();
	MatrixXd LF_zsimulation_leftwalk6();
	MatrixXd RF_ysimulation_rightwalk6();
	MatrixXd LF_ysimulation_rightwalk6();
	MatrixXd RF_zsimulation_rightwalk6();
	MatrixXd LF_zsimulation_rightwalk6();

	MatrixXd RF_xsimulation_huddle();
	MatrixXd LF_xsimulation_huddle();
	MatrixXd RF_zsimulation_huddle(double h, double COM_h);
	MatrixXd LF_zsimulation_huddle(double h, double COM_h);
	MatrixXd RF_xsimulation_huddle1();
	MatrixXd LF_xsimulation_huddle1();
	MatrixXd RF_zsimulation_huddle1(double h);
	MatrixXd LF_zsimulation_huddle1(double h);

	MatrixXd RF_zsimulation_sitdown(double h);
	MatrixXd LF_zsimulation_sitdown(double h);
	MatrixXd RF_zsimulation_standup(double h);
	MatrixXd LF_zsimulation_standup(double h);

	MatrixXd RF_zsimulation_sitdown3(double h);
	MatrixXd LF_zsimulation_sitdown3(double h);
	MatrixXd RF_zsimulation_standup3(double h);
	MatrixXd LF_zsimulation_standup3(double h);

	void Make_turn_trajectory(double angle);
	double Return_turn_trajectory(double t);
	double Return_back_trajectory(double t);
	void Go_Straight(double step, double distance, double height);
	void Freq_Change_Straight(double step, double distance, double height, double freq);
	void Side_Left2();
	void Side_Right2();
	void Side_Left6();
	void Side_Right6();
	void Side_Left1(double step);
	void Side_Right1(double step);
	void Step_in_place(double step, double distance,double height);
	void Stop_Trajectory_straightwalk(double step);
	void Stop_Trajectory_stepinplace(double step);
	void Huddle_Motion(double step,double height,double COM_h);
	void Huddle_Motion1(double step,double height);
	void Stand_up();
	void Sit_down();
	MatrixXd Ref_RL_x;
	MatrixXd Ref_RL_y;
	MatrixXd Ref_RL_z;
	MatrixXd Ref_LL_x;
	MatrixXd Ref_LL_y;
	MatrixXd Ref_LL_z;

	MatrixXd lsRef_RL_x;
	MatrixXd lsRef_RL_y;
	MatrixXd lsRef_RL_z;
	MatrixXd lsRef_LL_x;
	MatrixXd lsRef_LL_y;
	MatrixXd lsRef_LL_z;
	MatrixXd rsRef_RL_x;
	MatrixXd rsRef_RL_y;
	MatrixXd rsRef_RL_z;
	MatrixXd rsRef_LL_x;
	MatrixXd rsRef_LL_y;
	MatrixXd rsRef_LL_z;
/////////////////////////////////////////////////////////////////com and foot position
	MatrixXd Xcom;
	MatrixXd Ycom;
	MatrixXd LF_xFoot;
	MatrixXd RF_xFoot;
	MatrixXd RF_yFoot;
	MatrixXd LF_yFoot;

	VectorXd Turn_Trajectory;
};

class IK_Function
{
private:
	double walkfreq;
	double walktime;
	int walktime_n;
	double freq;
	double step;
	int step_n;
	double sim_time;
	int sim_n;

	//Inverse_kinematics member
	double L0 = 60;
	double L1 = 35.64;
	double L2 = 36.07;
	double L3 = 122.32;
	double L4 = 111.76;
	double L5 = 36.10;
	double L6 = 42.58;

	double FW = 92.8;
	double FL = 137.8;
	double RL_th_IK[6] = { 0.,0.,0.,0.,0.,0. }, LL_th_IK[6] = { 0.,0.,0.,0.,0.,0. };
	double Ref_RL_PR[6] = { 0.,0.,0.,0.,0.,0. }, Ref_LL_PR[6] = { 0.,0.,0.,0.,0.,0. };
	double RL_th_FK[6] = { 0.,0.,0.,0.,0.,0. }, RL_PR_FK[6] = { 0.,0.,0.,0.,0.,0. };
	double LL_th_FK[6] = { 0.,0.,0.,0.,0.,0. }, LL_PR_FK[6] = { 0.,0.,0.,0.,0.,0. };
	double Foot_Height = 0;
	Matrix<double, 6, 1> RL_Compensation_Support_Leg_up;
	Matrix<double, 6, 1> RL_Compensation_Support_Leg_down;
	Matrix<double, 6, 1> RL_Compensation_Swing_Leg_up;
	Matrix<double, 6, 1> RL_Compensation_Swing_Leg_down;
	Matrix<double, 6, 1> RL_Compensation_Support_knee_up;
	Matrix<double, 6, 1> RL_Compensation_Support_knee_down;
	Matrix<double, 6, 1> RL_Compensation_Support_ankle_up;
	Matrix<double, 6, 1> RL_Compensation_Support_ankle_down;
	double RL_Swing_Leg_Compensation_up(double t);
	double RL_Swing_Leg_Compensation_down(double t);
	double RL_Support_Leg_Compensation_up(double t);
	double RL_Support_Leg_Compensation_down(double t);
	double RL_Support_Knee_Compensation_up(double t);
	double RL_Support_Knee_Compensation_down(double t);
	double RL_Support_Ankle_Compensation_up(double t);
	double RL_Support_Ankle_Compensation_down(double t);
	double RL_Support_Leg;
	double RL_Swing_Leg;
	double RL_Support_Knee;
	double RL_Support_Ankle;

	Matrix<double, 6, 1> LL_Compensation_Support_Leg_up;
	Matrix<double, 6, 1> LL_Compensation_Support_Leg_down;
	Matrix<double, 6, 1> LL_Compensation_Swing_Leg_up;
	Matrix<double, 6, 1> LL_Compensation_Swing_Leg_down;
	Matrix<double, 6, 1> LL_Compensation_Support_knee_up;
	Matrix<double, 6, 1> LL_Compensation_Support_knee_down;
	Matrix<double, 6, 1> LL_Compensation_Support_ankle_up;
	Matrix<double, 6, 1> LL_Compensation_Support_ankle_down;
	double LL_Swing_Leg_Compensation_up(double t);
	double LL_Swing_Leg_Compensation_down(double t);
	double LL_Support_Leg_Compensation_up(double t);
	double LL_Support_Leg_Compensation_down(double t);
	double LL_Support_Knee_Compensation_up(double t);
	double LL_Support_Knee_Compensation_down(double t);
	double LL_Support_Ankle_Compensation_up(double t);
	double LL_Support_Ankle_Compensation_down(double t);
	double LL_Support_Leg;
	double LL_Swing_Leg;
	double LL_Support_Knee;
	double LL_Support_Ankle;
	double Com_Height;

public:
	IK_Function();
	void Get_Step_n(double a);
	void BRP_RL_FK(double th[6], double PR[6]);
	void BRP_LL_FK(double th[6], double PR[6]);
	void BRP_RL_IK(double REF_RL_RP[6], double Init_th[6], double IK_th[6]);
	void BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6]);
	void inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6]);
	MatrixXd BRP_RL_Simulation(MatrixXd RFx, MatrixXd RFy, MatrixXd RFz);
	MatrixXd BRP_LL_Simulation(MatrixXd RFx, MatrixXd RFy, MatrixXd RFz);
	void BRP_Simulation(MatrixXd RFx, MatrixXd RFy, MatrixXd RFz, MatrixXd LFx, MatrixXd LFy, MatrixXd LFz, int time);
	void Angle_Compensation(int indext, int size);
	void Fast_Angle_Compensation(int indext);
	void Angle_Compensation_Huddle(int indext);
	void Set_Angle_Compensation(int walktime_n);
	void Change_Angle_Compensation(double RL_Support, double RL_Swing, double RL_Knee , double RL_Ankle, double LL_Support, double LL_Swing, double LL_Knee ,double LL_Ankle);
	void Angle_Compensation_Leftwalk(int indext);
	void Angle_Compensation_Rightwalk(int indext);
	double RL_th[6] = { 0.,0.,-0.610865,1.22173,0.610865,0. }, LL_th[6] = { 0.,0.,-0.610865,1.22173,0.610865, 0.};
	void Change_Com_Height(double h);
    
	double check_index;
};