#include "Walkingpattern_generator.hpp"
using namespace Eigen;
using namespace std;
#define deg2rad 0.017453292519943
#define rad2deg 57.295779513082323
#define PI 3.141592653589793

Trajectory::Trajectory()
{
	walkfreq = 1.48114;
	walktime = 2 / walkfreq;
	freq = 100;
	walktime_n = walktime * freq;
	del_t = 1 / freq;
	z_c = 1.2 * 0.28224;
	g = 9.81;
	T_prev = 1.5;
	NL = T_prev * freq;
	A << 1, del_t, del_t * del_t / 2,
		0, 1, del_t,
		0, 0, 1;
	B << del_t * del_t * del_t / 6, del_t * del_t / 2, del_t;
	C << 1, 0, -z_c / g;
	Qe = 1;
	Qx = Matrix3d::Zero();
	Q_p = Matrix4d::Zero();
	Q_p << Qe, 0, 0, 0,
		0, Qx(0, 0), Qx(0, 1), Qx(0, 2),
		0, Qx(1, 0), Qx(1, 1), Qx(1, 2),
		0, Qx(2, 0), Qx(2, 1), Qx(2, 2);
	R << pow(10, -6);
	I_p << 1, 0, 0, 0;

	B_p.row(0) = C * B;
	B_p.row(1) = B.row(0);
	B_p.row(2) = B.row(1);
	B_p.row(3) = B.row(2);

	F_p.row(0) = C * A;
	F_p.row(1) = A.row(0);
	F_p.row(2) = A.row(1);
	F_p.row(3) = A.row(2);

	A_p = Matrix4d::Identity();
	A_p.block<4, 3>(0, 1) = F_p;

	K_p << 41.1035354770549, 824.198546618308, 163.877879931218, 0.866971302114951,
		824.198546618308, 17489.6862079445, 3483.96465227876, 19.6487374770632,
		163.877879931218, 3483.96465227876, 694.154076281009, 3.94329494961909,
		0.866971302114951, 19.6487374770632, 3.94329494961909, 0.0282399782535132;

	Gi = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * I_p;
	Gx = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * F_p;
	Gd = MatrixXd::Zero(NL, 1);
	Ac_p = A_p - B_p * (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * A_p;
	zmp_err_int = 0;
	u_prev = 0;
	step = 0.05;
	step_n = 0;
	sim_n = 0;
	sim_time = 0;
	Angle_trajectorty_turn << 0, 0, 0, 0, 0, 0;
	Angle_trajectorty_back << 0, 0, 0, 0, 0, 0;

	XStep << 0, 0, 0, 0, 0, 0;
	XStride << 0, 0, 0, 0, 0, 0;
};

void Trajectory::Change_Freq(double f)
{
	walktime = f / walkfreq;
	walktime_n = walktime * freq;
}

void Trajectory::Set_step(double a)
{
	step = a;
}

void Trajectory::Set_distance(double Goal_distance)
{
	step_n = (Goal_distance + 2 * step) / (2 * step);
	sim_n = walktime_n * step_n;

	Ref_Xpos = VectorXd::Zero(2 * step_n);
	for (int i = 0; i < (2 * step_n); i++)
	{
		if (i == 0)
			Ref_Xpos(i) = 0;
		else if (i < 2 * step_n - 1)
			Ref_Xpos(i) = step * (i - 1);
		else if (i == 2 * step_n - 1)
			Ref_Xpos(i) = step * (i - 2);
	}

	Ref_Ypos = VectorXd::Zero(2 * step_n);
	for (int i = 0; i < (2 * step_n); i++)
	{
		if (i < 1)
		{
			Ref_Ypos(i) = 0;
		}
		else if (i > 2 * step_n - 2)
		{
			Ref_Ypos(i) = 0;
		}
		else if ((i + 1) % 2 == 0)
		{
			Ref_Ypos(i) = 0.06;
		}
		else if ((i + 1) % 2 == 1)
		{
			Ref_Ypos(i) = -0.06;
		}
	}
}

double Trajectory::Return_Step_n()
{
	return step_n;
}

MatrixXd Trajectory::PreviewGd()
{
	for (int l = 0; l < NL; l++)
	{

		Matrix4d temp = Ac_p.transpose();
		for (int i = 1; i < l; i++)
		{
			temp = temp * Ac_p.transpose();
		}
		Gd(l, 0) = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * temp * K_p * I_p;
		if (l == 0)
			Gd(l, 0) = (R + B_p.transpose() * K_p * B_p).inverse() * B_p.transpose() * K_p * I_p;
	}
	return Gd;
};

MatrixXd Trajectory::XComSimulation()
{
	PreviewGd();
	zmp_err_int = 0;
	u_prev = 0;
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i < float_walktime_n)
		{
			zmp_ref(i) = 0;
		}
		else if (i > sim_n - 1)
		{
			zmp_ref(i) = Ref_Xpos(2 * step_n - 1);
		}
		else if (i < (float_Com_flag * 0.5 + 0.5) * float_walktime_n)
		{
			zmp_ref(i) = Ref_Xpos(Com_flag);
		}
		else if (i < (float_Com_flag * 0.5 + 1) * float_walktime_n)
		{
			zmp_ref(i) = Ref_Xpos(Com_flag + 1);
		}
		if ((i + 1) % walktime_n == 0)
		{
			Com_flag = Com_flag + 2;
			float_Com_flag = float_Com_flag + 2;
		}
	}

	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd XCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	XCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev += Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * XCom.col(i) + u_prev;

		XCom.col(i + 1) = A * XCom.col(i) + B * u[i];

		zmp[i] = C * XCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = XCom(0, i) + 1 / w * XCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	xzmp_ref = zmp_ref;
	ref_xCP = CP.block(18, 0, sim_n, 1);
	MatrixXd XCom_ = XCom.block(0, 18, 1, sim_n);
	return XCom_;
};

MatrixXd Trajectory::YComSimulation()
{
	PreviewGd();
	zmp_err_int = 0;
	u_prev = 0;
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i > sim_n - 1)
		{
			zmp_ref(i) = 0;
		}
		else if (i < (float_Com_flag * 0.5 + 0.5) * float_walktime_n)
		{
			zmp_ref(i) = Ref_Ypos(Com_flag);
		}
		else if (i < (float_Com_flag * 0.5 + 1) * float_walktime_n)
		{
			zmp_ref(i) = Ref_Ypos(Com_flag + 1);
		}

		if ((i + 1) % walktime_n == 0)
		{
			Com_flag = Com_flag + 2;
			float_Com_flag = float_Com_flag + 2;
		}
	}

	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	yzmp_ref = zmp_ref;
	ref_yCP = CP.block(18, 0, sim_n, 1);
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

VectorXd Trajectory::Get_xCP()
{
	return ref_xCP;
}

VectorXd Trajectory::Get_yCP()
{
	return ref_yCP;
}

RowVectorXd Trajectory::Get_xZMP()
{
	return xzmp_ref;
}

RowVectorXd Trajectory::Get_yZMP()
{
	return yzmp_ref;
}

MatrixXd Trajectory::YComSimulation_Sidewalk(double a, double b, double c, double d, double e, double f)
{
	step_n = 5;
	sim_n = walktime_n * step_n;
	PreviewGd();
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		double time = i * del_t;
		if (time < 1 * walktime)
		{
			zmp_ref[i] = 0;
		}
		else if (time < 1.5 * walktime)
		{
			zmp_ref[i] = a;
		}
		else if (time < 2 * walktime)
		{
			zmp_ref[i] = b;
		}
		else if (time < 2.5 * walktime)
		{
			zmp_ref[i] = c;
		}
		else if (time < 3 * walktime)
		{
			zmp_ref[i] = d;
		}
		else if (time < 3.5 * walktime)
		{
			zmp_ref[i] = e;
		}
		else if (time < 4 * walktime)
		{
			zmp_ref[i] = f;
		}
		else
			zmp_ref[i] = f;
	}
	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

MatrixXd Trajectory::YComSimulation_Sidewalk_half(double a, double b, double c, double d)
{
	step_n = 3;
	sim_n = walktime_n * step_n;
	PreviewGd();
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		double time = i * del_t;
		if (time < 1 * walktime)
		{
			zmp_ref[i] = 0;
		}
		else if (time < 1.5 * walktime)
		{
			zmp_ref[i] = a;
		}
		else if (time < 2 * walktime)
		{
			zmp_ref[i] = b;
		}
		else if (time < 2.5 * walktime)
		{
			zmp_ref[i] = c;
		}
		else if (time < 3 * walktime)
		{
			zmp_ref[i] = d;
		}
		else
		{
			zmp_ref[i] = d;
		}
	}
	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

MatrixXd Trajectory::YComSimulation_Sidewalk6(double step)
{
	step_n = 9;
	sim_n = walktime_n * step_n;
	PreviewGd();
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		double time = i * del_t;
		if (time < 1 * walktime)
		{
			zmp_ref[i] = 0;
		}
		else if (time < 1.5 * walktime)
		{
			zmp_ref[i] = -step;
		}
		else if (time < 2 * walktime)
		{
			zmp_ref[i] = -step;
		}
		else if (time < 2.5 * walktime)
		{
			zmp_ref[i] = 2 * step;
		}
		else if (time < 3 * walktime)
		{
			zmp_ref[i] = 0;
		}
		else if (time < 3.5 * walktime)
		{
			zmp_ref[i] = 3 * step;
		}
		else if (time < 4 * walktime)
		{
			zmp_ref[i] = 1 * step;
		}
		else if (time < 4.5 * walktime)
		{
			zmp_ref[i] = 4 * step;
		}
		else if (time < 5 * walktime)
		{
			zmp_ref[i] = 2 * step;
		}
		else if (time < 5.5 * walktime)
		{
			zmp_ref[i] = 5 * step;
		}
		else if (time < 6 * walktime)
		{
			zmp_ref[i] = 3 * step;
		}
		else if (time < 6.5 * walktime)
		{
			zmp_ref[i] = 6 * step;
		}
		else if (time < 7 * walktime)
		{
			zmp_ref[i] = 4 * step;
		}
		else if (time < 7.5 * walktime)
		{
			zmp_ref[i] = 7 * step;
		}
		else if (time < 8 * walktime)
		{
			zmp_ref[i] = 6 * step;
		}
		else
			zmp_ref[i] = 6 * step;
	}
	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

MatrixXd Trajectory::Huddle_Xcom()
{
	sim_n = walktime_n * 6;
	zmp_err_int = 0;
	u_prev = 0;
	PreviewGd();
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i < 1.75 * float_walktime_n)
		{
			zmp_ref[i] = 0;
		}
		else if (i < 4.5 * float_walktime_n)
		{
			zmp_ref[i] = step;
		}
		else
		{
			zmp_ref[i] = 1.4 * step;
		}
	}

	RowVectorXd zmp_ref_fifo(NL);
	RowVectorXd u(sim_n + 19);
	RowVectorXd zmp(sim_n + 19);
	RowVectorXd zmp_ref_final(sim_n + 19);
	RowVectorXd CP(sim_n + 19);
	MatrixXd XCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	XCom.setZero();
	double w = sqrt(g / z_c - 1.2 * 0.05);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev += Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * XCom.col(i) + u_prev;

		XCom.col(i + 1) = A * XCom.col(i) + B * u[i];

		zmp[i] = C * XCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = XCom(0, i) + 1 / w * XCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd XCom_ = XCom.block(0, 18, 1, sim_n);
	return XCom_;
}

MatrixXd Trajectory::Huddle_Ycom()
{
	sim_n = walktime_n * 6;
	PreviewGd();
	zmp_err_int = 0;
	u_prev = 0;
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i < 0.75 * float_walktime_n)
		{
			zmp_ref[i] = 0;
		}
		else if (i < 1.75 * float_walktime_n)
		{
			zmp_ref[i] = -0.06;
		}
		else if (i < 4.5 * float_walktime_n)
		{
			zmp_ref[i] = 0.06;
		}
		else if (i < 5.7 * float_walktime_n)
		{
			zmp_ref[i] = -0.06;
		}
		else
		{
			zmp_ref[i] = 0;
		}
	}

	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c - 1.2 * 0.05);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

MatrixXd Trajectory::Huddle_Xcom1()
{
	sim_n = walktime_n * 3;
	zmp_err_int = 0;
	u_prev = 0;
	PreviewGd();
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i < 1.75 * float_walktime_n)
		{
			zmp_ref[i] = 0;
		}
		else
		{
			zmp_ref[i] = step;
		}
	}

	RowVectorXd zmp_ref_fifo(NL);
	RowVectorXd u(sim_n + 19);
	RowVectorXd zmp(sim_n + 19);
	RowVectorXd zmp_ref_final(sim_n + 19);
	RowVectorXd CP(sim_n + 19);
	MatrixXd XCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	XCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev += Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * XCom.col(i) + u_prev;

		XCom.col(i + 1) = A * XCom.col(i) + B * u[i];

		zmp[i] = C * XCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = XCom(0, i) + 1 / w * XCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd XCom_ = XCom.block(0, 18, 1, sim_n);
	return XCom_;
}

MatrixXd Trajectory::Huddle_Ycom1()
{
	sim_n = walktime_n * 3;
	PreviewGd();
	zmp_err_int = 0;
	u_prev = 0;
	int Com_flag = 0;
	float float_Com_flag = 0;
	float float_walktime_n = walktime_n;
	RowVectorXd zmp_ref(sim_n + 19);
	for (int i = 0; i < sim_n + 19; i++)
	{
		if (i < 0.75 * float_walktime_n)
		{
			zmp_ref[i] = 0;
		}
		else if (i < 1.75 * float_walktime_n)
		{
			zmp_ref[i] = -0.06;
		}
		else if (i < 2.75 * float_walktime_n)
		{
			zmp_ref[i] = 0.06;
		}
		else
		{
			zmp_ref[i] = 0;
		}
	}

	VectorXd zmp_ref_fifo(NL);
	VectorXd u(sim_n + 19);
	VectorXd zmp(sim_n + 19);
	VectorXd zmp_ref_final(sim_n + 19);
	VectorXd CP(sim_n + 19);
	MatrixXd YCom(3, sim_n + 20);
	zmp_ref_fifo.setZero();
	u.setZero();
	zmp.setZero();
	zmp_ref_final.setZero();
	CP.setZero();
	YCom.setZero();
	double w = sqrt(g / z_c);
	for (int i = 0; i < sim_n + 19; i++)
	{
		for (int j = 0; j < NL; j++)
		{
			if (i + j < sim_n + 19)
			{
				zmp_ref_fifo[j] = zmp_ref[i + j];
			}
			else
			{
				zmp_ref_fifo[j] = zmp_ref[sim_n + 18];
			}
		}
		u_prev = 0;
		for (int j = 0; j < NL; j++)
		{
			u_prev = u_prev + Gd(j, 0) * zmp_ref_fifo[j];
		}
		u[i] = Gi * zmp_err_int - Gx * YCom.col(i) + u_prev;

		YCom.col(i + 1) = A * YCom.col(i) + B * u[i];

		zmp[i] = C * YCom.col(i);

		zmp_err_int += (zmp_ref[i] - zmp[i]);

		CP[i] = YCom(0, i) + 1.0 / w * YCom(1, i);

		zmp_ref_final[i] = zmp_ref[i];
	}
	MatrixXd YCom_ = YCom.block(0, 18, 1, sim_n);
	return YCom_;
}

MatrixXd Trajectory::Equation_solver(double t0, double t1, double start, double end)
{
	Matrix<double, 6, 6> A;
	Matrix<double, 6, 1> B;
	Matrix<double, 6, 1> X;
	A << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
		0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4),
		0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3),
		1, t1, pow(t1, 2), pow(t1, 3), pow(t1, 4), pow(t1, 5),
		0, 1, 2 * t1, 3 * pow(t1, 2), 4 * pow(t1, 3), 5 * pow(t1, 4),
		0, 0, 2, 6 * t1, 12 * pow(t1, 2), 20 * pow(t1, 3);
	B << start, 0, 0, end, 0, 0;
	X = A.colPivHouseholderQr().solve(B);
	return X;
};

double Trajectory::Step(double t)
{
	double X = XStep(0) + XStep(1) * t + XStep(2) * pow(t, 2) + XStep(3) * pow(t, 3) + XStep(4) * pow(t, 4) + XStep(5) * pow(t, 5);
	return X;
};

double Trajectory::Stride(double t)
{
	double X = XStride(0) + XStride(1) * t + XStride(2) * pow(t, 2) + XStride(3) * pow(t, 3) + XStride(4) * pow(t, 4) + XStride(5) * pow(t, 5);
	return X;
};

MatrixXd Trajectory::RF_xsimulation_straightwalk()
{
	int sim_n = step_n * walktime_n;
	double dwalktime_n = walktime_n;
	double Rfootflag = 0;

	XStep = Equation_solver(0, dwalktime_n * 0.3, 0, step);
	XStride = Equation_solver(0, dwalktime_n * 0.3, 0, 2 * step);

	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		if (i < 0.6 * dwalktime_n)
			Footpos[i] = 0;
		else if (i < 0.9 * dwalktime_n)
			Footpos[i] = Step(i - 0.6 * dwalktime_n);
		else if (i < 1.1 * dwalktime_n)
			Footpos[i] = step;
		else if (i < (Rfootflag + 0.6) * dwalktime_n)
		{
			Footpos[i] = (2 * Rfootflag - 1) * step;
		}
		else if (i < (Rfootflag + 0.9) * dwalktime_n)
		{
			Footpos[i] = Stride(i - (Rfootflag + 0.6) * dwalktime_n) + (2 * Rfootflag - 1) * step;
		}
		else if (i < (Rfootflag + 1.1) * dwalktime_n)
			Footpos[i] = (2 * Rfootflag + 1) * step;
		else
			Footpos[i] = (2 * step_n - 3) * step;

		if ((i + 1) % walktime_n == 0)
		{
			if (Rfootflag < step_n - 2)
			{
				Rfootflag = Rfootflag + 1;
			}
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_xsimulation_straightwalk()
{
	double dwalktime_n = walktime_n;
	int sim_n = step_n * walktime_n;
	double Lfootflag = 0;

	XStep = Equation_solver(0, dwalktime_n * 0.3, 0, step);
	XStride = Equation_solver(0, dwalktime_n * 0.3, 0, 2 * step);

	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		if (i < 1.1 * walktime_n)
			Footpos[i] = 0;
		else if (i < (Lfootflag + 0.1) * walktime_n)
			Footpos[i] = (2 * Lfootflag - 2) * step;
		else if (i < (Lfootflag + 0.4) * walktime_n)
			Footpos[i] = Stride(i - (Lfootflag + 0.1) * dwalktime_n) + (2 * Lfootflag - 2) * step;
		else if (i < (Lfootflag + 1.1) * walktime_n)
			Footpos[i] = (2 * Lfootflag) * step;
		else if (i < (step_n - 0.6) * walktime_n)
			Footpos[i] = Step(i - (Lfootflag + 1.1) * dwalktime_n) + (2 * step_n - 4) * step;
		else
			Footpos[i] = (2 * step_n - 3) * step;
		if ((i + 1) % walktime_n == 0)
		{
			if (Lfootflag < step_n - 2)
				Lfootflag = Lfootflag + 1;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_straightwalk(double h)
{
	int sim_n = step_n * walktime_n;
	double dwalktime_n = walktime_n;
	double Rfootflag = 0;

	this->XStep = Equation_solver(0, 0.2 * dwalktime_n, 0, h);
	this->XStride = Equation_solver(0.2 * dwalktime_n, 0.4 * dwalktime_n, h, 0);

	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		if (i < (Rfootflag + 0.55) * walktime_n)
			Footpos[i] = 0;
		else if (i < (Rfootflag + 0.75) * walktime_n)
			Footpos[i] = Step(i - (Rfootflag + 0.55) * walktime_n);
		else if (i < (Rfootflag + 0.95) * walktime_n)
			Footpos[i] = Stride(i - (Rfootflag + 0.55) * walktime_n);
		else if (i < (Rfootflag + 1) * walktime_n)
			Footpos[i] = 0;
		else
			Footpos[i] = 0;
		if ((i + 1) % walktime_n == 0)
		{
			if (Rfootflag < step_n - 2)
				Rfootflag = Rfootflag + 1;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_straightwalk(double h)
{
	int sim_n = step_n * walktime_n;
	double dwalktime_n = walktime_n;
	double Lfootflag = 0;

	this->XStep = Equation_solver(0, 0.2 * dwalktime_n, 0, h);
	this->XStride = Equation_solver(0.2 * dwalktime_n, 0.4 * dwalktime_n, h, 0);

	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		if (i < 1.05 * walktime_n)
			Footpos[i] = 0;
		else if (i < (Lfootflag + 0.05) * walktime_n)
			Footpos[i] = 0;
		else if (i < (Lfootflag + 0.25) * walktime_n)
			Footpos[i] = Step(i - (Lfootflag + 0.05) * walktime_n);
		else if (i < (Lfootflag + 0.45) * walktime_n)
			Footpos[i] = Stride(i - (Lfootflag + 0.05) * walktime_n);
		else if (i < (Lfootflag + 0.5) * walktime_n)
			Footpos[i] = 0;
		else
			Footpos[i] = 0;
		if ((i + 1) % walktime_n == 0)
		{
			if (Lfootflag < step_n - 1)
				Lfootflag = Lfootflag + 1;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_ysimulation_leftwalk()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) - 0.06;
		}
		else if (time < 3.1 * walktime)
		{
			Footpos[i] = -0.06 + step;
		}
		else if (time < 3.4 * walktime)
		{
			Footpos[i] = Step(time - 3.1 * walktime) - 0.06 + step;
		}
		else
		{
			Footpos[i] = -0.06 + 2 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_leftwalk()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) + 0.06;
		}
		else if (time < 2.6 * walktime)
		{
			Footpos[i] = 0.06 + step;
		}
		else if (time < 2.9 * walktime)
		{
			Footpos[i] = Step(time - 2.6 * walktime) + 0.06 + step;
		}
		else if (time < 3.6 * walktime)
		{
			Footpos[i] = 0.06 + 2 * step;
		}
		else if (time < 3.9 * walktime)
		{
			Footpos[i] = 0.06 + 2 * step;
		}
		else
		{
			Footpos[i] = 0.06 + 2 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_leftwalk()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime)
		{
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime)
		{
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_leftwalk()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime)
		{
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime)
		{
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.95 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_ysimulation_rightwalk()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) - 0.06;
		}
		else if (time < 2.6 * walktime)
		{
			Footpos[i] = -0.06 + step;
		}
		else if (time < 2.9 * walktime)
		{
			Footpos[i] = Step(time - 2.6 * walktime) - 0.06 + step;
		}
		else if (time < 3.6 * walktime)
		{
			Footpos[i] = -0.06 + 2 * step;
		}
		else if (time < 3.9 * walktime)
		{
			Footpos[i] = -0.06 + 2 * step;
		}
		else
		{
			Footpos[i] = -0.06 + 2 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_rightwalk()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) + 0.06;
		}
		else if (time < 3.1 * walktime)
		{
			Footpos[i] = 0.06 + step;
		}
		else if (time < 3.4 * walktime)
		{
			Footpos[i] = Step(time - 3.1 * walktime) + 0.06 + step;
		}
		else
		{
			Footpos[i] = +0.06 + 2 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_rightwalk()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime)
		{
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime)
		{
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.95 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_rightwalk()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 5;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime)
		{
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime)
		{
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_ysimulation_leftwalk6()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) - 0.06;
		}
		else if (time < 3.1 * walktime)
		{
			Footpos[i] = -0.06 + step;
		}
		else if (time < 3.4 * walktime)
		{
			Footpos[i] = Step(time - 3.1 * walktime) - 0.06 + step;
		}
		else if (time < 4.1 * walktime)
		{
			Footpos[i] = -0.06 + 2 * step;
		}
		else if (time < 4.4 * walktime)
		{
			Footpos[i] = Step(time - 4.1 * walktime) - 0.06 + 2 * step;
		}
		else if (time < 5.1 * walktime)
		{
			Footpos[i] = -0.06 + 3 * step;
		}
		else if (time < 5.4 * walktime)
		{
			Footpos[i] = Step(time - 5.1 * walktime) - 0.06 + 3 * step;
		}
		else if (time < 6.1 * walktime)
		{
			Footpos[i] = -0.06 + 4 * step;
		}
		else if (time < 6.4 * walktime)
		{
			Footpos[i] = Step(time - 6.1 * walktime) - 0.06 + 4 * step;
		}
		else if (time < 7.1 * walktime)
		{
			Footpos[i] = -0.06 + 5 * step;
		}
		else if (time < 7.4 * walktime)
		{
			Footpos[i] = Step(time - 7.1 * walktime) - 0.06 + 5 * step;
		}
		else
		{
			Footpos[i] = -0.06 + 6 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_leftwalk6()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) + 0.06;
		}
		else if (time < 2.6 * walktime)
		{
			Footpos[i] = 0.06 + step;
		}
		else if (time < 2.9 * walktime)
		{
			Footpos[i] = Step(time - 2.6 * walktime) + 0.06 + step;
		}
		else if (time < 3.6 * walktime)
		{
			Footpos[i] = 0.06 + 2 * step;
		}
		else if (time < 3.9 * walktime)
		{
			Footpos[i] = Step(time - 3.6 * walktime) + 0.06 + 2 * step;
		}
		else if (time < 4.6 * walktime)
		{
			Footpos[i] = 0.06 + 3 * step;
		}
		else if (time < 4.9 * walktime)
		{
			Footpos[i] = Step(time - 4.6 * walktime) + 0.06 + 3 * step;
		}
		else if (time < 5.6 * walktime)
		{
			Footpos[i] = 0.06 + 4 * step;
		}
		else if (time < 5.9 * walktime)
		{
			Footpos[i] = Step(time - 5.6 * walktime) + 0.06 + 4 * step;
		}
		else if (time < 6.6 * walktime)
		{
			Footpos[i] = 0.06 + 5 * step;
		}
		else if (time < 6.9 * walktime)
		{
			Footpos[i] = Step(time - 6.6 * walktime) + 0.06 + 5 * step;
		}
		else if (time < 7.6 * walktime)
		{
			Footpos[i] = 0.06 + 6 * step;
		}
		else if (time < 7.9 * walktime)
		{
			Footpos[i] = 0.06 + 6 * step;
		}
		else
		{
			Footpos[i] = 0.06 + 6 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_leftwalk6()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime)
		{
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime)
		{
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
		else if (time < 4.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 4.25 * walktime)
		{
			Footpos[i] = Step(time - 4.05 * walktime);
		}
		else if (time < 4.45 * walktime)
		{
			Footpos[i] = Stride(time - 4.05 * walktime);
		}
		else if (time < 5.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 5.25 * walktime)
		{
			Footpos[i] = Step(time - 5.05 * walktime);
		}
		else if (time < 5.45 * walktime)
		{
			Footpos[i] = Stride(time - 5.05 * walktime);
		}
		else if (time < 6.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 6.25 * walktime)
		{
			Footpos[i] = Step(time - 6.05 * walktime);
		}
		else if (time < 6.45 * walktime)
		{
			Footpos[i] = Stride(time - 6.05 * walktime);
		}
		else if (time < 7.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.25 * walktime)
		{
			Footpos[i] = Step(time - 7.05 * walktime);
		}
		else if (time < 7.45 * walktime)
		{
			Footpos[i] = Stride(time - 7.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_leftwalk6()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime)
		{
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime)
		{
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime)
		{
			Footpos[i] = Step(time - 3.55 * walktime);
		}
		else if (time < 3.95 * walktime)
		{
			Footpos[i] = Stride(time - 3.55 * walktime);
		}
		else if (time < 4.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 4.75 * walktime)
		{
			Footpos[i] = Step(time - 4.55 * walktime);
		}
		else if (time < 4.95 * walktime)
		{
			Footpos[i] = Stride(time - 4.55 * walktime);
		}
		else if (time < 5.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 5.75 * walktime)
		{
			Footpos[i] = Step(time - 5.55 * walktime);
		}
		else if (time < 5.95 * walktime)
		{
			Footpos[i] = Stride(time - 5.55 * walktime);
		}
		else if (time < 6.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 6.75 * walktime)
		{
			Footpos[i] = Step(time - 6.55 * walktime);
		}
		else if (time < 6.95 * walktime)
		{
			Footpos[i] = Stride(time - 6.55 * walktime);
		}
		else if (time < 7.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.75 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.95 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_ysimulation_rightwalk6()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) - 0.06;
		}
		else if (time < 2.6 * walktime)
		{
			Footpos[i] = -0.06 + step;
		}
		else if (time < 2.9 * walktime)
		{
			Footpos[i] = Step(time - 2.6 * walktime) - 0.06 + step;
		}
		else if (time < 3.6 * walktime)
		{
			Footpos[i] = -0.06 + 2 * step;
		}
		else if (time < 3.9 * walktime)
		{
			Footpos[i] = Step(time - 3.6 * walktime) - 0.06 + 2 * step;
		}
		else if (time < 4.6 * walktime)
		{
			Footpos[i] = -0.06 + 3 * step;
		}
		else if (time < 4.9 * walktime)
		{
			Footpos[i] = Step(time - 4.6 * walktime) - 0.06 + 3 * step;
		}
		else if (time < 5.6 * walktime)
		{
			Footpos[i] = -0.06 + 4 * step;
		}
		else if (time < 5.9 * walktime)
		{
			Footpos[i] = Step(time - 5.6 * walktime) - 0.06 + 4 * step;
		}
		else if (time < 6.6 * walktime)
		{
			Footpos[i] = -0.06 + 5 * step;
		}
		else if (time < 6.9 * walktime)
		{
			Footpos[i] = Step(time - 6.6 * walktime) - 0.06 + 5 * step;
		}
		else if (time < 7.6 * walktime)
		{
			Footpos[i] = -0.06 + 6 * step;
		}
		else if (time < 7.9 * walktime)
		{
			Footpos[i] = -0.06 + 6 * step;
		}
		else
		{
			Footpos[i] = -0.06 + 6 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_rightwalk6()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) + 0.06;
		}
		else if (time < 3.1 * walktime)
		{
			Footpos[i] = 0.06 + step;
		}
		else if (time < 3.4 * walktime)
		{
			Footpos[i] = Step(time - 3.1 * walktime) + 0.06 + step;
		}
		else if (time < 4.1 * walktime)
		{
			Footpos[i] = 0.06 + 2 * step;
		}
		else if (time < 4.4 * walktime)
		{
			Footpos[i] = Step(time - 4.1 * walktime) + 0.06 + 2 * step;
		}
		else if (time < 5.1 * walktime)
		{
			Footpos[i] = 0.06 + 3 * step;
		}
		else if (time < 5.4 * walktime)
		{
			Footpos[i] = Step(time - 5.1 * walktime) + 0.06 + 3 * step;
		}
		else if (time < 6.1 * walktime)
		{
			Footpos[i] = 0.06 + 4 * step;
		}
		else if (time < 6.4 * walktime)
		{
			Footpos[i] = Step(time - 6.1 * walktime) + 0.06 + 4 * step;
		}
		else if (time < 7.1 * walktime)
		{
			Footpos[i] = 0.06 + 5 * step;
		}
		else if (time < 7.4 * walktime)
		{
			Footpos[i] = Step(time - 7.1 * walktime) + 0.06 + 5 * step;
		}
		else
		{
			Footpos[i] = 0.06 + 6 * step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_rightwalk6()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.75 * walktime)
		{
			Footpos[i] = Step(time - 2.55 * walktime);
		}
		else if (time < 2.95 * walktime)
		{
			Footpos[i] = Stride(time - 2.55 * walktime);
		}
		else if (time < 3.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.75 * walktime)
		{
			Footpos[i] = Step(time - 3.55 * walktime);
		}
		else if (time < 3.95 * walktime)
		{
			Footpos[i] = Stride(time - 3.55 * walktime);
		}
		else if (time < 4.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 4.75 * walktime)
		{
			Footpos[i] = Step(time - 4.55 * walktime);
		}
		else if (time < 4.95 * walktime)
		{
			Footpos[i] = Stride(time - 4.55 * walktime);
		}
		else if (time < 5.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 5.75 * walktime)
		{
			Footpos[i] = Step(time - 5.55 * walktime);
		}
		else if (time < 5.95 * walktime)
		{
			Footpos[i] = Stride(time - 5.55 * walktime);
		}
		else if (time < 6.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 6.75 * walktime)
		{
			Footpos[i] = Step(time - 6.55 * walktime);
		}
		else if (time < 6.95 * walktime)
		{
			Footpos[i] = Stride(time - 6.55 * walktime);
		}
		else if (time < 7.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.75 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.95 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_rightwalk6()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 9;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else if (time < 3.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.25 * walktime)
		{
			Footpos[i] = Step(time - 3.05 * walktime);
		}
		else if (time < 3.45 * walktime)
		{
			Footpos[i] = Stride(time - 3.05 * walktime);
		}
		else if (time < 4.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 4.25 * walktime)
		{
			Footpos[i] = Step(time - 4.05 * walktime);
		}
		else if (time < 4.45 * walktime)
		{
			Footpos[i] = Stride(time - 4.05 * walktime);
		}
		else if (time < 5.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 5.25 * walktime)
		{
			Footpos[i] = Step(time - 5.05 * walktime);
		}
		else if (time < 5.45 * walktime)
		{
			Footpos[i] = Stride(time - 5.05 * walktime);
		}
		else if (time < 6.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 6.25 * walktime)
		{
			Footpos[i] = Step(time - 6.05 * walktime);
		}
		else if (time < 6.45 * walktime)
		{
			Footpos[i] = Stride(time - 6.05 * walktime);
		}
		else if (time < 7.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 7.25 * walktime)
		{
			Footpos[i] = Step(time - 7.05 * walktime);
		}
		else if (time < 7.45 * walktime)
		{
			Footpos[i] = Stride(time - 7.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_ysimulation_leftwalk_halfstep()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) - 0.06;
		}

		else
		{
			Footpos[i] = step - 0.06;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_leftwalk_halfstep()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) + 0.06;
		}
		else
		{
			Footpos[i] = step + 0.06;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_leftwalk_halfstep()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_zsimulation_leftwalk_halfstep()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_ysimulation_rightwalk_halfstep()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.6 * walktime)
		{
			Footpos[i] = -0.06;
		}
		else if (time < 1.9 * walktime)
		{
			Footpos[i] = Step(time - 1.6 * walktime) - 0.06;
		}
		else
		{
			Footpos[i] = -0.06 + step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_ysimulation_rightwalk_halfstep()
{
	XStep = Equation_solver(0, walktime * 0.3, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 1.4 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.1 * walktime)
		{
			Footpos[i] = 0.06;
		}
		else if (time < 2.4 * walktime)
		{
			Footpos[i] = Step(time - 2.1 * walktime) + 0.06;
		}
		else
		{
			Footpos[i] = 0.06 + step;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_rightwalk_halfstep()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.75 * walktime)
		{
			Footpos[i] = Step(time - 1.55 * walktime);
		}
		else if (time < 1.95 * walktime)
		{
			Footpos[i] = Stride(time - 1.55 * walktime);
		}
		else if (time < 2.55 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_rightwalk_halfstep()
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, 0.05);
	XStride = Equation_solver(0.2 * walktime, 0.4 * walktime, 0.05, 0);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.25 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.45 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.05 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.25 * walktime)
		{
			Footpos[i] = Step(time - 2.05 * walktime);
		}
		else if (time < 2.45 * walktime)
		{
			Footpos[i] = Stride(time - 2.05 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_xsimulation_huddle()
{
	XStep = Equation_solver(0, walktime, 0, 1.4 * step);
	step_n = 6;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 2.5 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 3.5 * walktime)
		{
			Footpos[i] = Step(time - 2.5 * walktime);
		}
		else
		{
			Footpos[i] = 1.4 * step;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_xsimulation_huddle()
{
	XStep = Equation_solver(0, walktime * 0.5, 0, step);
	XStride = Equation_solver(0, walktime * 0.3, 0, 0.4 * step);
	step_n = 6;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.5 * walktime)
		{
			Footpos[i] = Step(time - 1 * walktime);
		}
		else if (time < 5.2 * walktime)
		{
			Footpos[i] = step;
		}
		else if (time < 5.5 * walktime)
		{
			Footpos[i] = Stride(time - 5.2 * walktime) + step;
		}
		else
		{
			Footpos[i] = 1.4 * step;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_zsimulation_huddle(double h, double COM_h)
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, h);
	XStride = Equation_solver(0, 0.2 * walktime, h, 0);
	sim_n = walktime_n * 6;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 0.8 * walktime)
		{
			Footpos[i] = COM_h;
		}
		else if (time < 2.3 * walktime)
		{
			Footpos[i] = COM_h;
		}
		else if (time < 2.5 * walktime)
		{
			Footpos[i] = Step(time - 2.3 * walktime) + COM_h;
		}
		else if (time < 3.5 * walktime)
		{
			Footpos[i] = h + COM_h;
		}
		else if (time < 3.7 * walktime)
		{
			Footpos[i] = Stride(time - 3.5 * walktime) + COM_h;
		}
		else
		{
			Footpos[i] = COM_h;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_huddle(double h, double COM_h)
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, h);
	XStride = Equation_solver(0, 0.2 * walktime, h, 0);
	step_n = 6;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 0.8 * walktime)
		{
			Footpos[i] = COM_h;
		}
		else if (time < 1 * walktime)
		{
			Footpos[i] = Step(time - 0.8 * walktime) + COM_h;
		}
		else if (time < 1.5 * walktime)
		{
			Footpos[i] = h + COM_h;
		}
		else if (time < 1.7 * walktime)
		{
			Footpos[i] = Stride(time - 1.5 * walktime) + COM_h;
		}
		else if (time < 5 * walktime)
		{
			Footpos[i] = COM_h;
		}
		else if (time < 5.2 * walktime)
		{
			Footpos[i] = 0.2 * Step(time - 5 * walktime) + COM_h;
		}
		else if (time < 5.5 * walktime)
		{
			Footpos[i] = 0.2 * h + COM_h;
		}
		else if (time < 5.7 * walktime)
		{
			Footpos[i] = 0.2 * Stride(time - 5.5 * walktime) + COM_h;
		}
		else
		{
			Footpos[i] = COM_h;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_xsimulation_huddle1()
{
	XStep = Equation_solver(0, walktime * 0.5, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 2 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2.5 * walktime)
		{
			Footpos[i] = Step(time - 2 * walktime);
		}
		else
		{
			Footpos[i] = step;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_xsimulation_huddle1()
{
	XStep = Equation_solver(0, walktime * 0.5, 0, step);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n); // rightfoot motion
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;
		if (time < 1 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1.5 * walktime)
		{
			Footpos[i] = Step(time - 1 * walktime);
		}
		else
		{
			Footpos[i] = step;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_zsimulation_huddle1(double h)
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, h);
	XStride = Equation_solver(0, 0.2 * walktime, h, 0);
	sim_n = walktime_n * 3;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 1.8 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 2 * walktime)
		{
			Footpos[i] = Step(time - 1.8 * walktime);
		}
		else if (time < 2.5 * walktime)
		{
			Footpos[i] = h;
		}
		else if (time < 2.7 * walktime)
		{
			Footpos[i] = Stride(time - 2.5 * walktime);
		}
		else if (time < 3 * walktime)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::LF_zsimulation_huddle1(double h)
{
	XStep = Equation_solver(0, 0.2 * walktime, 0, h);
	XStride = Equation_solver(0, 0.2 * walktime, h, 0);
	step_n = 3;
	sim_n = walktime_n * step_n;
	double del_t = 1 / freq;
	RowVectorXd Footpos(sim_n);
	for (int i = 0; i < sim_n; i++)
	{
		double time = i * del_t;

		if (time < 0.8 * walktime)
		{
			Footpos[i] = 0;
		}
		else if (time < 1 * walktime)
		{
			Footpos[i] = Step(time - 0.8 * walktime);
		}
		else if (time < 1.5 * walktime)
		{
			Footpos[i] = h;
		}
		else if (time < 1.7 * walktime)
		{
			Footpos[i] = Stride(time - 1.5 * walktime);
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
};

MatrixXd Trajectory::RF_zsimulation_sitdown(double h)
{
	XStep = Equation_solver(0, 60, 0, h);
	RowVectorXd Footpos(100);
	for (int i = 0; i < 100; i++)
	{
		if (i < 20)
		{
			Footpos[i] = 0;
		}
		else if (i < 80)
		{
			Footpos[i] = Step(i - 20);
		}
		else if (i < 100)
		{
			Footpos[i] = h;
		}
		else
		{
			Footpos[i] = h;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_zsimulation_sitdown(double h)
{
	XStep = Equation_solver(0, 60, 0, h);
	RowVectorXd Footpos(100);
	for (int i = 0; i < 100; i++)
	{
		if (i < 20)
		{
			Footpos[i] = 0;
		}
		else if (i < 80)
		{
			Footpos[i] = Step(i - 20);
		}
		else if (i < 100)
		{
			Footpos[i] = h;
		}
		else
		{
			Footpos[i] = h;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_standup(double h)
{
	XStep = Equation_solver(0, 60, h, 0);
	RowVectorXd Footpos(100);
	for (int i = 0; i < 100; i++)
	{
		if (i < 20)
		{
			Footpos[i] = h;
		}
		else if (i < 80)
		{
			Footpos[i] = Step(i - 20);
		}
		else if (i < 100)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_zsimulation_standup(double h)
{
	XStep = Equation_solver(0, 60, h, 0);
	RowVectorXd Footpos(100);
	for (int i = 0; i < 100; i++)
	{
		if (i < 20)
		{
			Footpos[i] = h;
		}
		else if (i < 80)
		{
			Footpos[i] = Step(i - 20);
		}
		else if (i < 100)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_sitdown3(double h)
{
	XStep = Equation_solver(0, 300, 0, h);
	RowVectorXd Footpos(500);
	for (int i = 0; i < 500; i++)
	{
		if (i < 100)
		{
			Footpos[i] = 0;
		}
		else if (i < 400)
		{
			Footpos[i] = Step(i - 100);
		}
		else if (i < 500)
		{
			Footpos[i] = h;
		}
		else
		{
			Footpos[i] = h;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_zsimulation_sitdown3(double h)
{
	XStep = Equation_solver(0, 300, 0, h);
	RowVectorXd Footpos(500);
	for (int i = 0; i < 500; i++)
	{
		if (i < 100)
		{
			Footpos[i] = 0;
		}
		else if (i < 400)
		{
			Footpos[i] = Step(i - 100);
		}
		else if (i < 500)
		{
			Footpos[i] = h;
		}
		else
		{
			Footpos[i] = h;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::RF_zsimulation_standup3(double h)
{
	XStep = Equation_solver(0, 300, h, 0);
	RowVectorXd Footpos(500);
	for (int i = 0; i < 500; i++)
	{
		if (i < 100)
		{
			Footpos[i] = h;
		}
		else if (i < 400)
		{
			Footpos[i] = Step(i - 100);
		}
		else if (i < 500)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

MatrixXd Trajectory::LF_zsimulation_standup3(double h)
{
	XStep = Equation_solver(0, 300, h, 0);
	RowVectorXd Footpos(500);
	for (int i = 0; i < 500; i++)
	{
		if (i < 100)
		{
			Footpos[i] = h;
		}
		else if (i < 400)
		{
			Footpos[i] = Step(i - 100);
		}
		else if (i < 500)
		{
			Footpos[i] = 0;
		}
		else
		{
			Footpos[i] = 0;
		}
	};
	return Footpos;
}

void Trajectory::Go_Straight(double step, double distance, double height)
{
	Set_step(step);
	Set_distance(distance);
	Xcom = XComSimulation();
	Ycom = YComSimulation();
	LF_xFoot = LF_xsimulation_straightwalk();
	RF_xFoot = RF_xsimulation_straightwalk();
	RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	Ref_RL_x = RF_xFoot - Xcom;
	Ref_LL_x = LF_xFoot - Xcom;
	Ref_RL_y = RF_yFoot - Ycom;
	Ref_LL_y = LF_yFoot - Ycom;
	Ref_RL_z = RF_zsimulation_straightwalk(height);
	Ref_LL_z = LF_zsimulation_straightwalk(height);
}

void Trajectory::Freq_Change_Straight(double step, double distance, double height, double freq)
{
	Change_Freq(freq);
	Set_step(step);
	Set_distance(distance);

	MatrixXd Xcom_ = XComSimulation();
	MatrixXd Ycom_ = YComSimulation();
	MatrixXd LF_xFoot = LF_xsimulation_straightwalk();
	MatrixXd RF_xFoot = RF_xsimulation_straightwalk();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	Ref_RL_x = RF_xFoot - Xcom_;
	Ref_LL_x = LF_xFoot - Xcom_;
	Ref_RL_y = RF_yFoot - Ycom_;
	Ref_LL_y = LF_yFoot - Ycom_;
	// Ref_RL_x = RF_xFoot.block(0, 0, RF_xFoot.rows(), sim_n) - Xcom.block(0, 0, RF_xFoot.rows(), sim_n);
	// Ref_LL_x = LF_xFoot.block(0, 0, LF_xFoot.rows(), sim_n) - Xcom.block(0, 0, LF_xFoot.rows(), sim_n);
	// Ref_RL_y = RF_yFoot.block(0, 0, RF_yFoot.rows(), sim_n) - Ycom.block(0, 0, RF_yFoot.rows(), sim_n);
	// Ref_LL_y = LF_yFoot.block(0, 0, LF_yFoot.rows(), sim_n) - Ycom.block(0, 0, LF_yFoot.rows(), sim_n);
	Ref_RL_z = RF_zsimulation_straightwalk(height);
	Ref_LL_z = LF_zsimulation_straightwalk(height);
};

void Trajectory::Side_Left2()
{
	Set_step(0.06);
	MatrixXd Ycom_ = YComSimulation_Sidewalk(-L0, -L0, 2 * L0, 0, 3 * L0, 2 * L0);
	MatrixXd RF_yFoot = RF_ysimulation_leftwalk();
	MatrixXd LF_yFoot = LF_ysimulation_leftwalk();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom_;
	Ref_LL_y = LF_yFoot - Ycom_;
	Ref_RL_z = RF_zsimulation_leftwalk();
	Ref_LL_z = LF_zsimulation_leftwalk();
}

void Trajectory::Side_Right2()
{
	Set_step(-0.06);
	MatrixXd Ycom_ = YComSimulation_Sidewalk(L0, L0, -2 * L0, 0, -3 * L0, -2 * L0);
	MatrixXd RF_yFoot = RF_ysimulation_rightwalk();
	MatrixXd LF_yFoot = LF_ysimulation_rightwalk();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom_;
	Ref_LL_y = LF_yFoot - Ycom_;
	Ref_RL_z = RF_zsimulation_rightwalk();
	Ref_LL_z = LF_zsimulation_rightwalk();
}

void Trajectory::Side_Left1(double step)
{
	Set_step(step);
	MatrixXd Ycom = YComSimulation_Sidewalk_half(-L0, -L0, L0 + step, L0 - step);
	MatrixXd LF_yFoot = LF_ysimulation_leftwalk_halfstep();
	MatrixXd RF_yFoot = RF_ysimulation_leftwalk_halfstep();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom;
	Ref_LL_y = LF_yFoot - Ycom;
	Ref_RL_z = RF_zsimulation_leftwalk_halfstep();
	Ref_LL_z = LF_zsimulation_leftwalk_halfstep();
}

void Trajectory::Side_Right1(double step)
{
	Set_step(-step);
	MatrixXd Ycom = YComSimulation_Sidewalk_half(L0, L0, - L0 - step, -L0 + step);
	MatrixXd LF_yFoot = LF_ysimulation_rightwalk_halfstep();
	MatrixXd RF_yFoot = RF_ysimulation_rightwalk_halfstep();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom;
	Ref_LL_y = LF_yFoot - Ycom;
	Ref_RL_z = RF_zsimulation_rightwalk_halfstep();
	Ref_LL_z = LF_zsimulation_rightwalk_halfstep();
}

void Trajectory::Side_Left6()
{
	Set_step(0.06);
	MatrixXd Ycom_ = YComSimulation_Sidewalk6(0.06);
	MatrixXd RF_yFoot = RF_ysimulation_leftwalk6();
	MatrixXd LF_yFoot = LF_ysimulation_leftwalk6();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom_;
	Ref_LL_y = LF_yFoot - Ycom_;
	Ref_RL_z = RF_zsimulation_leftwalk6();
	Ref_LL_z = LF_zsimulation_leftwalk6();
}

void Trajectory::Side_Right6()
{
	Set_step(-0.06);
	MatrixXd Ycom_ = YComSimulation_Sidewalk6(-0.06);
	MatrixXd RF_yFoot = RF_ysimulation_rightwalk6();
	MatrixXd LF_yFoot = LF_ysimulation_rightwalk6();
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom_;
	Ref_LL_y = LF_yFoot - Ycom_;
	Ref_RL_z = RF_zsimulation_rightwalk6();
	Ref_LL_z = LF_zsimulation_rightwalk6();
}

void Trajectory::Step_in_place(double step, double distance, double height)
{
	Set_step(step);
	Set_distance(distance);
	Ycom = YComSimulation();
	RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	Ref_RL_x = MatrixXd::Zero(1, sim_n);
	Ref_LL_x = MatrixXd::Zero(1, sim_n);
	Ref_RL_y = RF_yFoot - Ycom;
	Ref_LL_y = LF_yFoot - Ycom;
	Ref_RL_z = RF_zsimulation_straightwalk(height);
	Ref_LL_z = LF_zsimulation_straightwalk(height);
}

void Trajectory::Stop_Trajectory_straightwalk(double step)
{
	Set_step(step);
	Set_distance(0.2);
	MatrixXd sXcom = XComSimulation();
	MatrixXd sYcom = YComSimulation();
	MatrixXd sLF_xFoot = LF_xsimulation_straightwalk();
	MatrixXd sRF_xFoot = RF_xsimulation_straightwalk();
	MatrixXd sRF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd sLF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	int numCols = sRF_xFoot.cols() / 3;
	lsRef_RL_x = sRF_xFoot.block(0, numCols, 1, 2 * numCols) - sXcom.block(0, numCols, 1, 2 * numCols);
	lsRef_LL_x = sLF_xFoot.block(0, numCols, 1, 2 * numCols) - sXcom.block(0, numCols, 1, 2 * numCols);
	lsRef_RL_y = sRF_yFoot.block(0, numCols, 1, 2 * numCols) - sYcom.block(0, numCols, 1, 2 * numCols);
	lsRef_LL_y = sLF_yFoot.block(0, numCols, 1, 2 * numCols) - sYcom.block(0, numCols, 1, 2 * numCols);
	lsRef_RL_z = RF_zsimulation_straightwalk(0.05).block(0, numCols, 1, 2 * numCols);
	lsRef_LL_z = LF_zsimulation_straightwalk(0.05).block(0, numCols, 1, 2 * numCols);
	lsRef_RL_x = rsRef_RL_x.rowwise().reverse();
	lsRef_LL_x = rsRef_LL_x.rowwise().reverse();
	lsRef_RL_y = rsRef_RL_y.rowwise().reverse();
	lsRef_LL_y = rsRef_LL_y.rowwise().reverse();
	lsRef_RL_z = rsRef_RL_z.rowwise().reverse();
	lsRef_LL_z = rsRef_LL_z.rowwise().reverse();

	rsRef_RL_x = sRF_xFoot.block(0, 0, 1, numCols) - sXcom.block(0, 0, 1, numCols);
	rsRef_LL_x = sLF_xFoot.block(0, 0, 1, numCols) - sXcom.block(0, 0, 1, numCols);
	rsRef_RL_y = sRF_yFoot.block(0, 0, 1, numCols) - sYcom.block(0, 0, 1, numCols);
	rsRef_LL_y = sLF_yFoot.block(0, 0, 1, numCols) - sYcom.block(0, 0, 1, numCols);
	rsRef_RL_z = RF_zsimulation_straightwalk(0.05).block(0, 0, 1, numCols);
	rsRef_LL_z = LF_zsimulation_straightwalk(0.05).block(0, 0, 1, numCols);
}

void Trajectory::Stop_Trajectory_stepinplace(double step)
{
	Set_step(step);
	Set_distance(0.2);
	MatrixXd sYcom = YComSimulation();
	MatrixXd sLF_xFoot = LF_xsimulation_straightwalk();
	MatrixXd sRF_xFoot = RF_xsimulation_straightwalk();
	MatrixXd sRF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd sLF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	int numCols = sRF_xFoot.cols() / 3;
	lsRef_RL_x = MatrixXd::Zero(1, sim_n);
	lsRef_LL_x = MatrixXd::Zero(1, sim_n);
	lsRef_RL_y = sRF_yFoot.block(0, numCols, 1, 2 * numCols) - sYcom.block(0, numCols, 1, 2 * numCols);
	lsRef_LL_y = sLF_yFoot.block(0, numCols, 1, 2 * numCols) - sYcom.block(0, numCols, 1, 2 * numCols);
	lsRef_RL_z = RF_zsimulation_straightwalk(0.05).block(0, numCols, 1, 2 * numCols);
	lsRef_LL_z = LF_zsimulation_straightwalk(0.05).block(0, numCols, 1, 2 * numCols);
	lsRef_RL_y = rsRef_RL_y.rowwise().reverse();
	lsRef_LL_y = rsRef_LL_y.rowwise().reverse();
	lsRef_RL_z = rsRef_RL_z.rowwise().reverse();
	lsRef_LL_z = rsRef_LL_z.rowwise().reverse();

	rsRef_RL_x = MatrixXd::Zero(1, sim_n);
	rsRef_LL_x = MatrixXd::Zero(1, sim_n);
	rsRef_RL_y = sRF_yFoot.block(0, 0, 1, numCols) - sYcom.block(0, 0, 1, numCols);
	rsRef_LL_y = sLF_yFoot.block(0, 0, 1, numCols) - sYcom.block(0, 0, 1, numCols);
	rsRef_RL_z = RF_zsimulation_straightwalk(0.05).block(0, 0, 1, numCols);
	rsRef_LL_z = LF_zsimulation_straightwalk(0.05).block(0, 0, 1, numCols);
}

void Trajectory::Huddle_Motion(double step, double height, double COM_h)
{
	Set_step(step);
	MatrixXd Ref_RL_x_t = RF_xsimulation_huddle() - Huddle_Xcom();
	MatrixXd Ref_LL_x_t = LF_xsimulation_huddle() - Huddle_Xcom();
	MatrixXd Ref_RL_y_t = -L0 * MatrixXd::Ones(1, sim_n) - Huddle_Ycom();
	MatrixXd Ref_LL_y_t = L0 * MatrixXd::Ones(1, sim_n) - Huddle_Ycom();
	MatrixXd Ref_RL_z_t = RF_zsimulation_huddle(height, COM_h);
	MatrixXd Ref_LL_z_t = LF_zsimulation_huddle(height, COM_h);
	MatrixXd Temp_RL_x(Ref_RL_x_t.rows(), MatrixXd::Zero(1, 100).cols() + Ref_RL_x_t.cols() + MatrixXd::Zero(1, 100).cols());
	MatrixXd Temp_LL_x(Ref_LL_x_t.rows(), MatrixXd::Zero(1, 100).cols() + Ref_LL_x_t.cols() + MatrixXd::Zero(1, 100).cols());
	MatrixXd Temp_RL_y(Ref_RL_y_t.rows(), MatrixXd::Ones(1, 100).cols() + Ref_RL_y_t.cols() + MatrixXd::Ones(1, 100).cols());
	MatrixXd Temp_LL_y(Ref_LL_y_t.rows(), MatrixXd::Ones(1, 100).cols() + Ref_LL_y_t.cols() + MatrixXd::Ones(1, 100).cols());
	MatrixXd Temp_RL_z(Ref_RL_z_t.rows(), RF_zsimulation_sitdown(COM_h).cols() + Ref_RL_z_t.cols() + RF_zsimulation_standup(COM_h).cols());
	MatrixXd Temp_LL_z(Ref_LL_z_t.rows(), LF_zsimulation_sitdown(COM_h).cols() + Ref_LL_z_t.cols() + LF_zsimulation_standup(COM_h).cols());
	Temp_RL_x << MatrixXd::Zero(1, 100), Ref_RL_x_t, MatrixXd::Zero(1, 100);
	Temp_LL_x << MatrixXd::Zero(1, 100), Ref_LL_x_t, MatrixXd::Zero(1, 100);
	Temp_RL_y << -L0 * MatrixXd::Ones(1, 100), Ref_RL_y_t, -L0 * MatrixXd::Ones(1, 100);
	Temp_LL_y << L0 * MatrixXd::Ones(1, 100), Ref_LL_y_t, L0 * MatrixXd::Ones(1, 100);
	Temp_RL_z << RF_zsimulation_sitdown(COM_h), Ref_RL_z_t, RF_zsimulation_standup(COM_h);
	Temp_LL_z << LF_zsimulation_sitdown(COM_h), Ref_LL_z_t, LF_zsimulation_standup(COM_h);
	Ref_RL_x = Temp_RL_x;
	Ref_LL_x = Temp_LL_x;
	Ref_RL_y = Temp_RL_y;
	Ref_LL_y = Temp_LL_y;
	Ref_RL_z = Temp_RL_z;
	Ref_LL_z = Temp_LL_z;
}

void Trajectory::Huddle_Motion1(double step, double height)
{
	Set_step(step);
	MatrixXd Xcom = Huddle_Xcom();
	MatrixXd Ycom = Huddle_Ycom();
	MatrixXd LF_xFoot = LF_xsimulation_huddle();
	MatrixXd RF_xFoot = RF_xsimulation_huddle();
	MatrixXd RF_yFoot = -L0 * MatrixXd::Ones(1, sim_n);
	MatrixXd LF_yFoot = L0 * MatrixXd::Ones(1, sim_n);
	Ref_RL_x = RF_xFoot - Xcom;
	Ref_LL_x = LF_xFoot - Xcom;
	Ref_RL_y = RF_yFoot - Ycom;
	Ref_LL_y = LF_yFoot - Ycom;
	Ref_RL_z = RF_zsimulation_huddle(height, 50);
	Ref_LL_z = LF_zsimulation_huddle(height, 50);
}

void Trajectory::Make_turn_trajectory(double angle)
{
	Angle_trajectorty_turn = Equation_solver(0, walktime_n * 0.3, 0, angle);
	Angle_trajectorty_back = Equation_solver(0, walktime_n * 0.3, angle, 0);
	Turn_Trajectory = VectorXd::Zero(walktime_n);
	for (int i = 0; i < walktime_n; i++)
	{
		if (i < 0.1 * walktime_n)
		{
			Turn_Trajectory(i) = 0;
		}
		else if (i < 0.4 * walktime_n)
		{
			Turn_Trajectory(i) = Return_turn_trajectory(i - 0.1 * walktime_n);
		}
		else if (i < 0.6 * walktime_n)
		{
			Turn_Trajectory(i) = angle;
		}
		else if (i < 0.9 * walktime_n)
		{
			Turn_Trajectory(i) = Return_back_trajectory(i - 0.6 * walktime_n);
		}
		else
			Turn_Trajectory(i) = 0;
	}
}

double Trajectory::Return_turn_trajectory(double t)
{
	double X = Angle_trajectorty_turn(0) + Angle_trajectorty_turn(1) * t + Angle_trajectorty_turn(2) * pow(t, 2) + Angle_trajectorty_turn(3) * pow(t, 3) + Angle_trajectorty_turn(4) * pow(t, 4) + Angle_trajectorty_turn(5) * pow(t, 5);
	return X;
}

double Trajectory::Return_back_trajectory(double t)
{
	double X = Angle_trajectorty_back(0) + Angle_trajectorty_back(1) * t + Angle_trajectorty_back(2) * pow(t, 2) + Angle_trajectorty_back(3) * pow(t, 3) + Angle_trajectorty_back(4) * pow(t, 4) + Angle_trajectorty_back(5) * pow(t, 5);
	return X;
}

IK_Function::IK_Function()
{
	walkfreq = 1.48114;
	walktime = 2 / walkfreq;
	freq = 100;
	walktime_n = walktime * freq;
	step = 0.05;
	freq = 100;
	step_n = 0;
	sim_n = 0;
	sim_time = 0;

	RL_th[0] = 0. * deg2rad;   // RHY
	RL_th[1] = 0. * deg2rad;   // RHR
	RL_th[2] = -35. * deg2rad; // RHP
	RL_th[3] = 70. * deg2rad;  // RKN
	RL_th[4] = -35. * deg2rad; // RAP
	RL_th[5] = 0. * deg2rad;   // RAR

	LL_th[0] = 0. * deg2rad;   // LHY
	LL_th[1] = 0. * deg2rad;   // LHR
	LL_th[2] = -35. * deg2rad; // LHP
	LL_th[3] = 70. * deg2rad;  // LKN
	LL_th[4] = -35. * deg2rad; // LAP
	LL_th[5] = 0. * deg2rad;   // LAR

	Ref_RL_PR[0] = 40.;
	Ref_RL_PR[1] = -L0;
	Ref_RL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40.;
	Ref_RL_PR[3] = 0 * deg2rad;
	Ref_RL_PR[4] = 0 * deg2rad;
	Ref_RL_PR[5] = 0 * deg2rad;

	Ref_LL_PR[0] = 40.;
	Ref_LL_PR[1] = L0;
	Ref_LL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + 40.;
	Ref_LL_PR[3] = 0 * deg2rad;
	Ref_LL_PR[4] = 0 * deg2rad;
	Ref_LL_PR[5] = 0 * deg2rad;

	RL_Support_Leg = 0;
	RL_Swing_Leg = 0;
	RL_Support_Knee = 0;
	LL_Support_Leg = 0;
	LL_Swing_Leg = 0;
	LL_Support_Knee = 0;
	Com_Height = 30;
}

void IK_Function::Get_Step_n(double a)
{
	step_n = a;
	sim_n = walktime_n * step_n;
}

void IK_Function::BRP_RL_FK(double th[6], double PR[6])
{
	double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, nx, ny, nz, ox, oy, oz, ax, ay, az;

	c1 = cos(th[0]);
	c2 = cos(th[1]);
	c3 = cos(th[2]);
	c4 = cos(th[3]);
	c5 = cos(th[4]);
	c6 = cos(th[5]);
	s1 = sin(th[0]);
	s2 = sin(th[1]);
	s3 = sin(th[2]);
	s4 = sin(th[3]);
	s5 = sin(th[4]);
	s6 = sin(th[5]);

	// Endeffector position
	PR[0] = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - L4 * s4 * (c1 * c3 - s1 * s2 * s3) - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2) - L2 * s1 * s2 - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;
	PR[1] = L2 * c1 * s2 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3) - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - L0 + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - L3 * s1 * s3 - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;
	PR[2] = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;

	// Endeffector orientation
	nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
	ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
	nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

	// printf("nx = %f, ny = %f, nz = %f \n",nx,ny,nz);

	ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
	oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
	oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

	// printf("ox = %f, oy = %f, oz = %f \n",ox,oy,oz);

	ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
	ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
	az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

	// printf("ax = %f, ay = %f, az = %f \n",ax,ay,az);

	PR[5] = atan2(ny, nx);								   // FI : Roll about z0 axis
	PR[4] = atan2(-nz, cos(PR[5]) * nx + sin(PR[5]) * ny); // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5]) * ax - cos(PR[5]) * ay, -sin(PR[5]) * ox + cos(PR[5]) * oy);
};

void IK_Function::BRP_RL_IK(double Ref_RL_RP[6], double Init_th[6], double IK_th[6])
{
	int iter, i, j, k;
	double th[6] = {0., 0., 0., 0., 0., 0.}, PR[6] = {0., 0., 0., 0., 0., 0.}, old_PR[6] = {0., 0., 0., 0., 0., 0.}, F[6] = {0., 0., 0., 0., 0., 0.}, old_Q[6] = {0., 0., 0., 0., 0., 0.},
		   New_Q4J[6] = {0., 0., 0., 0., 0., 0.}, ERR = 0., sum = 0.,
		   New_PR[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}},
		   J[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}},
		   Inv_J[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}};
	const double del_Q = 0.0001;
	for (i = 0; i < 6; i++)
		th[i] = Init_th[i];
	for (iter = 0; iter < 100; iter++)
	{

		for (i = 0; i < 6; i++)
			old_Q[i] = th[i];
		BRP_RL_FK(th, PR);

		for (i = 0; i < 6; i++)
			F[i] = Ref_RL_RP[i] - PR[i];

		ERR = sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2] + F[3] * F[3] + F[4] * F[4] + F[5] * F[5]);

		if (ERR < 0.0001)
		{
			for (i = 0; i < 6; i++)
				IK_th[i] = th[i];
			break;
		}
		else if (iter == 99)
		{
			for (i = 0; i < 6; i++)
				IK_th[i] = Init_th[i];

			break;
		}

		for (i = 0; i < 6; i++)
			old_PR[i] = PR[i];

		for (i = 0; i < 6; i++)
		{

			for (j = 0; j < 6; j++)
				New_Q4J[j] = old_Q[j];

			New_Q4J[i] = old_Q[i] + del_Q;

			BRP_RL_FK(New_Q4J, PR);

			for (j = 0; j < 6; j++)
				New_PR[j][i] = PR[j];
		}

		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++)
				J[i][j] = (New_PR[i][j] - old_PR[i]) / del_Q;

		inv_mat6(0, 6, 0, J, 0, Inv_J);

		for (k = 0; k < 6; k++)
		{
			sum = 0.;
			for (j = 0; j < 6; j++)
			{
				sum = sum + Inv_J[k][j] * F[j];
			}
			th[k] = old_Q[k] + sum;
		}

		if (th[3] < 0)
		{
			th[3] = -th[3];
		}
	}
}

void IK_Function::BRP_LL_FK(double th[6], double PR[6])
{
	double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, nx, ny, nz, ox, oy, oz, ax, ay, az;

	c1 = cos(th[0]);
	c2 = cos(th[1]);
	c3 = cos(th[2]);
	c4 = cos(th[3]);
	c5 = cos(th[4]);
	c6 = cos(th[5]);
	s1 = sin(th[0]);
	s2 = sin(th[1]);
	s3 = sin(th[2]);
	s4 = sin(th[3]);
	s5 = sin(th[4]);
	s6 = sin(th[5]);

	// Endeffector position
	PR[0] = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - L4 * s4 * (c1 * c3 - s1 * s2 * s3) - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2) - L2 * s1 * s2 - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;
	PR[1] = L0 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3) - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) + L2 * c1 * s2 + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - L3 * s1 * s3 - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;
	PR[2] = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;

	// Endeffector orientation
	nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
	ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
	nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

	ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
	oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
	oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

	ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
	ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
	az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

	PR[5] = atan2(ny, nx);																  // FI : Roll about z0 axis
	PR[4] = atan2(-nz, cos(PR[5]) * nx + sin(PR[5]) * ny);								  // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5]) * ax - cos(PR[5]) * ay, -sin(PR[5]) * ox + cos(PR[5]) * oy); // csi : Yaw about x0 axis
};

void IK_Function::BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6])
{
	int iter, i, j, k;
	double th[6] = {0., 0., 0., 0., 0., 0.}, PR[6] = {0., 0., 0., 0., 0., 0.}, old_PR[6] = {0., 0., 0., 0., 0., 0.}, F[6] = {0., 0., 0., 0., 0., 0.}, old_Q[6] = {0., 0., 0., 0., 0., 0.},
		   New_Q4J[6] = {0., 0., 0., 0., 0., 0.}, ERR = 0., sum = 0.,
		   New_PR[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}},
		   J[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}},
		   Inv_J[6][6] = {{0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.}};
	const double del_Q = 0.0001;

	// Initial Joint angles //
	for (i = 0; i < 6; i++)
		th[i] = Init_th[i];

	// printf("th[0] = %10.8f,	th[1] = %10.8f,	th[2] = %10.8f \n",th[0], th[1], th[2]);

	for (iter = 0; iter < 100; iter++)
	{

		// printf("iter = %d, ERR = %f \n",iter, ERR);

		for (i = 0; i < 6; i++)
			old_Q[i] = th[i];

		// Forward Kinematics
		BRP_LL_FK(th, PR);

		// Error calculation
		for (i = 0; i < 6; i++)
			F[i] = Ref_LL_RP[i] - PR[i];

		ERR = sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2] + F[3] * F[3] + F[4] * F[4] + F[5] * F[5]);

		if (ERR < 0.0001)
		{
			for (i = 0; i < 6; i++)
				IK_th[i] = th[i];

			break;
		}
		else if (iter == 99)
		{
			for (i = 0; i < 6; i++)
				IK_th[i] = Init_th[i];

			break;
		}

		// Jacobian Cacluation using perturbation //
		for (i = 0; i < 6; i++)
			old_PR[i] = PR[i];

		for (i = 0; i < 6; i++)
		{

			for (j = 0; j < 6; j++)
				New_Q4J[j] = old_Q[j]; // Reset

			New_Q4J[i] = old_Q[i] + del_Q; // Perturb

			// Forward Kinematics again //
			BRP_LL_FK(New_Q4J, PR);

			for (j = 0; j < 6; j++)
				New_PR[j][i] = PR[j];
		} // End of for(i=0; i<6; i++){

		for (i = 0; i < 6; i++)
			for (j = 0; j < 6; j++)
				J[i][j] = (New_PR[i][j] - old_PR[i]) / del_Q;

		inv_mat6(0, 6, 0, J, 0, Inv_J);

		// printf("old_Q[0] = %10.8f,	old_Q[1] = %10.8f,	old_Q[2] = %10.8f \n",old_Q[0], old_Q[1], old_Q[2]);

		for (k = 0; k < 6; k++)
		{
			sum = 0.;
			for (j = 0; j < 6; j++)
			{
				sum = sum + Inv_J[k][j] * F[j];
			}
			// printf("sum = %10.8f\n",sum);

			th[k] = old_Q[k] + sum;
		}

		// printf("R_iter = %d, ERR = %f,	th[0] = %10.8f,	th[1] = %10.8f,	th[2] = %10.8f \n",iter, ERR, th[0], th[1], th[2]);
		if (th[3] < 0)
		{ // ���������� (-)  �� �����ϴ� ���� ����
			th[3] = -th[3];
		}

	} // End of for (iter = 0; iter <= 100; iter++) {
};

void IK_Function::inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6])
{
	double big, size, abig, cbig, ratio;
	int i, k, j, ibig;
	double jcob[6][6];

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (n == 4)
				jcob[i][j] = Mat4[i][j];
			else if (n == 6)
				jcob[i][j] = Mat6[i][j];
		}
	}

	if (n == 6)
	{
		for (i = m; i < n; i++)
		{
			for (j = m; j < n; j++)
			{
				c_inv[i][j] = 0.;
				if (i == j)
					c_inv[i][i] = 1.;
			}
		}
		for (k = m; k < n; k++)
		{
			big = fabs(jcob[k][k]);
			ibig = k;

			for (i = k; i < n; i++)
			{
				size = fabs(jcob[i][k]);
				if (size < big)
					goto next;
				big = size;
				ibig = i;
			next:;
			}

			if (k == ibig)
				goto next2;
			for (j = m; j < n; j++)
			{
				if (j >= k)
				{
					abig = jcob[ibig][j];
					jcob[ibig][j] = jcob[k][j];
					jcob[k][j] = abig;
				}
				cbig = c_inv[ibig][j];
				c_inv[ibig][j] = c_inv[k][j];
				c_inv[k][j] = cbig;
			}

		next2:;

			if (jcob[k][k] == 0.)
			{ /*lcd_putch('S'); halt_e();return(1);*/
			}
			for (i = m; i < n; i++)
			{
				if (i == k)
					goto next3;
				ratio = jcob[i][k] / jcob[k][k];
				for (j = m; j < n; j++)
				{
					if (j >= k)
						jcob[i][j] = jcob[i][j] - ratio * jcob[k][j];
					c_inv[i][j] = c_inv[i][j] - ratio * c_inv[k][j];
				}

			next3:;
			}
		}
		for (k = m; k < n; k++)
		{
			for (j = m; j < n; j++)
			{
				c_inv[k][j] = c_inv[k][j] / jcob[k][k];
			}
		}
	}

	else if (n == 4)
	{
		for (i = m; i < n; i++)
		{
			for (j = m; j < n; j++)
			{
				c_inv4[i][j] = 0.;
				if (i == j)
					c_inv4[i][i] = 1.;
			}
		}
		for (k = m; k < n; k++)
		{
			big = fabs(jcob[k][k]);
			ibig = k;

			for (i = k; i < n; i++)
			{
				size = fabs(jcob[i][k]);
				if (size < big)
					goto next1_1;
				big = size;
				ibig = i;
			next1_1:;
			}

			if (k == ibig)
				goto next2_1;
			for (j = m; j < n; j++)
			{
				if (j >= k)
				{
					abig = jcob[ibig][j];
					jcob[ibig][j] = jcob[k][j];
					jcob[k][j] = abig;
				}
				cbig = c_inv4[ibig][j];
				c_inv4[ibig][j] = c_inv4[k][j];
				c_inv4[k][j] = cbig;
			}

		next2_1:;

			if (jcob[k][k] == 0.)
			{ /*lcd_putch('S'); halt_e();return(1);*/
			}
			for (i = m; i < n; i++)
			{
				if (i == k)
					goto next3_1;
				ratio = jcob[i][k] / jcob[k][k];
				for (j = m; j < n; j++)
				{
					if (j >= k)
						jcob[i][j] = jcob[i][j] - ratio * jcob[k][j];
					c_inv4[i][j] = c_inv4[i][j] - ratio * c_inv4[k][j];
				}

			next3_1:;
			}
		}
		for (k = m; k < n; k++)
		{
			for (j = m; j < n; j++)
			{
				c_inv4[k][j] = c_inv4[k][j] / jcob[k][k];
			}
		}
	}
}

void IK_Function::BRP_Simulation(MatrixXd RFx, MatrixXd RFy, MatrixXd RFz, MatrixXd LFx, MatrixXd LFy, MatrixXd LFz, int Index_CNT)
{

	if (Index_CNT > RFx.cols() - 1)
	{
		Index_CNT = RFx.cols() - 1;
	}
	Ref_RL_PR[0] = 1000 * RFx(0, Index_CNT);
	Ref_RL_PR[1] = 1000 * RFy(0, Index_CNT);
	Ref_RL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + Com_Height + 1000 * RFz(0, Index_CNT);
	Ref_RL_PR[3] = 0 * deg2rad;
	Ref_RL_PR[4] = 0 * deg2rad;
	Ref_RL_PR[5] = 0 * deg2rad;

	Ref_LL_PR[0] = 1000 * LFx(0, Index_CNT);
	Ref_LL_PR[1] = 1000 * LFy(0, Index_CNT);
	Ref_LL_PR[2] = -L1 - L2 - L3 - L4 - L5 - L6 + Com_Height + 1000 * LFz(0, Index_CNT);
	Ref_LL_PR[3] = 0 * deg2rad;
	Ref_LL_PR[4] = 0 * deg2rad;
	Ref_LL_PR[5] = 0 * deg2rad;

	BRP_RL_IK(Ref_RL_PR, RL_th, RL_th_IK);
	BRP_LL_IK(Ref_LL_PR, LL_th, LL_th_IK);

	for (int i = 0; i < 6; i++)
	{

		RL_th[i] = RL_th_IK[i];
		LL_th[i] = LL_th_IK[i];
	}
}

void IK_Function::Change_Com_Height(double h)
{
	Com_Height = h;
}

void IK_Function::Set_Angle_Compensation(int walktime_n)
{
	Trajectory traj;

	RL_Compensation_Support_Leg_up = traj.Equation_solver(0, walktime_n * 0.075, 0, RL_Support_Leg);
	RL_Compensation_Support_Leg_down = traj.Equation_solver(0, walktime_n * 0.075, RL_Support_Leg, 0);
	RL_Compensation_Swing_Leg_up = traj.Equation_solver(0, walktime_n * 0.075, 0, RL_Swing_Leg);
	RL_Compensation_Swing_Leg_down = traj.Equation_solver(0, walktime_n * 0.075, RL_Swing_Leg, 0);
	RL_Compensation_Support_knee_up = traj.Equation_solver(0, walktime_n * 0.075, 0, RL_Support_Knee);
	RL_Compensation_Support_knee_down = traj.Equation_solver(0, walktime_n * 0.075, RL_Support_Knee, 0);
	RL_Compensation_Support_ankle_up = traj.Equation_solver(0, walktime_n * 0.075, 0, RL_Support_Ankle);
	RL_Compensation_Support_ankle_down = traj.Equation_solver(0, walktime_n * 0.075, RL_Support_Ankle, 0);

	LL_Compensation_Support_Leg_up = traj.Equation_solver(0, walktime_n * 0.075, 0, LL_Support_Leg);
	LL_Compensation_Support_Leg_down = traj.Equation_solver(0, walktime_n * 0.075, LL_Support_Leg, 0);
	LL_Compensation_Swing_Leg_up = traj.Equation_solver(0, walktime_n * 0.075, 0, LL_Swing_Leg);
	LL_Compensation_Swing_Leg_down = traj.Equation_solver(0, walktime_n * 0.075, LL_Swing_Leg, 0);
	LL_Compensation_Support_knee_up = traj.Equation_solver(0, walktime_n * 0.075, 0, LL_Support_Knee);
	LL_Compensation_Support_knee_down = traj.Equation_solver(0, walktime_n * 0.075, LL_Support_Knee, 0);
	LL_Compensation_Support_ankle_up = traj.Equation_solver(0, walktime_n * 0.075, 0, LL_Support_Ankle);
	LL_Compensation_Support_ankle_down = traj.Equation_solver(0, walktime_n * 0.075, LL_Support_Ankle, 0);
}

void IK_Function::Change_Angle_Compensation(double RL_Support, double RL_Swing, double RL_Knee, double RL_Ankle, double LL_Support, double LL_Swing, double LL_Knee, double LL_Ankle)
{
	RL_Support_Leg = RL_Support * deg2rad;
	RL_Swing_Leg = RL_Swing * deg2rad;
	RL_Support_Knee = RL_Knee * deg2rad; //-5 * deg2rad;
	RL_Support_Ankle = RL_Ankle * deg2rad;

	LL_Support_Leg = LL_Support * deg2rad;
	LL_Swing_Leg = LL_Swing * deg2rad;
	LL_Support_Knee = LL_Knee * deg2rad; //-5 * deg2rad;
	LL_Support_Ankle = LL_Ankle * deg2rad;
}

void IK_Function::Angle_Compensation(int indext, int size)
{

	double check_index = indext % walktime_n;
	double dwalktime = walktime_n;
	if (indext > 74 && indext < size + walktime_n)
	{
		if (check_index > 0.075 * dwalktime && check_index < 0.15 * dwalktime) // swing
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(check_index - 0.075 * dwalktime);
		}
		else if (check_index > 0.15 * dwalktime && check_index < 0.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;
		}
		else if (check_index > 0.35 * dwalktime && check_index < 0.425 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(check_index - 0.35 * dwalktime);
		}
		else if (check_index > 0.575 * dwalktime && check_index < 0.65 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(check_index - 0.575 * dwalktime);
		}
		else if (check_index > 0.65 * dwalktime && check_index < 0.85 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;
		}
		else if (check_index > 0.85 * dwalktime && check_index < 0.925 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(check_index - 0.85 * dwalktime);
		}

		if (check_index > 0.075 * dwalktime && check_index < 0.15 * dwalktime) // support
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(check_index - 0.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(check_index - 0.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(check_index - 0.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(check_index - 0.075 * dwalktime);
		}
		else if (check_index > 0.15 * dwalktime && check_index < 0.35 * dwalktime)
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (check_index > 0.35 * dwalktime && check_index < 0.425 * dwalktime)
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(check_index - 0.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(check_index - 0.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(check_index - 0.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(check_index - 0.35 * dwalktime);
		}

		else if (check_index > 0.575 * dwalktime && check_index < 0.65 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(check_index - 0.575 * dwalktime);
		}
		else if (check_index > 0.65 * dwalktime && check_index < 0.85 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (check_index > 0.85 * dwalktime && check_index < 0.925 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(check_index - 0.85 * dwalktime);
		}
	}
}

void IK_Function::Fast_Angle_Compensation(int indext)
{
	int fwalktime_n = walktime_n * 0.5;
	double dwalktime = fwalktime_n;
	double check_index = indext % fwalktime_n;

	if (indext > 0.5 * dwalktime && indext < sim_n + dwalktime)
	{
		if (check_index > 0.075 * dwalktime && check_index < 0.15 * dwalktime) // swing
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(check_index - 0.075 * dwalktime);
		}
		else if (check_index > 0.15 * dwalktime && check_index < 0.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;
		}
		else if (check_index > 0.35 * dwalktime && check_index < 0.425 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(check_index - 0.35 * dwalktime);
		}
		else if (check_index > 0.575 * dwalktime && check_index < 0.65 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(check_index - 0.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(check_index - 0.575 * dwalktime);
		}
		else if (check_index > 0.65 * dwalktime && check_index < 0.85 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;
		}
		else if (check_index > 0.85 * dwalktime && check_index < 0.925 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(check_index - 0.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(check_index - 0.85 * dwalktime);
		}

		if (check_index > 0.075 * dwalktime && check_index < 0.15 * dwalktime) // support
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(check_index - 0.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(check_index - 0.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(check_index - 0.075 * dwalktime);
		}
		else if (check_index > 0.15 * dwalktime && check_index < 0.35 * dwalktime)
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (check_index > 0.35 * dwalktime && check_index < 0.425 * dwalktime)
		{
			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(check_index - 0.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(check_index - 0.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(check_index - 0.35 * dwalktime);
		}

		else if (check_index > 0.575 * dwalktime && check_index < 0.65 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(check_index - 0.575 * dwalktime);
		}
		else if (check_index > 0.65 * dwalktime && check_index < 0.85 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (check_index > 0.85 * dwalktime && check_index < 0.925 * dwalktime) // swing
		{
			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(check_index - 0.85 * dwalktime);
		}
	}
}

double IK_Function::RL_Swing_Leg_Compensation_up(double t)
{
	double X = RL_Compensation_Swing_Leg_up(0) + RL_Compensation_Swing_Leg_up(1) * t + RL_Compensation_Swing_Leg_up(2) * pow(t, 2) + RL_Compensation_Swing_Leg_up(3) * pow(t, 3) + RL_Compensation_Swing_Leg_up(4) * pow(t, 4) + RL_Compensation_Swing_Leg_up(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Leg_Compensation_up(double t)
{
	double X = RL_Compensation_Support_Leg_up(0) + RL_Compensation_Support_Leg_up(1) * t + RL_Compensation_Support_Leg_up(2) * pow(t, 2) + RL_Compensation_Support_Leg_up(3) * pow(t, 3) + RL_Compensation_Support_Leg_up(4) * pow(t, 4) + RL_Compensation_Support_Leg_up(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Swing_Leg_Compensation_down(double t)
{
	double X = RL_Compensation_Swing_Leg_down(0) + RL_Compensation_Swing_Leg_down(1) * t + RL_Compensation_Swing_Leg_down(2) * pow(t, 2) + RL_Compensation_Swing_Leg_down(3) * pow(t, 3) + RL_Compensation_Swing_Leg_down(4) * pow(t, 4) + RL_Compensation_Swing_Leg_down(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Leg_Compensation_down(double t)
{
	double X = RL_Compensation_Support_Leg_down(0) + RL_Compensation_Support_Leg_down(1) * t + RL_Compensation_Support_Leg_down(2) * pow(t, 2) + RL_Compensation_Support_Leg_down(3) * pow(t, 3) + RL_Compensation_Support_Leg_down(4) * pow(t, 4) + RL_Compensation_Support_Leg_down(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Knee_Compensation_up(double t)
{
	double X = RL_Compensation_Support_knee_up(0) + RL_Compensation_Support_knee_up(1) * t + RL_Compensation_Support_knee_up(2) * pow(t, 2) + RL_Compensation_Support_knee_up(3) * pow(t, 3) + RL_Compensation_Support_knee_up(4) * pow(t, 4) + RL_Compensation_Support_knee_up(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Knee_Compensation_down(double t)
{
	double X = RL_Compensation_Support_knee_down(0) + RL_Compensation_Support_knee_down(1) * t + RL_Compensation_Support_knee_down(2) * pow(t, 2) + RL_Compensation_Support_knee_down(3) * pow(t, 3) + RL_Compensation_Support_knee_down(4) * pow(t, 4) + RL_Compensation_Support_knee_down(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Ankle_Compensation_up(double t)
{
	double X = RL_Compensation_Support_ankle_up(0) + RL_Compensation_Support_ankle_up(1) * t + RL_Compensation_Support_ankle_up(2) * pow(t, 2) + RL_Compensation_Support_ankle_up(3) * pow(t, 3) + RL_Compensation_Support_ankle_up(4) * pow(t, 4) + RL_Compensation_Support_ankle_up(5) * pow(t, 5);
	return X;
};
double IK_Function::RL_Support_Ankle_Compensation_down(double t)
{
	double X = RL_Compensation_Support_ankle_down(0) + RL_Compensation_Support_ankle_down(1) * t + RL_Compensation_Support_ankle_down(2) * pow(t, 2) + RL_Compensation_Support_ankle_down(3) * pow(t, 3) + RL_Compensation_Support_ankle_down(4) * pow(t, 4) + RL_Compensation_Support_ankle_down(5) * pow(t, 5);
	return X;
};

double IK_Function::LL_Swing_Leg_Compensation_up(double t)
{
	double X = LL_Compensation_Swing_Leg_up(0) + LL_Compensation_Swing_Leg_up(1) * t + LL_Compensation_Swing_Leg_up(2) * pow(t, 2) + LL_Compensation_Swing_Leg_up(3) * pow(t, 3) + LL_Compensation_Swing_Leg_up(4) * pow(t, 4) + LL_Compensation_Swing_Leg_up(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Leg_Compensation_up(double t)
{
	double X = LL_Compensation_Support_Leg_up(0) + LL_Compensation_Support_Leg_up(1) * t + LL_Compensation_Support_Leg_up(2) * pow(t, 2) + LL_Compensation_Support_Leg_up(3) * pow(t, 3) + LL_Compensation_Support_Leg_up(4) * pow(t, 4) + LL_Compensation_Support_Leg_up(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Swing_Leg_Compensation_down(double t)
{
	double X = LL_Compensation_Swing_Leg_down(0) + LL_Compensation_Swing_Leg_down(1) * t + LL_Compensation_Swing_Leg_down(2) * pow(t, 2) + LL_Compensation_Swing_Leg_down(3) * pow(t, 3) + LL_Compensation_Swing_Leg_down(4) * pow(t, 4) + LL_Compensation_Swing_Leg_down(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Leg_Compensation_down(double t)
{
	double X = LL_Compensation_Support_Leg_down(0) + LL_Compensation_Support_Leg_down(1) * t + LL_Compensation_Support_Leg_down(2) * pow(t, 2) + LL_Compensation_Support_Leg_down(3) * pow(t, 3) + LL_Compensation_Support_Leg_down(4) * pow(t, 4) + LL_Compensation_Support_Leg_down(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Knee_Compensation_up(double t)
{
	double X = LL_Compensation_Support_knee_up(0) + LL_Compensation_Support_knee_up(1) * t + LL_Compensation_Support_knee_up(2) * pow(t, 2) + LL_Compensation_Support_knee_up(3) * pow(t, 3) + LL_Compensation_Support_knee_up(4) * pow(t, 4) + LL_Compensation_Support_knee_up(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Knee_Compensation_down(double t)
{
	double X = LL_Compensation_Support_knee_down(0) + LL_Compensation_Support_knee_down(1) * t + LL_Compensation_Support_knee_down(2) * pow(t, 2) + LL_Compensation_Support_knee_down(3) * pow(t, 3) + LL_Compensation_Support_knee_down(4) * pow(t, 4) + LL_Compensation_Support_knee_down(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Ankle_Compensation_up(double t)
{
	double X = LL_Compensation_Support_ankle_up(0) + LL_Compensation_Support_ankle_up(1) * t + LL_Compensation_Support_ankle_up(2) * pow(t, 2) + LL_Compensation_Support_ankle_up(3) * pow(t, 3) + LL_Compensation_Support_ankle_up(4) * pow(t, 4) + LL_Compensation_Support_ankle_up(5) * pow(t, 5);
	return X;
};
double IK_Function::LL_Support_Ankle_Compensation_down(double t)
{
	double X = LL_Compensation_Support_ankle_down(0) + LL_Compensation_Support_ankle_down(1) * t + LL_Compensation_Support_ankle_down(2) * pow(t, 2) + LL_Compensation_Support_ankle_down(3) * pow(t, 3) + LL_Compensation_Support_ankle_down(4) * pow(t, 4) + LL_Compensation_Support_ankle_down(5) * pow(t, 5);
	return X;
};

void IK_Function::Angle_Compensation_Leftwalk(int indext)
{
	double dwalktime = walktime_n;
	if (1.575 * dwalktime < indext && indext < 1.65 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 1.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 1.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 1.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 1.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 1.575 * dwalktime);
	}
	else if (1.65 * dwalktime < indext && indext < 1.85 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (1.85 * dwalktime < indext && indext < 1.925 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 1.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 1.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 1.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 1.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 1.85 * dwalktime);
	}

	else if (2.075 * dwalktime < indext && indext < 2.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 2.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 2.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 2.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 2.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 2.075 * dwalktime);
	}
	else if (2.15 * dwalktime < indext && indext < 2.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (2.35 * dwalktime < indext && indext < 2.425 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 2.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 2.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 2.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 2.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 2.35 * dwalktime);
	}
	
	else if (2.575 * dwalktime < indext && indext < 2.65 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 2.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 2.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 2.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 2.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 2.575 * dwalktime);
	}
	else if (2.65 * dwalktime < indext && indext < 2.85 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (2.85 * dwalktime < indext && indext < 2.925 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 2.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 2.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 2.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 2.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 2.85 * dwalktime);
	}
	
	else if (3.075 * dwalktime < indext && indext < 3.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 3.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 3.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 3.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 3.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 3.075 * dwalktime);
	}
	else if (3.15 * dwalktime < indext && indext < 3.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (3.35 * dwalktime < indext && indext < 3.425 * walktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 3.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 3.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 3.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 3.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 3.35 * dwalktime);
	}

	else if (3.575 * dwalktime < indext && indext < 3.65 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 3.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 3.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 3.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 3.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 3.575 * dwalktime);
	}
	else if (3.65 * dwalktime < indext && indext < 3.85 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (3.85 * dwalktime < indext && indext < 3.925 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 3.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 3.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 3.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 3.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 3.85 * dwalktime);
	}

	else if (4.075 * dwalktime < indext && indext < 4.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 4.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 4.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 4.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 4.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 4.075 * dwalktime);
	}
	else if (4.15 * dwalktime < indext && indext < 4.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (4.35 * dwalktime < indext && indext < 4.425 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 4.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 4.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 4.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 4.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 4.35 * dwalktime);
	}
	
	else if (4.575 * dwalktime < indext && indext < 4.65 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 4.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 4.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 4.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 4.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 4.575 * dwalktime);
	}
	else if (4.65 * dwalktime < indext && indext < 4.85 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (4.85 * dwalktime < indext && indext < 4.925 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 4.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 4.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 4.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 4.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 4.85 * dwalktime);
	}
	
	else if (5.075 * dwalktime < indext && indext < 5.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 5.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 5.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 5.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 5.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 5.075 * dwalktime);
	}
	else if (5.15 * dwalktime < indext && indext < 5.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (5.35 * dwalktime < indext && indext < 5.425 * walktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 5.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 5.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 5.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 5.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 5.35 * dwalktime);
	}

	else if (6.575 * dwalktime < indext && indext < 6.65 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 6.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 6.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 6.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 6.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 6.575 * dwalktime);
	}
	else if (6.65 * dwalktime < indext && indext < 6.85 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (6.85 * dwalktime < indext && indext < 6.925 * dwalktime) // support
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 6.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 6.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 6.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 6.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 6.85 * dwalktime);
	}

	else if (7.075 * dwalktime < indext && indext < 7.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 7.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 7.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 7.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 7.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 7.075 * dwalktime);
	}
	else if (7.15 * dwalktime < indext && indext < 7.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (7.35 * dwalktime < indext && indext < 7.425 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 7.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 7.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 7.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 7.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 7.35 * dwalktime);
	}
	
	else if (7.575 * dwalktime < indext && indext < 7.65 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 7.575 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 7.575 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 7.575 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 7.575 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 7.575 * dwalktime);
	}
	else if (7.65 * dwalktime < indext && indext < 7.85 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;

		LL_th[1] = LL_th[1] + LL_Swing_Leg;
	}
	else if (7.85 * dwalktime < indext && indext < 7.925 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 7.85 * dwalktime);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 7.85 * dwalktime);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 7.85 * dwalktime);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 7.85 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 7.85 * dwalktime);
	}
	
	else if (8.075 * dwalktime < indext && indext < 8.15 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 8.075 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 8.075 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 8.075 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 8.075 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 8.075 * dwalktime);
	}
	else if (8.15 * dwalktime < indext && indext < 8.35 * dwalktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg;

		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;
	}
	else if (8.35 * dwalktime < indext && indext < 8.425 * walktime)
	{
		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 8.35 * dwalktime);

		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 8.35 * dwalktime);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 8.35 * dwalktime);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 8.35 * dwalktime);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 8.35 * dwalktime);
	}

}

void IK_Function::Angle_Compensation_Rightwalk(int indext)
{
	double dwalktime = walktime_n;
	if (indext > 0.5 * walktime_n && indext < sim_n - 0.5*dwalktime)
	{
		if (1.575 * dwalktime < indext && indext < 1.65 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 1.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 1.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 1.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 1.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 1.575 * dwalktime);
		}
		else if (1.65 * dwalktime < indext && indext < 1.85 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (1.85 * dwalktime < indext && indext < 1.925 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 1.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 1.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 1.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 1.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 1.85 * dwalktime);
		}

		else if (2.075 * dwalktime < indext && indext < 2.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 2.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 2.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 2.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 2.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 2.075 * dwalktime);
		}
		else if (2.15 * dwalktime < indext && indext < 2.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (2.35 * dwalktime < indext && indext < 2.425 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 2.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 2.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 2.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 2.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 2.35 * dwalktime);
		}
		
		else if (2.575 * dwalktime < indext && indext < 2.65 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 2.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 2.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 2.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 2.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 2.575 * dwalktime);
		}
		else if (2.65 * dwalktime < indext && indext < 2.85 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (2.85 * dwalktime < indext && indext < 2.925 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 2.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 2.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 2.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 2.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 2.85 * dwalktime);
		}
		
		else if (3.075 * dwalktime < indext && indext < 3.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 3.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 3.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 3.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 3.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 3.075 * dwalktime);
		}
		else if (3.15 * dwalktime < indext && indext < 3.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (3.35 * dwalktime < indext && indext < 3.425 * walktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 3.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 3.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 3.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 3.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 3.35 * dwalktime);
		}

		else if (3.575 * dwalktime < indext && indext < 3.65 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 3.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 3.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 3.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 3.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 3.575 * dwalktime);
		}
		else if (3.65 * dwalktime < indext && indext < 3.85 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (3.85 * dwalktime < indext && indext < 3.925 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 3.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 3.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 3.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 3.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 3.85 * dwalktime);
		}

		else if (4.075 * dwalktime < indext && indext < 4.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 4.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 4.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 4.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 4.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 4.075 * dwalktime);
		}
		else if (4.15 * dwalktime < indext && indext < 4.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (4.35 * dwalktime < indext && indext < 4.425 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 4.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 4.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 4.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 4.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 4.35 * dwalktime);
		}
		
		else if (4.575 * dwalktime < indext && indext < 4.65 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 4.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 4.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 4.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 4.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 4.575 * dwalktime);
		}
		else if (4.65 * dwalktime < indext && indext < 4.85 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (4.85 * dwalktime < indext && indext < 4.925 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 4.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 4.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 4.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 4.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 4.85 * dwalktime);
		}
		
		else if (5.075 * dwalktime < indext && indext < 5.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 5.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 5.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 5.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 5.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 5.075 * dwalktime);
		}
		else if (5.15 * dwalktime < indext && indext < 5.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (5.35 * dwalktime < indext && indext < 5.425 * walktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 5.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 5.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 5.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 5.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 5.35 * dwalktime);
		}

		else if (5.575 * dwalktime < indext && indext < 5.65 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 5.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 5.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 5.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 5.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 5.575 * dwalktime);
		}
		else if (5.65 * dwalktime < indext && indext < 5.85 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (5.85 * dwalktime < indext && indext < 5.925 * dwalktime) // support
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 5.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 5.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 5.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 5.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 5.85 * dwalktime);
		}

		else if (6.075 * dwalktime < indext && indext < 6.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 6.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 6.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 6.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 6.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 6.075 * dwalktime);
		}
		else if (6.15 * dwalktime < indext && indext < 6.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (6.35 * dwalktime < indext && indext < 6.425 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 6.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 6.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 6.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 6.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 6.35 * dwalktime);
		}
		
		else if (6.575 * dwalktime < indext && indext < 6.65 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 6.575 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 6.575 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 6.575 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 6.575 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 6.575 * dwalktime);
		}
		else if (6.65 * dwalktime < indext && indext < 6.85 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg;
			LL_th[3] = LL_th[3] + LL_Support_Knee;
			LL_th[4] = LL_th[4] + LL_Support_Ankle;
			LL_th[5] = LL_th[5] + LL_Support_Leg;

			RL_th[1] = RL_th[1] - RL_Swing_Leg;
		}
		else if (6.85 * dwalktime < indext && indext < 6.925 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 6.85 * dwalktime);
			LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 6.85 * dwalktime);
			LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 6.85 * dwalktime);
			LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 6.85 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 6.85 * dwalktime);
		}
		
		else if (7.075 * dwalktime < indext && indext < 7.15 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 7.075 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 7.075 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 7.075 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 7.075 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 7.075 * dwalktime);
		}
		else if (7.15 * dwalktime < indext && indext < 7.35 * dwalktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg;

			RL_th[1] = RL_th[1] - RL_Support_Leg;
			RL_th[3] = RL_th[3] - RL_Support_Knee;
			RL_th[4] = RL_th[4] - RL_Support_Ankle;
			RL_th[5] = RL_th[5] - RL_Support_Leg;
		}
		else if (7.35 * dwalktime < indext && indext < 7.425 * walktime)
		{
			LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 7.35 * dwalktime);

			RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 7.35 * dwalktime);
			RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 7.35 * dwalktime);
			RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 7.35 * dwalktime);
			RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 7.35 * dwalktime);
		}

	
	
	}
}

void IK_Function::Angle_Compensation_Huddle(int indext)
{
	double dwalktime = walktime_n;
	if (indext > 0.8 * dwalktime + 100 && indext < 0.875 * dwalktime + 100) // swing
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 0.8 * dwalktime - 100);
		LL_th[5] = LL_th[5] - LL_Swing_Leg_Compensation_up(indext - 0.8 * dwalktime - 100);

		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 0.8 * dwalktime - 100);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 0.8 * dwalktime - 100);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 0.8 * dwalktime - 100);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 0.8 * dwalktime - 100);
	}
	else if (indext > 0.875 * dwalktime + 100 && indext < 1.625 * dwalktime + 100)
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg;
		LL_th[5] = LL_th[5] - LL_Swing_Leg;

		RL_th[1] = RL_th[1] - RL_Support_Leg;
		RL_th[3] = RL_th[3] - RL_Support_Knee;
		RL_th[4] = RL_th[4] - RL_Support_Ankle;
		RL_th[5] = RL_th[5] - RL_Support_Leg;
	}
	else if (indext > 1.625 * dwalktime + 100 && indext < 1.7 * dwalktime + 100)
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 1.625 * dwalktime - 100);
		LL_th[5] = LL_th[5] - LL_Swing_Leg_Compensation_down(indext - 1.625 * dwalktime - 100);

		RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 1.625 * dwalktime - 100);
		RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 1.625 * dwalktime - 100);
		RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 1.625 * dwalktime - 100);
		RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 1.625 * dwalktime - 100);
	}
	else if (indext > 2.3 * dwalktime + 100 && indext < 2.375 * dwalktime + 100) // support
	{
		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_up(indext - 2.3 * dwalktime - 100);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_up(indext - 2.3 * dwalktime - 100);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_up(indext - 2.3 * dwalktime - 100);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_up(indext - 2.3 * dwalktime - 100);

		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_up(indext - 2.3 * dwalktime - 100);
		// RL_th[5] = RL_th[5] + RL_Swing_Leg_Compensation_up(indext - 2.3 * dwalktime - 100);
	}
	else if (indext > 2.375 * dwalktime + 100 && indext < 3.625 * dwalktime + 100) // support
	{
		LL_th[1] = LL_th[1] + LL_Support_Leg;
		LL_th[3] = LL_th[3] + LL_Support_Knee;
		LL_th[4] = LL_th[4] + LL_Support_Ankle;
		LL_th[5] = LL_th[5] + LL_Support_Leg;

		RL_th[1] = RL_th[1] - RL_Swing_Leg;
		// RL_th[5] = RL_th[5] + RL_Swing_Leg;
	}
	else if (indext > 3.625 * dwalktime + 100 && indext < 3.7 * dwalktime + 100) // support
	{
		LL_th[1] = LL_th[1] + LL_Support_Leg_Compensation_down(indext - 3.625 * dwalktime - 100);
		LL_th[3] = LL_th[3] + LL_Support_Knee_Compensation_down(indext - 3.625 * dwalktime - 100);
		LL_th[4] = LL_th[4] + LL_Support_Ankle_Compensation_down(indext - 3.625 * dwalktime - 100);
		LL_th[5] = LL_th[5] + LL_Support_Leg_Compensation_down(indext - 3.625 * dwalktime - 100);

		RL_th[1] = RL_th[1] - RL_Swing_Leg_Compensation_down(indext - 3.625 * dwalktime - 100);
		// RL_th[5] = RL_th[5] + RL_Swing_Leg_Compensation_down(indext - 3.625 * dwalktime - 100);
	}

	else if (indext > 5 * dwalktime + 100 && indext < 5.075 * dwalktime + 100) // support
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_up(indext - 5 * dwalktime - 100);
		// LL_th[5] = LL_th[5] - LL_Swing_Leg_Compensation_up(indext - 4 * dwalktime - 100);

		// RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_up(indext - 4 * dwalktime - 100);
		// RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_up(indext - 4 * dwalktime - 100);
		// RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_up(indext - 34 * dwalktime - 100);
		// RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_up(indext - 4 * dwalktime - 100);
	}
	else if (indext > 5.075 * dwalktime + 100 && indext < 5.625 * dwalktime + 100)
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg;
		// LL_th[5] = LL_th[5] - LL_Swing_Leg;

		// RL_th[1] = RL_th[1] - RL_Support_Leg;
		// RL_th[3] = RL_th[3] - RL_Support_Knee;
		// RL_th[4] = RL_th[4] - RL_Support_Ankle;
		// RL_th[5] = RL_th[5] - RL_Support_Leg;
	}
	else if (indext > 5.625 * dwalktime + 100 && indext < 5.7 * dwalktime + 100)
	{
		LL_th[1] = LL_th[1] + LL_Swing_Leg_Compensation_down(indext - 5.625 * dwalktime - 100);
		// LL_th[5] = LL_th[5] - LL_Swing_Leg_Compensation_down(indext - 4.625 * dwalktime - 100);

		// RL_th[1] = RL_th[1] - RL_Support_Leg_Compensation_down(indext - 4.625 * dwalktime - 100);
		// RL_th[3] = RL_th[3] - RL_Support_Knee_Compensation_down(indext - 4.625 * dwalktime - 100);
		// RL_th[4] = RL_th[4] - RL_Support_Ankle_Compensation_down(indext - 4.625 * dwalktime - 100);
		// RL_th[5] = RL_th[5] - RL_Support_Leg_Compensation_down(indext - 4.625 * dwalktime - 100);
	}
}