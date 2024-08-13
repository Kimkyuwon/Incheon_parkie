// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <math.h>
// #include <ceres/ceres.h>
// #include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <common.h>

struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}

	template <typename T>
    bool operator()(const T *t, const T *q, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		//Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

    /*
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
                LidarEdgeFactor, 3, 3, 4>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	} */

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
	double s;
};

struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
    bool operator()(const T *t, const T *q, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

        residual[0] = (lp - lpj).dot(ljm);

		return true;
	}

    /*
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		return (new ceres::AutoDiffCostFunction<
                LidarPlaneFactor, 1, 3, 4>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	} */

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarEdgePDFactor
{
    LidarEdgePDFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d edge_vec_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_, Eigen::Vector3d last_point_c_, double s_)
        : curr_point(curr_point_), edge_vec(edge_vec_), last_point_a(last_point_a_), last_point_b(last_point_b_), last_point_c(last_point_c_), s(s_) {}

    template <typename T>
    bool operator()(const T *t, const T *q, T *residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};
        Eigen::Matrix<T, 3, 1> lpc{T(last_point_c.x()), T(last_point_c.y()), T(last_point_c.z())};
        Eigen::Matrix<T, 3, 1> ev{T(edge_vec.x()), T(edge_vec.y()), T(edge_vec.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        q_last_curr = q_identity.slerp(T(s), q_last_curr);
        Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;
        Eigen::Matrix<T, 3, 1> lev;
        lev = q_last_curr * ev;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();
        residual[3] = acos((lev).dot(lpc) / (lev.norm() * lpc.norm()));

        return true;
    }

    /*
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d edge_vec_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const Eigen::Vector3d last_point_c_, const double s_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarEdgePDFactor, 4, 3, 4>(
            new LidarEdgePDFactor(curr_point_, edge_vec_, last_point_a_, last_point_b_, last_point_c_, s_)));
    } */

    Eigen::Vector3d curr_point, edge_vec, last_point_a, last_point_b, last_point_c;
    double s;
};

struct LidarPlanePDFactor
{
    LidarPlanePDFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d normal_vec_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, Eigen::Vector3d last_point_n_, double s_)
        : curr_point(curr_point_), normal_vec(normal_vec_), last_point_j(last_point_j_), last_point_l(last_point_l_),
          last_point_m(last_point_m_), last_point_n(last_point_n_), s(s_)
    {
        ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        ljm_norm.normalize();
    }

    template <typename T>
    bool operator()(const T *t, const T *q, T *residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};
        Eigen::Matrix<T, 3, 1> nv{T(normal_vec.x()), T(normal_vec.y()), T(normal_vec.z())};
        Eigen::Matrix<T, 3, 1> lpn{T(last_point_n.x()), T(last_point_n.y()), T(last_point_n.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        q_last_curr = q_identity.slerp(T(s), q_last_curr);
        Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;
        Eigen::Matrix<T, 3, 1> lnv;
        lnv = q_last_curr * nv;

        residual[0] = (lp - lpj).dot(ljm);
        residual[1] = acos((lnv).dot(lpn) / (lnv.norm() * lpn.norm()));

        return true;
    }

    /*
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, Eigen::Vector3d normal_vec_, const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_, Eigen::Vector3d last_point_n_,
                                       const double s_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarPlanePDFactor, 2, 3, 4>(
            new LidarPlanePDFactor(curr_point_, normal_vec_, last_point_j_, last_point_l_, last_point_m_, last_point_n_, s_)));
    } */

    Eigen::Vector3d curr_point, normal_vec, last_point_j, last_point_l, last_point_m, last_point_n;
    Eigen::Vector3d ljm_norm;
    double s;
};
