#ifndef _A_LOAM_LIDAR_FACTOR_HPP
#define _A_LOAM_LIDAR_FACTOR_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

namespace lidarslam{
// 地面点残差计算
struct GroundPlaneFactor{
    GroundPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d point_j_,
                     Eigen::Vector3d point_l_, Eigen::Vector3d point_m_)
        : curr_point(curr_point_), point_j(point_j_), 
          point_l(point_l_), point_m(point_m_)
    {
        ljm_norm = (point_j - point_l).cross(point_j - point_m);
        ljm_norm.normalize();
    }

    template<typename T>
    bool operator()(const T* q, const T* t, T* residual) const{
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(point_j.x()), T(point_j.y()), T(point_j.z())};
        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

        T cx = ceres::cos(q[0]);
        T sx = ceres::sin(q[0]);
        T cy = ceres::cos(q[1]);
        T sy = ceres::sin(q[1]);
        // T cz = ceres::cos(q[2]);
        // T sz = ceres::sin(q[2]);
        Eigen::Matrix<T, 3, 3> qx;
        qx << T(1.0), T(0.0), T(0.0),
              T(0.0), cx, -sx,
              T(0.0), sx, cx;
        Eigen::Matrix<T, 3, 3> qy;
        qy << cy, T(0.0), sy,
              T(0.0), T(1.0), T(0.0),
              -sy, T(0.0), cy;
        // Eigen::Matrix<T, 3, 3> qz;
        // qz << cz, -sz, T(0.0),
        //       sz, cz, T(0.0),
        //       T(0.0), T(0.0), T(1.0);
    
        

        // Eigen::Matrix<T, 3, 1> txyz{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> txyz{T(0), T(0), t[0]};

        // Eigen::Matrix<T, 3, 1> lp = qz * qy * qx * cp + txyz;
        Eigen::Matrix<T, 3, 1> lp = qy * qx * cp + txyz;
        
        residual[0] = (lp - lpj).dot(ljm);
        return true;
    }

    static ceres::CostFunction *Create(Eigen::Vector3d curr_point_, Eigen::Vector3d point_j_,
                     Eigen::Vector3d point_l_, Eigen::Vector3d point_m_){
        return (new ceres::AutoDiffCostFunction<GroundPlaneFactor, 1, 2, 1>
                (new GroundPlaneFactor(curr_point_, point_j_, point_l_, point_m_)));
    }
    Eigen::Vector3d curr_point, point_j, point_l, point_m;
    Eigen::Vector3d ljm_norm;
};

// 边缘点残差计算
struct CornerFactor{
    CornerFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d point_a_,
                    Eigen::Vector3d point_b_, Eigen::Matrix<double, 3, 3> qyx_, double z_)
        : _curr_point(curr_point_), _point_a(point_a_), _point_b(point_b_), _qyx(qyx_), _z(z_){}
    // qz, tx, ty
    template<typename T>
    bool operator()(const T* q, const T* t, T* residual) const {
        T cz = ceres::cos(q[0]);
        T sz = ceres::sin(q[0]);
        Eigen::Matrix<T, 3, 3> qz;
        qz << cz, -sz, T(0.0),
              sz, cz, T(0.0),
              T(0.0), T(0.0), T(1.0);
        Eigen::Matrix<T, 3, 3> qyx;
        qyx << T(_qyx(0, 0)), T(_qyx(0, 1)), T(_qyx(0, 2)), 
               T(_qyx(1, 0)), T(_qyx(1, 1)), T(_qyx(1, 2)), 
               T(_qyx(2, 0)), T(_qyx(2, 1)), T(_qyx(2, 2));
        Eigen::Matrix<T, 3, 1> txyz{t[0], t[1], T(_z)};

        Eigen::Matrix<T, 3, 1> cp{T(_curr_point.x()), T(_curr_point.y()), T(_curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(_point_a.x()), T(_point_a.y()), T(_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(_point_b.x()), T(_point_b.y()), T(_point_b.z())};

        Eigen::Matrix<T, 3, 1> lp = qz * qyx * cp + txyz;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        return true;
    }

    static ceres::CostFunction *Create(Eigen::Vector3d curr_point_, Eigen::Vector3d point_a_,
                    Eigen::Vector3d point_b_, Eigen::Matrix<double, 3, 3> qyx_, double z_){
        return (new ceres::AutoDiffCostFunction<CornerFactor, 3, 1, 2>
            (new CornerFactor(curr_point_, point_a_, point_b_, qyx_, z_)));
    }

    Eigen::Vector3d _curr_point;
    Eigen::Vector3d _point_a;
    Eigen::Vector3d _point_b;
    Eigen::Matrix<double, 3, 3> _qyx;
    double _z;
};

// 后端残差
struct PlaneFactorBack{
    PlaneFactorBack(double _pa, double _pb, double _pc, double _pd, PointType _point)
        : pa(_pa), pb(_pb), pc(_pc), pd(_pd), point(_point)
    {
        
    }

    template<typename T>
    bool operator()(const T* q, const T* t, T* residual) const{
        T cx = ceres::cos(q[0]);
        T sx = ceres::sin(q[0]);
        T cy = ceres::cos(q[1]);
        T sy = ceres::sin(q[1]);
        T cz = ceres::cos(q[2]);
        T sz = ceres::sin(q[2]);
        Eigen::Matrix<T, 3, 3> qx;
        qx << T(1.0), T(0.0), T(0.0),
              T(0.0), cx, -sx,
              T(0.0), sx, cx;
        Eigen::Matrix<T, 3, 3> qy;
        qy << cy, T(0.0), sy,
              T(0.0), T(1.0), T(0.0),
              -sy, T(0.0), cy;
        Eigen::Matrix<T, 3, 3> qz;
        qz << cz, -sz, T(0.0),
              sz, cz, T(0.0),
              T(0.0), T(0.0), T(1.0);
    
        Eigen::Matrix<T, 3, 1> txyz{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> cp{T(point.x), T(point.y), T(point.z)};

        Eigen::Matrix<T, 3, 1> lp = qz * qy * qx * cp + txyz;

        residual[0] = (Eigen::Matrix<T, 1, 3>(T(pa), T(pb), T(pc))).dot(lp) + T(pd);
        residual[0] = residual[0] < 0 ? -residual[0] : residual[0];
        return true;
    }

    static ceres::CostFunction *Create(double _pa, double _pb, double _pc, double _pd, PointType _point){
        return (new ceres::AutoDiffCostFunction<PlaneFactorBack, 1, 3, 3>
            (new PlaneFactorBack(_pa, _pb, _pc, _pd, _point)));
    }
    
    double pa;
    double pb;
    double pc;
    double pd;
    PointType point;
};

// 后端边缘点残差
struct EdgeFatorBack{
    EdgeFatorBack(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

		T cx = ceres::cos(q[0]);
        T sx = ceres::sin(q[0]);
        T cy = ceres::cos(q[1]);
        T sy = ceres::sin(q[1]);
        T cz = ceres::cos(q[2]);
        T sz = ceres::sin(q[2]);
        Eigen::Matrix<T, 3, 3> qx;
        qx << T(1.0), T(0.0), T(0.0),
              T(0.0), cx, -sx,
              T(0.0), sx, cx;
        Eigen::Matrix<T, 3, 3> qy;
        qy << cy, T(0.0), sy,
              T(0.0), T(1.0), T(0.0),
              -sy, T(0.0), cy;
        Eigen::Matrix<T, 3, 3> qz;
        qz << cz, -sz, T(0.0),
              sz, cz, T(0.0),
              T(0.0), T(0.0), T(1.0);

        // 平移矩阵
		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};
        // 变换后的点坐标
		Eigen::Matrix<T, 3, 1> lp = qz * qy * qx * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_)
	{
		return (new ceres::AutoDiffCostFunction<
				EdgeFatorBack, 3, 3, 3>(
			new EdgeFatorBack(curr_point_, last_point_a_, last_point_b_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
};

// 四元数方式
struct LidarEdgeFactor{
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d point_a_, Eigen::Vector3d point_b_)
        : curr_point(curr_point_), point_a(point_a_), point_b(point_b_){}

    template<typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(point_a.x()), T(point_a.y()), T(point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(point_b.x()), T(point_b.y()), T(point_b.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp = q_last_curr * cp + t_last_curr;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d point_a_,
                                        const Eigen::Vector3d point_b_){
        return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>
            (new LidarEdgeFactor(curr_point_, point_a_, point_b_)));
    }


    Eigen::Vector3d curr_point;
    Eigen::Vector3d point_a;
    Eigen::Vector3d point_b;
};
struct LidarPlaneFactor{
    LidarPlaneFactor(double _pa, double _pb, double _pc, double _pd, PointType _point)
        : pa(_pa), pb(_pb), pc(_pc), pd(_pd), point(_point)
    {
        
    }

    template<typename T>
    bool operator()(const T* q, const T* t, T* residual) const{
        Eigen::Quaternion<T> q_last{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> cp{T(point.x), T(point.y), T(point.z)};
        Eigen::Matrix<T, 3, 1> lp = q_last * cp + t_last;

        residual[0] = (Eigen::Matrix<T, 1, 3>(T(pa), T(pb), T(pc))).dot(lp) + T(pd);
        residual[0] = residual[0] < 0 ? -residual[0] : residual[0];
        return true;
    }

    static ceres::CostFunction *Create(double _pa, double _pb, double _pc, double _pd, PointType _point){
        return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>
            (new LidarPlaneFactor(_pa, _pb, _pc, _pd, _point)));
    }
    
    double pa, pb, pc, pd;
    PointType point;
};
}

#endif