#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include  <pybind11/chrono.h>
#include "modern_robotics.h"
#include "MR_Indy7.h"
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)
using namespace mr;
namespace py = pybind11;

PYBIND11_MODULE(modern_robotics, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
  m.def("se3ToVec", [](const se3& T) {
        return mr::se3ToVec(T);
    });
  m.def("ad", [](const Vector6d& V) {
        return mr::ad(V);
    });
   m.def("VecTose3", [](const Vector6d& V) {
        return VecTose3(V);
    });
   m.def("VecToso3", [](const Vector3d& omg) {
        return VecToso3(omg);
    });
   m.def("so3ToVec", [](const so3& so3mat) {
        return so3ToVec(so3mat);
    });
   m.def("AxisAng3", [](const Vector3d& expc3) {
        return AxisAng3(expc3);
    });

   m.def("NearZero", [](const double val) {
        return NearZero(val);
    });

   m.def("MatrixExp3", [](const so3& so3mat) {
        return MatrixExp3(so3mat);
    });

   m.def("MatrixExp6", [](const se3& se3mat) {
        return MatrixExp6(se3mat);
    });
   m.def("Normalize", [](Vector3d V) {
        return Normalize(V);
    });
   m.def("FKinSpace", [](const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
        return FKinSpace(M,Slist,thetaList);
    });
   m.def("Adjoint", [](const SE3& T) {
        return Adjoint(T);
    });
   m.def("pinvJacobianBody", [](const ScrewList& Blist, const JVec& thetaList) {
        return pinvJacobianBody(Blist,thetaList);
    });
   m.def("TransToP", [](const SE3& T) {
        return TransToP(T);
    });
   m.def("TransToR", [](const SE3& T) {
        return TransToR(T);
    });    
   m.def("JacobianSpace", [](const ScrewList& Slist, const JVec& thetaList) {
        return JacobianSpace(Slist,thetaList);
    });    
   m.def("pinvJacobianBody", [](const ScrewList& Blist, const JVec& thetaList) {
        return pinvJacobianBody(Blist,thetaList);
    });    
   m.def("JacobianBody", [](const ScrewList& Blist, const JVec& thetaList) {
        return JacobianBody(Blist,thetaList);
    });    
   m.def("AnalyticJacobianBody", [](SE3 M,const ScrewList& Blist, const JVec& thetaList) {
        return AnalyticJacobianBody(M,Blist,thetaList);
    });    
   m.def("TwistSE3toSE3", [](const SE3 &Xd,const SE3 &X,double dt) {
        return TwistSE3toSE3(Xd,X,dt);
    });    

   m.def("VelSE3toSE3", [](const SE3 &Xd,const SE3 &X,double dt) {
        return VelSE3toSE3(Xd,X,dt);
    });    

   m.def("MatrixLog3", [](const SO3& R) {
        return MatrixLog3(R);
    });    

   m.def("MatrixLog6", [](const SE3& T) {
        return MatrixLog6(T);
    });    
   m.def("RpToTrans", [](const Matrix3d& R, const Vector3d& p) {
        return RpToTrans(R,p);
    });    
   m.def("FKinBody", [](const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
        return FKinBody(M,Blist,thetaList);
    });    
   m.def("RotInv", [](const SO3& rotMatrix) {
        return RotInv(rotMatrix);
    });  
   m.def("IKinBody", [](const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
        return IKinBody(Blist,M,T,thetalist,eomg,ev);
    });    
   m.def("IKinSpace", [](const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
        return IKinSpace(Slist,M,T,thetalist,eomg,ev);
    });   
   m.def("InverseDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist);
    });   
   m.def("InverseDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist,eef_mass);
    });   

   m.def("GravityForces", [](const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return GravityForces(thetalist,g,Mlist,Glist,Slist);
    });  

   m.def("GravityForces", [](const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return GravityForces(thetalist,g,Mlist,Glist,Slist,eef_mass);
    });  

   m.def("MassMatrix", [](const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return MassMatrix(thetalist,Mlist,Glist,Slist);
    });  
   m.def("MassMatrix", [](const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return MassMatrix(thetalist,Mlist,Glist,Slist,eef_mass);
    });  

   m.def("VelQuadraticForces", [](const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist);
    });  
    m.def("VelQuadraticForces", [](const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist,eef_mass);
    });  
    m.def("EndEffectorForces", [](const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist);
    });  
    m.def("EndEffectorForces", [](const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist,eef_mass);
    });  
    m.def("ForwardDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return ForwardDynamics(thetalist,dthetalist,taulist,g, Ftip,Mlist,Glist,Slist );
    });  
    m.def("ForwardDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return ForwardDynamics(thetalist,dthetalist,taulist,g, Ftip,Mlist,Glist,Slist ,eef_mass);
    });  
    m.def("EulerStep", [](JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt) {
        return EulerStep(thetalist,dthetalist,ddthetalist,dt);
    });  
    m.def("ComputedTorque", [](const JVec& thetalist, const JVec& dthetalist, const JVec& eint,
		const Vector3d& g, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist,
		const ScrewList& Slist, const JVec& thetalistd, const JVec& dthetalistd, const JVec& ddthetalistd,
		double Kp, double Ki, double Kd) {
        return ComputedTorque(thetalist,dthetalist,eint,g,Mlist,Glist,Slist,thetalistd,dthetalistd,ddthetalistd,Kp,Ki,Kd);
    });  
    m.def("QuinticTimeScalingKinematics", [](double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) {
        return QuinticTimeScalingKinematics(s0,sT,ds0,dsT,dds0,ddsT,Tf,t);
    });  
      m.def("TransInv", [](const SE3& transform) {
        return TransInv(transform);
    });    


m.def("AccVeltoVel", [](const Vector6d &Vd,const Vector6d &V,double dt) {
        return mr::AccVeltoVel(Vd,V,dt);
    });

m.def("DerivativeVectorizeJacobianBody", [](const ScrewList& Blist, const JVec& thetaList) {
        return mr::DerivativeVectorizeJacobianBody(Blist,thetaList);
    });

m.def("vec", [](const JVec& thetaList) {
        return mr::vec(thetaList);
    });


m.def("dJacobianBody", [](const Jacobian& Jb ,const JVec& dthetaList) {
        return mr::dJacobianBody(Jb,dthetaList);
    });

m.def("dAnalyticJacobianBody", [](const SE3&M, const ScrewList& Blist, const JVec& thetaList ,const JVec& dthetaList) {
        return mr::dAnalyticJacobianBody(M,Blist,thetaList,dthetaList);
    });

m.def("JointTrajectory", [](const JVec q0, const JVec qT, double Tf, double t , int method , JVec& q_des, JVec& dq_des, JVec& ddq_des) {
        return mr::JointTrajectory(q0,qT,Tf,t,method,q_des,dq_des,ddq_des);
    });

m.def("wrapTo2PI", [](double angle) {
        return mr::wrapTo2PI(angle);
    });

m.def("wrapToPI", [](double angle) {
        return mr::wrapToPI(angle);
    });

m.def("wrapTo2PI", [](const JVec& angles) {
        return mr::wrapTo2PI(angles);
    });

m.def("wrapToPI", [](const JVec& angles) {
        return mr::wrapToPI(angles);
    });

m.def("CartesianError", [](const SE3& X,const SE3& Xd) {
        return mr::CartesianError(X,Xd);
    });

m.def("pinvAnalyticJacobianBody", [](SE3 M, const ScrewList& Blist, const JVec& thetaList) {
        return mr::pinvAnalyticJacobianBody(M,Blist,thetaList);
    });

m.def("FkinBody", [](SE3 M,ScrewList Blist, const JVec& q ,const JVec& dq, SE3 &T, Jacobian &Jb,Jacobian& dJb) {
        return mr::FkinBody(M,Blist,q,dq,T,Jb,dJb);
    });

m.def("dexp3", [](const Vector3d& xi) {
        return mr::dexp3(xi);
    });

m.def("dlog3", [](const Vector3d& xi) {
        return mr::dlog3(xi);
    });

m.def("skew3", [](const Vector3d& xi) {
        return mr::skew3(xi);
    });

m.def("dexp6", [](const Vector6d& lambda) {
        return mr::dexp6(lambda);
    });

m.def("ddexp3", [](const Vector3d& xi, const Vector3d& dxi) {
        return mr::ddexp3(xi,dxi);
    });

m.def("dddexp3", [](const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
        return mr::dddexp3(xi,dxi,y,dy);
    });

m.def("ddexp6", [](const Vector6d& lambda, const Vector6d& lambda_dot) {
        return mr::ddexp6(lambda,lambda_dot);
    });

m.def("skew_sum", [](const Vector3d& a, const Vector3d& b) {
        return mr::skew_sum(a,b);
    });

m.def("ddlog3", [](const Vector3d& xi, const Vector3d& dxi) {
        return mr::ddlog3(xi,dxi);
    });

m.def("dddlog3", [](const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
        return mr::dddlog3(xi,dxi,y,dy);
    });

m.def("dlog6", [](const Vector6d& lambda) {
        return mr::dlog6(lambda);
    });

m.def("ddlog6", [](const Vector6d& lambda, const Vector6d& lambda_dot) {
        return mr::ddlog6(lambda,lambda_dot);
    });
m.def("LieScrewTrajectory", [](const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N) 
    -> std::tuple<py::list, py::list, py::list> {
        vector<SE3> Xd_list;
        Xd_list.resize(N);
        vector<Vector6d> Vd_list;
        Vd_list.resize(N);
        vector<Vector6d> dVd_list;
        dVd_list.resize(N);
        mr::LieScrewTrajectory(X0,XT,V0,VT,dV0,dVT,Tf,N,Xd_list,Vd_list,dVd_list);
         py::list py_Xd_list;
        py::list py_Vd_list;
        py::list py_dVd_list;

        for (const auto& Xd : Xd_list) {
            py_Xd_list.append(py::cast(Xd));
        }

        for (const auto& Vd : Vd_list) {
            py_Vd_list.append(py::cast(Vd));
        }

        for (const auto& dVd : dVd_list) {
            py_dVd_list.append(py::cast(dVd));
        }

        return std::make_tuple(py_Xd_list, py_Vd_list, py_dVd_list);
    });


    
     py::class_<MR_Indy7>(m, "MR_Indy7")
        .def(py::init<>()); // Constructor
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
