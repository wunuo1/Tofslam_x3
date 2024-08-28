#include <loop/registration_methods.h>

// #include <imgui.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

namespace zjloc {

RegistrationMethods::RegistrationMethods() : registration_method(6), registration_resolution(2.0f), transformation_epsilon(1e-4), max_iterations(16), registration_methods({"ICP", "GICP", "NDT", "GICP_OMP", "NDT_OMP", "VGICP", "FGICP"}) {}

RegistrationMethods::~RegistrationMethods() {}
 
// void RegistrationMethods::draw_ui() {
//   ImGui::Text("Scan matching");
//   ImGui::Combo("Method", &registration_method, registration_methods.data(), registration_methods.size());
//   if(std::string(registration_methods[registration_method]).find("NDT") != std::string::npos) {
//     ImGui::DragFloat("Resolution", &registration_resolution, 0.1f, 0.1f, 20.0f);
//   }
//   ImGui::DragFloat("Transformation epsilon", &transformation_epsilon, 1e-5f, 1e-5f, 1e-2f, "%.6f");
//   ImGui::DragInt("Max iterations", &max_iterations, 1, 1, 256);
// }

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr RegistrationMethods::method() const {
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

  switch(registration_method) {
    case 0: {
      auto icp = pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>);
      registration = icp;
    } break;

    default:
      std::cerr << "warning: unknown registration method!!" << std::endl;
      std::cerr << "       : use GICP" << std::endl;
    case 1: {
      auto gicp = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>);
      registration = gicp;
    } break;

    case 2: {
      auto ndt = pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>);
      ndt->setResolution(registration_resolution);
      registration = ndt;
    } break;

    case 3: {
      auto gicp = pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>);
      registration = gicp;
    } break;

    case 4: {
      auto ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>);
      ndt->setResolution(registration_resolution);
      registration = ndt;
    } break;

    case 5: {
      fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr vgicp(new fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>());
      vgicp->setNumThreads(4);
      vgicp->setResolution(1.0);
      vgicp->setCorrespondenceRandomness(15);
      registration = vgicp;
    } break;

    case 6: {
      fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp(new fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>());
      gicp->setNumThreads(4);
      // gicp->setResolution(1.0);
      gicp->setCorrespondenceRandomness(15);
      registration = gicp;
    } break;
  }

  registration->setTransformationEpsilon(transformation_epsilon);
  registration->setMaximumIterations(max_iterations);

  return registration;
}
}  // namespace zjloc
