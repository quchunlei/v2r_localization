#include "icp_localization/icp_point_filter.h"
#include "icp_localization/icp_sampler.h"
#include "icp_localization/icp_pre_matcher.h"
#include "icp_localization/icp_matcher.h"
#include "icp_localization/icp_evaluator.h"
#include "map_server/map_server.h"
#include "map_server/celled_map.h"
#include <ros/ros.h>
#include "common/point_cloud_viewer.h"

using namespace robosense::localization;

PointCloudViewer viewer("view");
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_icp_localization");
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");
  std::string parameter_file;
  std::string map_file;
  pri_nh.getParam("parameter_file", parameter_file);
  pri_nh.getParam("map_file", map_file);
  YAML::Node parameters_ = YAML::LoadFile(parameter_file);
  parameters_["MapServer"]["map_file"] = map_file;

  /******************* Init ******************/
  auto* transform = PM::get().TransformationRegistrar.create("RigidTransformation");
  std::shared_ptr<MapServer> map = std::make_shared<MapServer>(parameters_["MapServer"]);
  parameters_ = parameters_["RSLocalizationNormalRun"]["observers"][2];

  // ICP Point Filter
  ICPCustomPointFilter filter(parameters_["point_filter"]);
  // ICP Sampler
  ICPTransRotateSampler sampler(map, parameters_["sampler"]);
  // ICP pre matcher
  ICPNullPreMatcher pre_matcher(parameters_["pre_matcher"]);
  // ICP matcher
  PMICPMatcher matcher(parameters_["matcher"]);
  // ICP evaluator
  ICPMeanDistEvaluator evaluator(parameters_["evaluator"]);

  /****************** Run *********************/
  // const DP local_map = map.getLocalMap(Eigen::Vector3f(0, 0, 0));
  // std::cout << "number of points in local map " << local_map.getNbPoints() << std::endl;
  int pos_x = 50;
  int pos_y = 50;
  DP scan = map->getLocalMap(Eigen::Vector3f(pos_x, pos_y, 0));

  Eigen::Matrix4f trans_real = Eigen::Matrix4f::Identity();
  trans_real(0, 3) = pos_x;
  trans_real(1, 3) = pos_y;
  trans_real(2, 3) = 0;

  scan = transform->compute(scan, trans_real.inverse());

  Eigen::Matrix4f trans_guess = Eigen::Matrix4f::Identity();
  trans_guess(0, 3) = pos_x + 2;
  trans_guess(1, 3) = pos_y - 2;
  trans_guess(2, 3) = 0;

  ICPSamples samples;
  filter.run(scan);
  sampler.run(scan, trans_guess, samples);
  pre_matcher.run(samples);
  matcher.run(samples);
  Eigen::Matrix4f final_tf;
  Eigen::Matrix<float, 6, 6> final_cov;
  float final_score;
  evaluator.run(samples, final_tf, final_cov, final_score);

  //   auto final_tf =
  //       samples.rotated_samples_.front().map_tf_ * std::get<1>(samples.rotated_samples_.front().samples_.front());
  auto final_scan = scan;
  auto final_map = samples.rotated_samples_.front().map_;

  final_scan = transform->compute(final_scan, final_tf);
  //   std::cout << "The final trans is x = " << trans.front()(0.3) << "y = " << trans.front()(1.3) << std::endl;

  viewer.setPointCloud(final_map, Eigen::Vector3i(0, 250, 0), "map");
  viewer.setPointCloud(final_scan, Eigen::Vector3i(0, 0, 250), "scan");

  std::cout << final_tf << std::endl;
  while (ros::ok())
    sleep(1);
  return 0;
}
