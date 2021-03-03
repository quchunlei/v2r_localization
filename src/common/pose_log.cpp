#include <fstream>
#include "msf/core/msf_state.h"

namespace robosense
{
namespace localization
{
class PoseLogger
{
public:
  PoseLogger(std::string file);
  ~PoseLogger();
  void resetFile(std::string file);
  void saveToRAM(const msf::MsfState& state);
  void saveToFile();

private:
  struct Pose
  {
    int frame;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    double stamp;
  };
  std::ofstream os_;
  std::vector<Pose> vec_;

  Pose stateToPose(const msf::MsfState& state);
};

PoseLogger::PoseLogger(std::string file)
{
  os_.open(file);
}

PoseLogger::~PoseLogger()
{
  os_.close();
}

void PoseLogger::saveToRAM(const msf::MsfState& state)
{
  vec_.emplace_back(stateToPose(state));
}

PoseLogger::Pose PoseLogger::stateToPose(const msf::MsfState& state)
{
  PoseLogger::Pose pose;
  pose.stamp = state.timestamp;
  pose.frame = state.seq;
  Eigen::Vector3f position = state.state_vec.block<3, 1>(msf::StatePositionIdx, 0);
  pose.x = position.x();
  pose.y = position.y();
  pose.z = position.z();

  Eigen::Quaternionf q(state.state_vec(msf::StateAngleIdx + 3), state.state_vec(msf::StateAngleIdx),
                       state.state_vec(msf::StateAngleIdx + 1), state.state_vec(msf::StateAngleIdx + 2));

  auto ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
  pose.roll = ypr[2];
  pose.pitch = ypr[1];
  pose.yaw = ypr[0];

  return pose;
}

}  // ns: localization
}  // ns: robosense
