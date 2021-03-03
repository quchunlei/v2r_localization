#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include "msf/core/msf_state.h"
#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <atomic>
#include <thread>

namespace robosense
{
namespace localization
{
class Logger
{
public:
  Logger();
  Logger(std::string file_path, std::size_t buffer_length);
  Logger(std::string file_path);
  ~Logger();
  template <typename T1, typename... T2>
  bool log(T1 t1, T2... t2);
  bool log();

private:
  using VecStr = std::vector<std::string>;
  std::ofstream os_;
  std::size_t buffer_length_;
  std::unique_ptr<VecStr> log_buffer_;
  std::unique_ptr<VecStr> output_buffer_;
  std::string line_;
  std::size_t count_;
  std::atomic_bool start_;
  std::atomic_bool start_saving_;
  std::atomic_bool use_output_buffer_;
  std::thread save_to_file_thread_;

  void setFilePath(std::string file_path, std::size_t buffer_length);
  void saveToFile(std::unique_ptr<VecStr>& ptr);
  void saveToFileThread();
};

template <typename T1, typename... T2>
bool Logger::log(T1 t1, T2... t2)
{
  std::stringstream ss;
  ss << std::setprecision(std::numeric_limits<double>::digits10) << t1;  // keep full double pricision
  line_ = line_ + ss.str() + " ";
  log(t2...);
};

}  // namespace localization
}  // namespace robosense

#endif