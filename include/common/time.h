/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group
 * Version: 0.2.0
 * Date: 2018.05
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#ifndef RS_LOCALIZATION_TIME_H
#define RS_LOCALIZATION_TIME_H

#include <chrono>
#include <iostream>

#include "common/prompt.h"

namespace robosense
{
namespace localization
{
class Timer
{
public:
  Timer()
  {
    reset();
  }

  void start(void)
  {
    start_time_ = std::chrono::system_clock::now();
    running_ = true;
  }

  void stop(void)
  {
    end_time_ = std::chrono::system_clock::now();
    running_ = false;
  }

  void reset(void)
  {
    start_time_ = std::chrono::system_clock::now();
    end_time_ = std::chrono::system_clock::now();
    running_ = false;
  }

  void print(void)
  {
    std::chrono::duration<double> elapsed_seconds;
    if (running_)
    {
      auto curr_time = std::chrono::system_clock::now();
      elapsed_seconds = curr_time - start_time_;
    }
    else
    {
      elapsed_seconds = end_time_ - start_time_;
    }
    INFO << "Elapsed_sencods = " << elapsed_seconds.count() << RESET << END;
  }

  double get(void)
  {
    std::chrono::duration<double> elapsed_seconds;
    if (running_)
    {
      auto curr_time = std::chrono::system_clock::now();
      elapsed_seconds = curr_time - start_time_;
    }
    else
    {
      elapsed_seconds = end_time_ - start_time_;
    }
    return elapsed_seconds.count();
  }

private:
  std::chrono::system_clock::time_point start_time_;
  std::chrono::system_clock::time_point end_time_;
  bool running_;
};
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_TIME_H */