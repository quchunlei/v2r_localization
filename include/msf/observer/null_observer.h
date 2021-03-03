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
#ifndef RS_LOCALIZATION_NULL_OBSERVER_H
#define RS_LOCALIZATION_NULL_OBSERVER_H

#include "msf/observer/observer_base.h"

namespace robosense
{
namespace localization
{
namespace msf
{
// For now the observer is sync with Imudata.
// The odom velocity is linear interpolated to the timestamp of imudata
class NullObserver : public Observer
{
public:
  NullObserver() = delete;
  NullObserver(const std::shared_ptr<std::condition_variable> obs_available);
  ~NullObserver();
  bool ready(void);
  void start(void);
  void stop(void);

protected:
  void observeThread(void);
  void config(void);
};
}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_NULL_OBSERVER_H */
