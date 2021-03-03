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

#ifndef RS_LOCALIZATION_YAML_H
#define RS_LOCALIZATION_YAML_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <common/prompt.h>

namespace robosense
{
namespace localization
{
namespace common
{
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, bool& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<bool>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, int32_t& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<int32_t>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, uint32_t& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<uint32_t>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, float& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<float>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, double& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<double>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, std::string& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<std::string>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, int64_t& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<int64_t>();
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, uint64_t& out_val)
{
  if (!yaml[key])
  {
    WARNING << "Warning, not set " << key << " , use default value !" << RESET << END;
    return false;
  }
  else
  {
    out_val = yaml[key].as<uint64_t>();
    return true;
  }
}

///
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, bool& out_val, const bool& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, int32_t& out_val, const int32_t& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, uint32_t& out_val, const uint32_t& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, float& out_val, const float& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, double& out_val, const double& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, std::string& out_val,
                     const std::string& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, int64_t& out_val, const int64_t& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

inline bool yamlRead(const YAML::Node& yaml, const std::string& key, uint64_t& out_val, const uint64_t& default_val)
{
  if (!yamlRead(yaml, key, out_val))
  {
    out_val = default_val;
    return false;
  }
  else
  {
    return true;
  }
}

std::string getBaseDirectory(std::string path);
bool catYAML(YAML::Node& node);

inline bool catenateYAML(YAML::Node& node, const std::string& file_path)
{
  std::string base_dir = getBaseDirectory(file_path);
  return catYAML(node);
}

}  // namespace common
}  // namespace localization
}  // namespace robosense
#endif /* RS_LOCALIZATION_YAML_H */
