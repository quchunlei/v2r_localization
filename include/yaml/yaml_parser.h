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
 * Date: 2018.09
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <boost/algorithm/string.hpp>
#include "yaml/yaml.h"
#include "yaml/aes_encryptor.h"
#include <memory>
#include <iostream>

// TODO path to file
namespace robosense
{
namespace localization
{
class YamlParser
{
public:
  YamlParser() = delete;
  ~YamlParser();
  YAML::Node& getYamlNode()
  {
    return *param_;
  };
  YamlParser(const std::string& key);
  YAML::Node& YamlParserMerge(const std::string& sys_cfg_path, const std::string& user_cfg_file);

private:
  std::string base_dir_;
  std::string base_file_path_;
  std::unique_ptr<YAML::Node> param_;

  AesEncryptor* pAesDecryptor;

  std::string getBaseDirectory(const std::string& path);
  bool catYAML(YAML::Node&);
  bool printNodeType(YAML::Node&);
  bool printNodeType(YAML::Node&&);
  YAML::Node& YamlParserLoadFile(const std::string& path);
};
}  // ns: localization
}  // ns: robosense

#endif  // YAML_PARSER_H
