#include "yaml/yaml_parser.h"
#include "yaml/aes_encryptor.h"
#include "common/prompt.h"
#include <unistd.h>
#include <sstream>
#include <fstream>

namespace robosense
{
namespace localization
{
YamlParser::YamlParser(const std::string& key)
{
  unsigned char lkey[16] = { 0 };
  int len = key.length() < 16 ? key.length() : 16;

  key.copy((char*)lkey, len, 0);
  pAesDecryptor = new AesEncryptor(lkey);
}

YamlParser::~YamlParser()
{
  if (pAesDecryptor != NULL)
    delete pAesDecryptor;
}

YAML::Node& YamlParser::YamlParserLoadFile(const std::string& path)
{
  param_ = std::make_unique<YAML::Node>();

  if (access(path.c_str(), F_OK) != 0)
  {
    ERROR << "No such file: " << path << REND;
    exit(0);
  }

  // ifstream read yaml file
  std::stringstream ss;
  std::string str;

  std::ifstream fin(path, std::ios::in);

  std::string strDecrypt("#! Encrypt");
  std::getline(fin, str);

  if (str == strDecrypt)
  {  // file haved encrypted
    while (std::getline(fin, str))
    {
      strDecrypt = pAesDecryptor->DecryptString(str);
      ss << strDecrypt << std::endl;
    }
  }
  else
  {  // file not eccrypted
    ss << str << std::endl;
    while (std::getline(fin, str))
    {
      ss << str << std::endl;
    }
  }
  fin.close();

  *param_ = YAML::Load(ss.str());

  catYAML(*param_);

  return *param_;
}

YAML::Node& YamlParser::YamlParserMerge(const std::string& sys_cfg_file, const std::string& user_cfg_file)
{
  if ((access(sys_cfg_file.c_str(), F_OK) != 0))
  {
    ERROR << "System config missing: " << sys_cfg_file << REND;
    exit(0);
  }

  base_dir_ = getBaseDirectory(sys_cfg_file);
  YAML::Node& sysNode = YamlParserLoadFile(sys_cfg_file);

  if (user_cfg_file.size() == 0)
  {
    WARNING << "No user config file provided." << REND;
    WARNING << "Using Default parameters..." << REND;
  }
  else if (access(user_cfg_file.c_str(), F_OK) != 0)
  {
    WARNING << "No such config file: " << user_cfg_file << REND;
    WARNING << "Default parameters loaded..." << REND;
    WARNING << "You may want to relaunch with correct config file." << REND;
  }
  else
  {
    YAML::Node userNode = YAML::LoadFile(user_cfg_file);
    if (userNode.Type() == YAML::NodeType::Map)
    {
      for (YAML::iterator it = userNode.begin(); it != userNode.end(); ++it)
      {
        YAML::Node& subNode = it->second;
        for (YAML::iterator sit = subNode.begin(); sit != subNode.end(); ++sit)
        {
          YAML::Node subNodeSys = sysNode[it->first.as<std::string>()];
          if (subNodeSys && subNodeSys[sit->first.as<std::string>()])
          {
            subNodeSys[sit->first.as<std::string>()] = sit->second;
          }
          else if (!subNodeSys[sit->first.as<std::string>()])
          {
            subNodeSys[sit->first.as<std::string>()] = sit->second;
          }
        }
      }
    }
  }

  return sysNode;
}

bool YamlParser::printNodeType(YAML::Node& node)
{
  switch (node.Type())
  {
    case YAML::NodeType::Null:
      std::cout << ":Null";  // << std::endl;
      break;
    case YAML::NodeType::Scalar:
      std::cout << ":Scalar";  // << std::endl;
      break;
    case YAML::NodeType::Sequence:
      std::cout << ":Sequence";  // << std::endl;
      break;
    case YAML::NodeType::Map:
      std::cout << ":Map";  //<< std::endl;
      break;
    case YAML::NodeType::Undefined:
      std::cout << ":Undefined";  // << std::endl;
      break;
    default:
      std::cout << ":NodeType: " << node.Type();  // << std::endl;
  }
  return true;
}

bool YamlParser::YamlParser::printNodeType(YAML::Node&& node)
{
  switch (node.Type())
  {
    case YAML::NodeType::Null:
      std::cout << ":Null" << std::endl;
      break;
    case YAML::NodeType::Scalar:
      std::cout << ":Scalar" << std::endl;
      break;
    case YAML::NodeType::Sequence:
      std::cout << ":Sequence" << std::endl;
      break;
    case YAML::NodeType::Map:
      std::cout << ":Map" << std::endl;
      break;
    case YAML::NodeType::Undefined:
      std::cout << ":Undefined" << std::endl;
      break;
    default:
      std::cout << ":NodeType: " << node.Type() << std::endl;
  }
  return true;
}

std::string YamlParser::getBaseDirectory(const std::string& path)
{
  std::vector<std::string> split_path;
  boost::split(split_path, path, boost::is_any_of("/"));
  std::string base_directory("/");
  for (std::size_t i = 0; i < split_path.size() - 1; ++i)
  {
    if (split_path[i].size() != 0)
    {
      base_directory = base_directory + split_path[i] + "/";
    }
  }

  return base_directory;
}

bool YamlParser::catYAML(YAML::Node& node)
{
  if (node.Type() == YAML::NodeType::Null)
  {
    // std::cout <<"Node is null" << std::endl;
    return false;
  }
  if (node.Type() == YAML::NodeType::Scalar)
  {
    // std::cout <<"Node is Scalar" << std::endl;
    return false;
  }

  try
  {
    if (node.Type() == YAML::NodeType::Map)
    {
      if (node["include"])
      {
        std::string path(base_dir_ + node["include"].as<std::string>());
        //  std::cout << "[include]" << path << std::endl;
        // ifstream read yaml file
        std::stringstream ss;
        std::string str;

        std::ifstream fin(path, std::ios::in);

        std::string strDecrypt("#! Encrypt");
        std::getline(fin, str);
        if (str == strDecrypt)
        {
          while (std::getline(fin, str))
          {
            strDecrypt = pAesDecryptor->DecryptString(str);
            ss << strDecrypt << std::endl;
          }
        }
        else
        {
          ss << str << std::endl;
          while (std::getline(fin, str))
          {
            ss << str << std::endl;
          }
        }
        fin.close();

        YAML::Node included_sub_node = YAML::Load(ss);

        if (included_sub_node.Type() != YAML::NodeType::Null)
        {
          catYAML(included_sub_node);
          node = included_sub_node;
        }
        else
        {
          ERROR << "No file named: " << path << RESET << END;
        }
      }

      for (YAML::iterator it = node.begin(); it != node.end(); ++it)
      {
        YAML::Node& sub_node = it->second;
        catYAML(sub_node);
      }
    }
    else  // node is a yaml sequence
    {
      for (YAML::iterator it = node.begin(); it != node.end(); ++it)
      {
        YAML::Node sub_node = *it;
        catYAML(sub_node);
      }
    }
  }
  catch (std::exception& e)
  {
    // std::cerr << "YAML parsing error: " << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace localization
}  // namespace robosense
