#include "common/logger.h"

namespace robosense
{
namespace localization
{
Logger::Logger() : buffer_length_(1000), count_(0), start_saving_(false), start_(false), use_output_buffer_(true)
{
}

Logger::Logger(std::string file_path, std::size_t buffer_length)
  : buffer_length_(buffer_length), count_(0), start_saving_(false), start_(false), use_output_buffer_(true)
{
  setFilePath(file_path, buffer_length_);
}

Logger::~Logger()
{
  use_output_buffer_ = false;  // now stop using output buffer to avoid losing contents already in output buffer
  start_ = false;
  if (output_buffer_.get() != nullptr && output_buffer_->size() > 0)
  {
    saveToFile(output_buffer_);
  }
  if (log_buffer_.get() != nullptr && log_buffer_->size() > 0)
  {
    saveToFile(log_buffer_);
  }
  os_.close();
  if (save_to_file_thread_.joinable())
    save_to_file_thread_.join();
}

void Logger::setFilePath(std::string file_path, std::size_t buffer_length)
{
  // std::cout << "File path: " << file_path << std::endl;
  os_.open(file_path, std::ios::out | std::ios::trunc);
  log_buffer_.reset(new VecStr);

  start_ = true;
  save_to_file_thread_ = std::thread(&Logger::saveToFileThread, this);
}

bool Logger::log()
{
  line_ = line_ + "\n";
  // std::cout << line_;
  log_buffer_->push_back(line_);
  line_.clear();

  count_++;
  if (count_ == buffer_length_ && use_output_buffer_.load())
  {
    count_ = 0;
    output_buffer_.reset(log_buffer_.release());
    log_buffer_.reset(new VecStr);
    start_saving_.store(true);
  }
}

void Logger::saveToFile(std::unique_ptr<VecStr>& ptr)
{
  for (auto it = ptr->begin(); it != ptr->end(); it++)
  {
    os_ << *it;
  }
}

void Logger::saveToFileThread()
{
  while (start_.load())
  {
    if (start_saving_.load())
    {
      // saving VecStr to file
      saveToFile(output_buffer_);
      output_buffer_.release();
      start_saving_ = false;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  std::cout << "Exit save to file thread." << std::endl;
}

}  // ns: localization
}  // ns: robosense
