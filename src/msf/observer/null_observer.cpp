#include "msf/observer/null_observer.h"

namespace robosense
{
namespace localization
{
namespace msf
{
NullObserver::NullObserver(const std::shared_ptr<std::condition_variable> obs_available) : Observer(obs_available)
{
  config();
  start();
}

NullObserver::~NullObserver()
{
  stop();
}

void NullObserver::config(void)
{
  name_ = "NullObserver";
}

void NullObserver::start(void)
{
  // Config Observation
  observer_thread_active_ = true;
  obs_queue_max_size_ = 1;
  const auto& func = [this] { observe_thread(); };
  observer_thread_ = std::thread(func);
}

void NullObserver::stop(void)
{
  if (observer_thread_active_.load())
  {
    observer_thread_active_ = false;
    observer_thread_.join();
  }
}

bool NullObserver::ready(void)
{
  return true;
}

void NullObserver::observeThread(void)
{
  while (observer_thread_active_.load())
  {
    Observation obs;
    {
      std::unique_lock<std::mutex> obs_lock(obs_queue_mutex_);
      obs_queue_.emplace(std::move(obs));

      while (obs_queue_.size() > obs_queue_max_size_)
      {
        obs_queue_.pop();
      }
    }
    usleep(9399);
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense
