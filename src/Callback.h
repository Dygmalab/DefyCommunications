#ifndef _CALLBACK_H_
#define _CALLBACK_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

template<class... Arguments>
class Callback {
 public:
  using Function    = std::function<void(Arguments...)>;
  using FunctionPtr = std::shared_ptr<Function>;
  using Functions   = std::vector<FunctionPtr>;

  FunctionPtr addListener(const Function &f) {
    FunctionPtr fp = std::make_shared<Function>(f);
    functions_.push_back(fp);
    return fp;
  }

  size_t numberOfListeners() const {
    return functions_.size();
  }

  void removeListener(const FunctionPtr &fp) {
    for (auto it = functions_.begin(); it != functions_.end();) {
      if (*it == fp) {
        functions_.erase(it);
        return;
      } else
        ++it;
    }
  }

  void operator()(const Arguments &...args) const {
    for (const auto &f : functions_)
      f.get()->operator()(args...);
  }

 private:
  Functions functions_;
};

template<class Key, class Value>
class BindingCallbacks {
 public:
  using Function = typename Callback<Value>::Function;

  typename Callback<Value>::FunctionPtr bind(const Key &command, const Function &function) {
    if (callbacks.find(command) == callbacks.end())
      callbacks.insert({command, std::unique_ptr<Callback<Value>>{new Callback<Value>}});
    return callbacks[command]->addListener(function);
  }

  void unBind(const Key &command, const typename Callback<Value>::FunctionPtr &id) {
    if (callbacks.find(command) == callbacks.end()) return;
    callbacks[command]->removeListener(id);
    if (callbacks[command]->numberOfListeners() == 0) {
      callbacks.erase(command);
    }
  }

  void call(const Key &key, const Value &value) {
    if (callbacks.find(key) == callbacks.end()) return;
    callbacks[key]->operator()(value);
  }

  std::unordered_map<Key, std::unique_ptr<Callback<Value>>> callbacks;
};

#endif