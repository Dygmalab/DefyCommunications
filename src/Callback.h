#ifndef _CALLBACK_H_
#define _CALLBACK_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

template<class... Arguments>
class Callback {
 public:
  using Function  = std::function<void(Arguments...)>;
  using Functions = std::vector<Function>;

  void addListener(const Function &f) {
    functions_.push_back(f);
  }

  void operator()(const Arguments &...args) const {
    for (const auto &f : functions_)
      f.operator()(args...);
  }

 private:
  Functions functions_;
};

template<class Key, class Value>
class BindingCallbacks {
 public:
  void bind(const Key &command, const typename Callback<Value>::Function &function) {
    for (auto &item : callbacks) {
      if (item.key == command) {
        item.callback->addListener(function);
        return;
      }
    }
    callbacks.push_back({command, std::unique_ptr<Callback<Value>>{new Callback<Value>}});
    for (auto &item : callbacks) {
      if (item.key == command) {
        item.callback->addListener(function);
        return;
      }
    }
  }

  void call(const Key &command, const Value &value) {
    for (auto &item : callbacks) {
      if (item.key == command) {
        item.callback->operator()(value);
      }
    }
  }

  void operator()(const Key &command, const Value &value) {
    for (auto &item : callbacks) {
      if (item.key == command) {
        item.callback->operator()(value);
      }
    }
  }

 private:
  struct Join {
    Key key;
    std::unique_ptr<Callback<Value>> callback;
  };
  std::vector<Join> callbacks;
};

#endif