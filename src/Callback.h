/*
 * Copyright (C) 2024  Dygma Lab S.L.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Gustavo Gomez Lopez @Noteolvides
 *
 */

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


#endif  // _CALLBACK_H_
