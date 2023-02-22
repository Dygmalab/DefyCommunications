
#ifndef _COMMUNICATIONS_H_
#define _COMMUNICATIONS_H_

#include "KeyScanner_communications_protocol.h"
#include "Callback.h"
using namespace KeyScanner_communications_protocol;

class Communications {
 public:
  void init();

  void run();

  bool sendPacket(Packet data);

  BindingCallbacks<Commands, Packet> callbacks{};
};

extern Communications Communications;

#endif  //_COMMUNICATIONS_H_
