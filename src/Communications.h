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

#ifndef _COMMUNICATIONS_H_
#define _COMMUNICATIONS_H_


#include "Communications_protocol.h"
#include "Callback.h"

using namespace Communications_protocol;


class Communications {
 public:
  void init();

  void run();

  bool isWiredLeftAlive();
  bool isWiredRightAlive();

  bool sendPacket(Packet data);
  bool sendPacketHostConnection( void );

  void get_keyscanner_configuration();

   void request_keyscanner_layers();

   bool is_host_connected();

  BindingCallbacks<Commands, Packet> callbacks{};
};

extern Communications Communications;


#endif  //_COMMUNICATIONS_H_
