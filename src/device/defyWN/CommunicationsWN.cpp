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

#ifdef ARDUINO_ARCH_RP2040

#include "Communications.h"
#include "SpiPort.h"
#include "kaleidoscope/plugin/LEDControl.h"
#include <kaleidoscope.h>


#define DEBUG_LOG_N2_COMMUNICATIONS 0

#define PORT_IS_ALIVE_TIMEOUT_MS    2000

#define HOST_CONNECTION_CHECK_INTERVAL  250

#define USB_CONNECTION_MARGIN          100


static SpiPort spiPort1(0);
static Devices spiPort1Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort1LastCommunication{0};

static SpiPort spiPort2(1);
static Devices spiPort2Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort2LastCommunication{0};

static bool host_connection_requested = false;

enum class Connection_status
{
    SET_USB_TIMER,
    CHECK_USB_CONN,
    CHECK_WIRED_OR_WIRELESS,
    HOST_CONNECTED,
}conn_state = Connection_status::SET_USB_TIMER;

bool mode_led_requested = false;

void connection_state_machine ();

/****************************************************************** */

struct SideInfo {
  SideInfo(Devices _devices)
    : device(_devices) {}
  Devices device;
  bool online{false};
  uint32_t lastCommunication{0};
  bool port{false};
};

static SideInfo left{Communications_protocol::KEYSCANNER_DEFY_LEFT};
static SideInfo right{Communications_protocol::KEYSCANNER_DEFY_RIGHT};

void checkActive(SideInfo &side);


/****************************************************************** */


class WiredCommunications {
 public:
  static void init() 
  {
    spiPort1.init();
    spiPort2.init();

    Communications.callbacks.bind(IS_ALIVE, [](Packet p) {
        p.header.size    = 0;
        p.header.device  = p.header.device;
        p.header.command = IS_ALIVE;
        Communications.sendPacket(p);
    });
  }

  static void readPacket(uint8_t port) 
  {
    SpiPort &spiPort             = port == 0 ? spiPort1 : spiPort2;
    Devices &device              = port == 0 ? spiPort1Device : spiPort2Device;
    uint32_t &lastCommunications = port == 0 ? spiPort1LastCommunication : spiPort2LastCommunication;
    Packet packet{};
    
    if (spiPort.readPacket(packet)) 
    {
      lastCommunications = millis();
      device             = packet.header.device;
      Communications.callbacks.call(packet.header.command, packet);
    }
  }

  static bool isPortAlive(uint8_t port) {
    uint32_t &lastCommunications = port == 0 ? spiPort1LastCommunication : spiPort2LastCommunication;

    /* Check if there was any communication at all */
    if (lastCommunications == 0) {
      return false;
    }

    /* Check how long it is since the last communication */
    return ((millis() - lastCommunications) < PORT_IS_ALIVE_TIMEOUT_MS) ? true : false;
  }

  static bool isPortLeftAlive(void) {
    if (spiPort1Device == KEYSCANNER_DEFY_LEFT) {
      return isPortAlive(0);
    } else if (spiPort2Device == KEYSCANNER_DEFY_LEFT) {
      return isPortAlive(1);
    } else {
      return false;
    }
  }

  static bool isPortRightAlive(void) {
    if (spiPort1Device == KEYSCANNER_DEFY_RIGHT) {
      return isPortAlive(0);
    } else if (spiPort2Device == KEYSCANNER_DEFY_RIGHT) {
      return isPortAlive(1);
    } else {
      return false;
    }
  }

  static void disconnect(uint8_t port) {
    SpiPort &spiPort = port == 0 ? spiPort1 : spiPort2;
    Devices &device  = port == 0 ? spiPort1Device : spiPort2Device;
    Packet packet{};

    packet.header.command = Communications_protocol::DISCONNECTED;
    packet.header.device  = device;
    Communications.callbacks.call(packet.header.command, packet);
    device = UNKNOWN;
    //Remove all the left packets at disconnections
    spiPort.clearRead();
    spiPort.clearSend();
  }

  static void portRun(uint8_t port) {
    SpiPort &spiPort = port == 0 ? spiPort1 : spiPort2;

    spiPort.run();
  }

  static void run() {

    auto const &keyScanner       = kaleidoscope::Runtime.device().keyScanner();
    static bool wasLeftConnected = false;
    auto isDefyLeftWired         = keyScanner.leftSideWiredConnection();
    if (isDefyLeftWired) {
      portRun(0);
      readPacket(0);
    }
    if (wasLeftConnected && !isDefyLeftWired) {
      disconnect(0);
    }
    wasLeftConnected = isDefyLeftWired;

    static bool wasRightConnected = false;
    auto isDefyRightWired         = keyScanner.rightSideWiredConnection();
    if (isDefyRightWired) {
      portRun(1);
      readPacket(1);
    }
    if (wasRightConnected && !isDefyRightWired) {
      disconnect(1);
    }
    wasRightConnected = isDefyRightWired;
  }
};

void Communications::init() {

  callbacks.bind(CONNECTED, [this](Packet p) {
    p.header.size    = 0;
    p.header.device  = p.header.device;
    p.header.command = CONNECTED;
    sendPacket(p);
  });

  callbacks.bind(HOST_CONNECTION_STATUS, [](Packet p) {
        //Keyscanner asked for the host connection.
        host_connection_requested = true;
  });

  callbacks.bind(MODE_LED, [](Packet p) {
        //Mode LED requested from KS.
        mode_led_requested = true;
  });

  WiredCommunications::init();
}

void Communications::run() {

  WiredCommunications::run();
  connection_state_machine();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;

  if (device_to_send == UNKNOWN) {

    if (spiPort1Device != UNKNOWN) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort1.sendPacket(packet);
    }

    if (spiPort2Device != UNKNOWN) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort2.sendPacket(packet);
    }
    return true;
  }

  if (spiPort1Device == device_to_send) {
    packet.header.device = Communications_protocol::NEURON_DEFY;
    spiPort1.sendPacket(packet);
  }

  if (spiPort2Device == device_to_send) {
    packet.header.device = Communications_protocol::NEURON_DEFY;
    spiPort2.sendPacket(packet);
  }

  return true;
}

bool Communications::isWiredLeftAlive() {
  return WiredCommunications::isPortLeftAlive();
}

bool Communications::isWiredRightAlive() {
  return WiredCommunications::isPortRightAlive();
}

/*
 * This function is called when the USB is connected or disconnected.
 * For the moment it only logs the event.
 * We will use it to decide if we send the disconnect message to the host. Or not.
 */
bool check_usb_connection()
{
    return  tud_ready();
}

static void send_host_connection(bool connected)
{
    Communications_protocol::Packet packet{};
    packet.header.command = Communications_protocol::HOST_CONNECTION;
    packet.header.size    = 2;
    packet.data[0]        = connected;
    Communications.sendPacket(packet);
    //**************************************
}

void connection_state_machine ()
{
    //HOST CONNECTION
    static bool host_connected = false;
    static bool prev_host_connected = true;
    static uint32_t last_host_connection_check = 0;

    switch (conn_state)
    {
        case Connection_status::SET_USB_TIMER:
        {
            last_host_connection_check = millis();
            conn_state = Connection_status::CHECK_USB_CONN;
        }
        break;

        case Connection_status::CHECK_USB_CONN:
        {
            //Small debounce for the connection check.
            if (millis() - last_host_connection_check < HOST_CONNECTION_CHECK_INTERVAL)
            {
                return;
            }
            last_host_connection_check = millis();

            if (!check_usb_connection())
            {
                host_connected = false;
            }
            else
            {
                host_connected = true;
            }

            if(mode_led_requested && host_connected)
            {
                mode_led_requested = false;
                ::LEDControl.set_mode(::LEDControl.get_mode_index()); //Send the mode to the KS.
            }
            else if(mode_led_requested && !host_connected)
            {
                //Mode LED asked but the host is not connected.
                mode_led_requested = false;
            }

            // host_connection_requested will be true if the KS has requested the host connection.
            if(prev_host_connected != host_connected || host_connection_requested)
            {
                //Host connection changed or the KS has requested the host connection.
                // This will override the previous state.
                conn_state = Connection_status::HOST_CONNECTED;
            }

        }
        break;

        case Connection_status::HOST_CONNECTED:
        {
            if (!host_connected)
            {
                last_host_connection_check = millis();
            }
            else
            {
                ::LEDControl.set_mode(::LEDControl.get_mode_index());
            }

            //Send the connected message to the KS
            send_host_connection(host_connected);

            prev_host_connected = host_connected;
            host_connection_requested = false;

            conn_state = Connection_status::CHECK_USB_CONN;
        }
        break;

        default:
        break;


    }
}

class Communications Communications;

#endif
