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

#ifdef NRF52_ARCH
#include "Communications.h"
#include "Communications_rf_pipe.h"
#include "CommunicationsN2_side.h"
#include "SpiPort.h"
#include "rf_host_device_api.h"
#include "CRC_wrapper.h"
#include "Arduino.h"
#include "Adafruit_USBD_Device.h"
#include "Radio_manager.h"
#include "Kaleidoscope-IdleLEDsDefy.h"
#include "Kaleidoscope-LEDControl.h"
#include "Battery.h"
#include "Ble_manager.h"
#include "FirmwareVersion.h"


#define DEBUG_LOG_N2_COMMUNICATIONS     0

#define PORT_IS_ALIVE_TIMEOUT_MS        2000

// This interval exist to prevent the host to be connected and disconnected too fast when the user is pluging and unplugging the USB cable.
#define HOST_CONNECTION_CHECK_INTERVAL  250

#define USB_CONNECTION_TIMEOUT         500

#define USB_CONNECTION_MARGIN          100

typedef enum
{
    COM_SPIPORT_STATE_DISCONNECTED = 1,
    COM_SPIPORT_STATE_CONNECTED,
} com_spiPort_state_t;

typedef struct
{
    com_spiPort_state_t state;

    SpiPort * p_spiPort;
    ComN2Side * p_comN2Side;
} com_spiPort_t;

static SpiPort spiPort1(1);
static SpiPort spiPort2(2);
static ComN2Side comN2SideLeft( COM_SIDE_TYPE_KS_LEFT );
static ComN2Side comN2SideRight( COM_SIDE_TYPE_KS_RIGHT );
static ComRfPipe comRfPipeLeft( RFGW_PIPE_ID_KEYSCANNER_LEFT );
static ComRfPipe comRfPipeRight( RFGW_PIPE_ID_KEYSCANNER_RIGHT );

static com_spiPort_t com_spiPort1 = { .state = COM_SPIPORT_STATE_DISCONNECTED, .p_spiPort = &spiPort1, .p_comN2Side = NULL };
static com_spiPort_t com_spiPort2 = { .state = COM_SPIPORT_STATE_DISCONNECTED, .p_spiPort = &spiPort2, .p_comN2Side = NULL };

static bool host_connection_requested = false;

enum class HostConnectionStatus
{
    HOST_CONNECTED,
    CHECK_CONNECTION,
    WAIT_RESPONSE
} host_connection_status;

enum class Connection_status
{
    IDLE,
    SET_USB_TIMER,
    CHECK_FORCE_BLE,
    INIT_BT,
    CHECK_USB_CONN,
    CHECK_WIRED_OR_WIRELESS,
    CHECK_USB_TIMER,
    CHECK_NEURON_CONNECTORS,
    RESET_NEURON,
    HOST_CONNECTED,
    PAIRING_MODE_KEY_CHECK

}conn_state = Connection_status::SET_USB_TIMER;

enum class Side
{
    LEFT = 1,
    RIGHT,
    BOTH,
    NONE
};

bool mode_led_requested = false;

//HOST CONNECTION STATUS
static bool host_connected = false;

void checkActive();

void new_connection_handle(void)
{
    IdleLEDsDefy.new_connection_set();
}

class RFGWCommunications {
 public:
  static void cbPipeDisconnection(rfgw_pipe_id_t pipeId) {
#if DEBUG_LOG_N2_COMMUNICATIONS
    NRF_LOG_DEBUG("Disconnected RF %lu", pipeId);
#endif

    ComRfPipe * p_rfPipe = ( pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ) ? &comRfPipeRight : &comRfPipeLeft;

    p_rfPipe->set_disconnected( );

  };

  static void cbPipeConnection(rfgw_pipe_id_t pipeId) {

    ComRfPipe * p_rfPipe = ( pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ) ? &comRfPipeRight : &comRfPipeLeft;

    p_rfPipe->set_connected();

    new_connection_handle();

#if DEBUG_LOG_N2_COMMUNICATIONS
    NRF_LOG_DEBUG("Connected RF %lu", pipeId);
#endif
  };

  static void init() {
    rfgw_cb_pipe_disconnection_set(cbPipeDisconnection);
    rfgw_cb_pipe_connection_set(cbPipeConnection);
  }

  static void run() {
    if (!kaleidoscope::plugin::RadioManager::isInited()) return;
    kaleidoscope::plugin::RadioManager::poll();
    comRfPipeLeft.run();
    comRfPipeRight.run();
  }
};

class WiredCommunications
{
public:
    static void init()
    {
        spiPort1.init( );
        spiPort2.init( );
    }

    static inline void _com_spiPort_state_set( com_spiPort_t * p_com_spiPort, com_spiPort_state_t state )
    {
        p_com_spiPort->state = state;
    }

    static void _com_spiPort_state_connected_set( com_spiPort_t * p_com_spiPort, ComN2Side * p_comN2Side )
    {
        p_com_spiPort->p_comN2Side = p_comN2Side;
        p_com_spiPort->p_comN2Side->spi_port_register( p_com_spiPort->p_spiPort );

        _com_spiPort_state_set( p_com_spiPort, COM_SPIPORT_STATE_CONNECTED );

        new_connection_handle( );
    }

    static void _com_spiPort_state_disconnected_process( com_spiPort_t * p_com_spiPort )
    {
        Packet packet{};

        /* Check whether the SPI port is connected to a side */
        if( p_com_spiPort->p_spiPort->is_connected() == false )
        {
            /* The SPI port is not connected */
            return;
        }

        /* The SPI port is connected - check which side it is */

        if ( p_com_spiPort->p_spiPort->peekPacket( packet ) == false )
        {
            /* No packet in the buffer */
            return;
        }

        switch( packet.header.device )
        {
            case KEYSCANNER_DEFY_LEFT:
//            case RF_DEFY_LEFT:
            case BLE_DEFY_LEFT:

                /* Set the SPI COM left connectivity */
                _com_spiPort_state_connected_set( p_com_spiPort, &comN2SideLeft );

                break;

            case KEYSCANNER_DEFY_RIGHT:
//            case RF_DEFY_RIGHT:
            case BLE_DEFY_RIGHT:

                /* Set the SPI COM right connectivity */
                _com_spiPort_state_connected_set( p_com_spiPort, &comN2SideRight );

                break;

            default:

                ASSERT_DYGMA( false, "Unexpected SPI packet Device type" );

                break;
        }
    }

    static void _com_spiPort_state_connected_process( com_spiPort_t * p_com_spiPort )
    {
        /* Check whether the Side is still connected over wire */
        if( p_com_spiPort->p_comN2Side->spi_is_connected() == true )
        {
            return;
        }

        /* The side is disconnected over wire */

        p_com_spiPort->p_comN2Side = nullptr;
        _com_spiPort_state_set( p_com_spiPort, COM_SPIPORT_STATE_DISCONNECTED );
    }

    static void com_spiPort_machine( com_spiPort_t * p_com_spiPort )
    {
        /* Run the spi port */
        p_com_spiPort->p_spiPort->run();

        switch( p_com_spiPort->state )
        {
            case COM_SPIPORT_STATE_DISCONNECTED:

                _com_spiPort_state_disconnected_process( p_com_spiPort );

                break;

            case COM_SPIPORT_STATE_CONNECTED:

                _com_spiPort_state_connected_process( p_com_spiPort );

                break;
        }
    }

    static void run()
    {
        com_spiPort_machine( &com_spiPort1 );
        com_spiPort_machine( &com_spiPort2 );
    }
};


void Communications::get_keyscanner_configuration(){
  NRF_LOG_DEBUG("Sending configuration command to KS (broadcast)");
  Communications_protocol::Packet p{};
  p.header.size = 1;
  p.header.command = Communications_protocol::CONFIGURATION;
  sendPacket(p);
}


void Communications::init()
{
  callbacks.bind(CONNECTED, [this](Packet p) {

#if DEBUG_LOG_N2_COMMUNICATIONS
    NRF_LOG_INFO("Get connected from %i", p.header.device);
#endif

    get_keyscanner_configuration();

    /* Send the Host connection info */
    sendPacketHostConnection( );
  });

  callbacks.bind(HOST_CONNECTION_STATUS, [this](Packet p)
  {
      //Keyscanner will ask for the host connection.
      //NRF_LOG_INFO("HOST CONNECTION ASKED");
      host_connection_requested = true;
  });

  callbacks.bind(MODE_LED, [this](Packet p)
  {
          NRF_LOG_INFO("MODE LED ASKED");
          mode_led_requested = true;
  });

  WiredCommunications::init();
  RFGWCommunications::init();
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

bool Communications::is_host_connected()
{
    return host_connected || ble_connected();
}

void connection_state_machine ()
{
    //BLE STATUS
    static bool force_ble_enabled = _BleManager.getForceBle();
    static bool ble_denied = false;

    //USB CONNECTION
    volatile uint32_t usbConnectionTime = 0;

    //HOST CONNECTION
    static bool prev_host_connected = false;
    static uint32_t last_host_connection_check = 0;

    // KEYSCANNER INSTANCE
    auto const &keyScanner = kaleidoscope::Runtime.device().keyScanner();

    //CABLES CONNECTIONS
    uint8_t bat_status_l = kaleidoscope::plugin::Battery::get_battery_status_left();
    uint8_t bat_status_r = kaleidoscope::plugin::Battery::get_battery_status_right();
    auto isDefyLeftWired = keyScanner.leftSideWiredConnection();
    auto isDefyRightWired = keyScanner.rightSideWiredConnection();

    //WIRELESS MODE STATUS
    bool bleInitiated = ble_innited();
    bool radioInited = kaleidoscope::plugin::RadioManager::isInited();

    switch (conn_state)
    {
        case Connection_status::SET_USB_TIMER:
        {
            //NRF_LOG_INFO("SET USB TIMER");
            usbConnectionTime = millis();
            conn_state = Connection_status::CHECK_FORCE_BLE;
        }
        break;

        case Connection_status::CHECK_FORCE_BLE:
        {
            //NRF_LOG_INFO("CHECK FORCE BLE");
            //If the force ble is enable, we must go into advertising mode, ignoring the USB.
            if(force_ble_enabled)
            {
                //NRF_LOG_INFO("FORCE BLE ENABLED");
                conn_state = Connection_status::INIT_BT;
            }
            else
            {
                conn_state = Connection_status::CHECK_USB_CONN;
            }
        }
        break;

        case Connection_status::INIT_BT:
        {
            //NRF_LOG_INFO("INIT BT");
            //Force connnect again just in case it was set as a device and not a host
            _BleManager.init();

            _BleManager.setForceBle(false);


            comN2SideLeft.ble_enable();
            comN2SideRight.ble_enable();

            ble_denied = false; //We will restart the BLE denied flag when we connect to BT due to the press of the pairing key. This will allow the BLE to be enabled again if the Neuron is disconnected.

            conn_state = Connection_status::CHECK_USB_CONN;
        }
        break;

        case Connection_status::CHECK_USB_CONN:
        {
             //Small debouncer for the connection check.
            if (millis() - last_host_connection_check < HOST_CONNECTION_CHECK_INTERVAL)
            {
                return;
            }
            last_host_connection_check = millis();

            if (!check_usb_connection() && !ble_innited())
            {
                host_connected = false;
                conn_state = Connection_status::CHECK_WIRED_OR_WIRELESS;
            }
            else
            {
                host_connected = true;
                if (ble_connected() || ble_is_advertising_mode())
                {
                    // We need to check if the sides are connected to the Neuron. if the Ble is initialized and we disconnect both sides, neuron will get stuck
                    // until we reconnect one side and quit the ble advertising mode.
                    if(!isDefyLeftWired && !isDefyRightWired)
                    {
                        // reset neuron to get out of the ble advertising mode. And start the radio manager.
                        conn_state = Connection_status::RESET_NEURON;
                    }

                   ::LEDControl.set_mode(::LEDControl.get_mode_index());
                }
                if(_BleManager.get_pairing_key_press())
                {
                    conn_state = Connection_status::IDLE;
                    break;
                }
            }

            // host_connection_requested will be true if the KS has requested the host connection.
            if(prev_host_connected != host_connected || host_connection_requested == true)
            {
                //NRF_LOG_INFO("Host connection changed or requested");
                // This will override the previous state.
                conn_state = Connection_status::HOST_CONNECTED;
            }

            if(mode_led_requested == true && host_connected == true && !force_ble_enabled)
            {
                //NRF_LOG_INFO("MODE LED ASKED");
                mode_led_requested = false;
                ::LEDControl.set_mode(::LEDControl.get_mode_index()); //Send the mode to the KS.
            }
            else if(mode_led_requested == true && host_connected == false)
            {
                //NRF_LOG_INFO("MODE LED ASKED BUT HOST DISCONNECTED OR BLE ACTIVE");
                mode_led_requested = false;
            }

        }
        break;

        case Connection_status::HOST_CONNECTED:
        {
            if (host_connected == false)
            {
                //NRF_LOG_INFO("Host DISCONNECTED");
                last_host_connection_check = millis();
                conn_state = Connection_status::CHECK_WIRED_OR_WIRELESS;
            }
            else
            {
                //NRF_LOG_INFO("Host CONNECTED");

                // If the host is connected with USB AND we didn't initilialize the ble or we dont't have any side connected we need to initialize the radio manager.
                if(!radioInited && !ble_innited())
                {
                    kaleidoscope::plugin::RadioManager::init();

                    comN2SideLeft.rf_enable( &comRfPipeLeft );
                    comN2SideRight.rf_enable( &comRfPipeRight );
                }
                if (!ble_innited())
                {
                    ::LEDControl.set_mode(::LEDControl.get_mode_index());
                }
                conn_state = Connection_status::PAIRING_MODE_KEY_CHECK;
            }
            //Send the connected message to the KS.
            //NRF_LOG_INFO("Sending host connection Status");

            Communications.sendPacketHostConnection( );
            //**************************************
            prev_host_connected = host_connected;
            host_connection_requested = false;
        }
        break;

        case Connection_status::CHECK_WIRED_OR_WIRELESS:
        {
            //NRF_LOG_INFO("CHECK WIRED OR WIRELESS");

            if(FirmwareVersion.keyboard_is_wireless())
            {
                //NRF_LOG_INFO("CHECK USB TIMER");
                conn_state = Connection_status::CHECK_USB_TIMER;
            }
            else
            {
                //NRF_LOG_INFO("KEYBOARD IS WIRED ONLY");
                conn_state = Connection_status::CHECK_USB_CONN;
            }
        }
        break;

        case Connection_status::CHECK_USB_TIMER:
        {
            if( (millis() - usbConnectionTime > USB_CONNECTION_TIMEOUT  && !bleInitiated) || force_ble_enabled)
            {
                //NRF_LOG_INFO("+++++USB CONNECTION TIMEOUT++++");
                conn_state = Connection_status::CHECK_NEURON_CONNECTORS;
            }
            else
            {
                //NRF_LOG_INFO("USB CHECK TIMER FALSE");
                conn_state = Connection_status::CHECK_USB_CONN;
            }
        }
        break;

        case Connection_status::CHECK_NEURON_CONNECTORS:
        {
            //NRF_LOG_INFO("CHECK NEURON CONNECTORS");
            /*
            0 -> Side connected and powered from its battery or the other side's battery.
            1 o 2 -> Side connected and powered from the N2 while it is connected to the PC via USB.
            4 -> Side disconnected.
            */
            bool isWiredMode = isDefyLeftWired && isDefyRightWired;

            bool isRFMode = !isDefyLeftWired && !isDefyRightWired;

            if ( (bat_status_l == 1 || bat_status_l == 2 || bat_status_r == 1 || bat_status_r == 2) || isWiredMode || isRFMode)
            {
                //Both sides are connected or disconnected from Neuron. We will not start the BLE automatically.

                //NRF_LOG_DEBUG("BLE mode denied both sides connected");
                ble_denied = true;
                conn_state = Connection_status::PAIRING_MODE_KEY_CHECK;
            }

            if(!ble_denied)
            {
                //One sides are disconnected from Neuron. We will start the BLE automatically.

                //NRF_LOG_DEBUG("BLE mode allowed, one  sides disconnected from Neuron.");

                conn_state = Connection_status::INIT_BT;
            }
        }
        break;

        case Connection_status::PAIRING_MODE_KEY_CHECK:
        {
            if(_BleManager.get_pairing_key_press())
            {
                //NRF_LOG_INFO("Pairing key pressed");
                conn_state = Connection_status::RESET_NEURON;
            }
            else
            {
                conn_state = Connection_status::CHECK_USB_CONN;
            }
        }
        break;

        case Connection_status::RESET_NEURON:
        {
            NRF_LOG_INFO("RESET NEURON");
            //We will reset the Neuron to start the BLE.
            reset_mcu();
            //Just in case the reset takes too long we will go to the next state and wait.
            conn_state = Connection_status::IDLE;
        }
        break;

        case Connection_status::IDLE:
        {

            if(prev_host_connected != host_connected || host_connection_requested)
            {
                //The only way to exit this state is by a reset. That will happend when the user press the pairing key and then the ESC key.
                ::LEDControl.set_mode(::LEDControl.get_mode_index());

                  //Send the connected message to the KS.
                NRF_LOG_INFO("Sending host connection Status");

                Communications.sendPacketHostConnection( );
                //**************************************
                prev_host_connected = host_connected;
                host_connection_requested = false;
            }
            //Here the BLE_manager plugin will be initialized and send the corresponding ble led mode.
        }
        break;

        default:
            NRF_LOG_INFO("Unknown state in connection_state_machine");
        break;


    }
}

void Communications::run()
{
  WiredCommunications::run();
  RFGWCommunications::run();
  connection_state_machine();

  comN2SideLeft.run();
  comN2SideRight.run();
}

bool Communications::isWiredLeftAlive()
{
  return comN2SideLeft.spi_is_connected();
}

bool Communications::isWiredRightAlive()
{
  return comN2SideRight.spi_is_connected();
}

bool Communications::sendPacket(Packet packet)
{
    bool result = false;

    /* We route the packet both comN2Side modules. They will decide if they send the packet or not.
     * The result is true if any of the sides has processed the packet. */

    if( comN2SideLeft.sendPacket( packet ) == true )
    {
        result = true;
    }

    if( comN2SideRight.sendPacket( packet ) == true )
    {
        result = true;
    }

    return result;

  return true;
}

bool Communications::sendPacketHostConnection( void )
{
    Communications_protocol::Packet packet{};

    packet.header.command = Communications_protocol::HOST_CONNECTION;
    packet.header.size    = 4;
    // Send host connection status.
    packet.data[0]        = host_connected;
    packet.data[1]        = ble_innited();
    // We will decide if Keyscanner is allowed to go to sleep if we don't have the host connected. This will depend on the Neuron connection to the KS sides.
    auto const &keyScanner = kaleidoscope::Runtime.device().keyScanner();
    packet.data[2]        =  (keyScanner.leftSideWiredConnection() && keyScanner.rightSideWiredConnection());
    packet.data[3]       = false; // Shutdown LEDs. This will be true only for the WN. We dont want to show the disconnected LED effect.

    return sendPacket(packet);
}

class Communications Communications;


#endif
