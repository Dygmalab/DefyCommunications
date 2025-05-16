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
#include "CommunicationsWN_side.h"
#include "SpiPort.h"
#include "kaleidoscope/plugin/LEDControl.h"
#include <kaleidoscope.h>


#define DEBUG_LOG_N2_COMMUNICATIONS 0

#define PORT_IS_ALIVE_TIMEOUT_MS    2000

#define HOST_CONNECTION_CHECK_INTERVAL  250

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
    ComWNSide * p_comWNSide;
} com_spiPort_t;

static SpiPort spiPort1(0);
static SpiPort spiPort2(1);
static ComWNSide comWNSideLeft( COM_SIDE_TYPE_KS_LEFT );
static ComWNSide comWNSideRight( COM_SIDE_TYPE_KS_RIGHT );

static com_spiPort_t com_spiPort1 = { .state = COM_SPIPORT_STATE_DISCONNECTED, .p_spiPort = &spiPort1, .p_comWNSide = NULL };
static com_spiPort_t com_spiPort2 = { .state = COM_SPIPORT_STATE_DISCONNECTED, .p_spiPort = &spiPort2, .p_comWNSide = NULL };

static bool host_connection_requested = false;

enum class Connection_status
{
    SET_USB_TIMER,
    CHECK_USB_CONN,
    CHECK_WIRED_OR_WIRELESS,
    HOST_CONNECTED,
}conn_state = Connection_status::SET_USB_TIMER;

bool mode_led_requested = false;

//HOST CONNECTION
static bool host_connected = false;


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

void new_connection_handle(void)
{
    /*
     * Report new connection here
     */
}

/****************************************************************** */

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

    static void _com_spiPort_state_connected_set( com_spiPort_t * p_com_spiPort, ComWNSide * p_comWNSide )
    {
        p_com_spiPort->p_comWNSide = p_comWNSide;
        p_com_spiPort->p_comWNSide->spi_port_register( p_com_spiPort->p_spiPort );

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

                /* Set the SPI COM left connectivity */
                _com_spiPort_state_connected_set( p_com_spiPort, &comWNSideLeft );

                break;

            case KEYSCANNER_DEFY_RIGHT:

                /* Set the SPI COM right connectivity */
                _com_spiPort_state_connected_set( p_com_spiPort, &comWNSideRight );

                break;

            default:

                ASSERT_DYGMA( false, "Unexpected SPI packet Device type" );

                break;
        }
    }

    static void _com_spiPort_state_connected_process( com_spiPort_t * p_com_spiPort )
    {
        /* Check whether the Side is still connected over wire */
        if( p_com_spiPort->p_comWNSide->spi_is_connected() == true )
        {
            return;
        }

        /* The side is disconnected over wire */

        p_com_spiPort->p_comWNSide = nullptr;
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

void Communications::init() {

  callbacks.bind(CONNECTED, [this](Packet p) {
      /* Send the Host connection info */
      sendPacketHostConnection( );
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
    return host_connected;
}

void connection_state_machine ()
{
    //HOST CONNECTION
    static bool prev_host_connected = false;
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
            Communications.sendPacketHostConnection( );

            prev_host_connected = host_connected;
            host_connection_requested = false;

            conn_state = Connection_status::CHECK_USB_CONN;
        }
        break;

        default:
        break;


    }
}

void Communications::run() {

  WiredCommunications::run();
  connection_state_machine();

  comWNSideLeft.run();
  comWNSideRight.run();
}

bool Communications::isWiredLeftAlive() {
    return comWNSideLeft.spi_is_connected();
}

bool Communications::isWiredRightAlive() {
    return comWNSideRight.spi_is_connected();
}

bool Communications::sendPacket(Packet packet)
{
    bool result = false;

    /* We route the packet both comWNSide modules. They will decide if they send the packet or not.
     * The result is true if any of the sides has processed the packet. */

    if( comWNSideLeft.sendPacket( packet ) == true )
    {
        result = true;
    }

    if( comWNSideRight.sendPacket( packet ) == true )
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
    packet.header.size    = 2;
    packet.data[0]        = host_connected;
    packet.data[1]        = false;

    return sendPacket(packet);
}

class Communications Communications;

#endif

