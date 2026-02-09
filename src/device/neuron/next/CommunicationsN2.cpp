/*
 * Copyright (C) 2025  Dygma Lab S.L.
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
 */

#include "Communications.h"
#include "CommunicationsN2_side.h"
#include "SpiPort.h"
#include "CRC_wrapper.h"
#include "Arduino.h"
#include "Adafruit_USBD_Device.h"
#include "Kaleidoscope-LEDControl.h"
#include "Battery.h"
#include "Ble_manager.h"
#include "FirmwareVersion.h"
#include "LEDManager.h"


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

static SpiPort spiPort(1);
static ComN2Side comN2Side( COM_SIDE_TYPE_KS_LEFT );

static com_spiPort_t com_spiPort = { .state = COM_SPIPORT_STATE_DISCONNECTED, .p_spiPort = &spiPort, .p_comN2Side = NULL };

enum class Connection_status
{
    STATE_IDLE = 1,
    STATE_CONNECTION_MODE_WAIT,

    STATE_USB_CONNECTION_START,
    STATE_USB_CONNECTION_WAIT,
    STATE_USB_CONNECTED,

    STATE_BLE_CONNECTION_START,
    STATE_BLE_CONNECTION_WAIT,
    STATE_BLE_CONNECTED,

}conn_state = Connection_status::STATE_CONNECTION_MODE_WAIT;

//HOST CONNECTION STATUS
static bool host_connected = false;

void checkActive();

void new_connection_handle(void)
{
    LEDManager.com_new_connection_set();
}

class WiredCommunications
{
public:
    static void init()
    {
        spiPort.init( );
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
                _com_spiPort_state_connected_set( p_com_spiPort, &comN2Side );

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
        com_spiPort_machine( &com_spiPort );
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
      //Keyscanner requests the host connection status.
      //NRF_LOG_INFO("HOST CONNECTION ASKED");
      sendPacketHostConnection( );
  });

  callbacks.bind(MODE_LED, [this](Packet p)
  {
      NRF_LOG_INFO("MODE LED ASKED");
      if( host_connected == true )
      {
          /* Refreshing only if the host is connected */
          LEDManager.led_effect_refresh(); //Send the mode to the KS.
      }
  });

  WiredCommunications::init();
}

void usb_disable( void )
{
    TinyUSBDevice.detach();
#ifdef USE_TINYUSB
    TinyUSB_Device_FlushCDC();
#endif
//    if(!Serial) Serial.begin(115200);
//    RawHID.flush();
}

/*
 * This function is called when the USB is connected or disconnected.
 * For the moment it only logs the event.
 * We will use it to decide if we send the disconnect message to the host. Or not.
 */
bool usb_check_connection()
{
    return  tud_ready();
}

bool Communications::is_host_connected()
{
    return host_connected || ble_connected();
}

void INLINE _host_connected_set( bool status )
{
    /* Set the new host_connected status */
    host_connected = status;
    Communications.sendPacketHostConnection( );
}

void INLINE _state_connection_mode_wait( void )
{
    auto const &keyScanner = kaleidoscope::Runtime.device().keyScanner();

    if( keyScanner.slideSwitchPositionUsb() == true )
    {
        conn_state = Connection_status::STATE_USB_CONNECTION_START;
    }
    else if( keyScanner.slideSwitchPositionBle() == true /*&& FirmwareVersion::keyboard_is_wireless() == true*/ )
    {
        conn_state = Connection_status::STATE_BLE_CONNECTION_START;
    }
}

void INLINE _state_usb_connection_start( void )
{
    /* Just move to the next state, the USB is started elsewhere by default */

    conn_state = Connection_status::STATE_USB_CONNECTION_WAIT;
}

void INLINE _state_usb_connection_wait( void )
{
    if ( usb_check_connection() == false )
    {
        return;
    }

    _host_connected_set( true );
    conn_state = Connection_status::STATE_USB_CONNECTED;
}

void INLINE _state_usb_connected( void )
{
#warning "Check the effect of short host connection lost"
    /* Check the USB connection is still active */
    if ( usb_check_connection() == false )
    {
        /* Go back to the USB Connection wait */
        _host_connected_set( false );
        conn_state = Connection_status::STATE_USB_CONNECTION_WAIT;
        return;
    }
}

void INLINE _state_ble_connection_start( void )
{
    /* Disable the USB */
    usb_disable();

    /* Enable the BLE */
    BleManager.enable();
    BleManager.setForceBle(false);

    comN2Side.ble_enable();

    /* Wait for the BLE Host connection */
    conn_state = Connection_status::STATE_BLE_CONNECTION_WAIT;
}

void INLINE _state_ble_connection_wait()
{
    if( ble_innited() == false )
    {
        return;
    }

    _host_connected_set( true );
    conn_state = Connection_status::STATE_BLE_CONNECTED;
}

void INLINE _state_ble_connected()
{
    if( ble_innited() == false )
    {
        /* Go back to the BLE Connection wait */
        _host_connected_set( false );
        conn_state = Connection_status::STATE_USB_CONNECTION_WAIT;
        return;
    }
}

void connection_state_machine( void )
{
    switch (conn_state)
    {
        case Connection_status::STATE_CONNECTION_MODE_WAIT:
        {
            _state_connection_mode_wait();
        }
        break;

        case Connection_status::STATE_USB_CONNECTION_START:
        {
            _state_usb_connection_start();
        }
        break;

        case Connection_status::STATE_USB_CONNECTION_WAIT:
        {
            _state_usb_connection_wait();
        }
        break;

        case Connection_status::STATE_USB_CONNECTED:
        {
            _state_usb_connected();
        }
        break;

        case Connection_status::STATE_BLE_CONNECTION_START:
        {
            _state_ble_connection_start();
        }
        break;

        case Connection_status::STATE_BLE_CONNECTION_WAIT:
        {
            _state_ble_connection_wait();
        }
        break;

        case Connection_status::STATE_BLE_CONNECTED:
        {
            _state_ble_connected();
        }
        break;

        default:
        {
            ASSERT_DYGMA( false, "Unhandled communication connection status" );
        }
        break;
    }
}

void Communications::run()
{
  WiredCommunications::run();
  connection_state_machine();

  comN2Side.run();
}

bool Communications::isWiredLeftAlive()
{
  return comN2Side.spi_is_connected();
}

bool Communications::isWiredRightAlive()
{
#warning "This needs to be considered yet. Currently we return true, stating that non-existent right side is alive over wire (SPI)"
//  return comN2SideRight.spi_is_connected();
  return true;
}

bool Communications::sendPacket(Packet packet)
{
    bool result = false;

    /* We route the packet both comN2Side modules. They will decide if they send the packet or not.
     * The result is true if any of the sides has processed the packet. */

    if( comN2Side.sendPacket( packet ) == true )
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
