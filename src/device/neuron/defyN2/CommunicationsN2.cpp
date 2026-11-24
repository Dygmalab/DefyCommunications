/*
 * Copyright (C) 2026  Dygma Lab S.L.
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


#include "Adafruit_USBD_Device.h"
#include "Arduino.h"
#include "Ble_manager.h"
#include "Communications.h"
#include "CommunicationsN2_side.h"
#include "FirmwareVersion.h"
#include "kbd_if.h"
#include "kbd_if_manager.h"
#include "Kbd_manager.h"
#include "LEDManager.h"
#include "SpiPort.h"
#include "Radio_manager.h"
#include "Time_counter.h"

//#define DEBUG_LOG_N2_COMMUNICATIONS     0

#define USB_CONNECTION_TIMEOUT_MS       500
#define USB_RECONNECTION_TIMEOUT_MS     1000

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

enum class Connection_status
{
    STATE_IDLE = 1,
    STATE_CONNECTION_MODE_WAIT,

    STATE_USB_CONNECTION_START,
    STATE_USB_CONNECTED,
    STATE_USB_RECONNECTION_WAIT,

    STATE_BLE_ENABLE,
    STATE_BLE_ENABLE_WAIT,
    STATE_BLE_CONNECTION_WAIT,
    STATE_BLE_CONNECTED,
    STATE_BLE_FAILED,
    STATE_BLE_REENABLE,

    STATE_RESET_NEEDED,

}conn_state;

enum class Connection_mode
{
    CONN_MODE_UNKNOWN = 1,
    CONN_MODE_USB,
    CONN_MODE_BLE,
}conn_mode;


/* Timer */
static dl_timer_t timer_usb_connection;

/* HOST CONNECTION STATUS */
static bool host_connected = false;

/* Forced BLE */
static bool forced_ble_flag = false;

/* Keyboard interface */
static kbdif_t * p_kbdif;

//void checkActive();
INLINE void _state_set( Connection_status connection_status );
INLINE void _state_connection_mode_wait_set( void );
static INLINE result_t kbdif_initialize( void );

void new_connection_handle(void)
{
    LEDManager.com_new_connection_set();
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
    if (!RadioManager.isInited()) return;
    RadioManager.poll();
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

        mcu_sleep_postpone();
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
  //NRF_LOG_DEBUG("Sending configuration command to KS (broadcast)");
  Communications_protocol::Packet p{};
  p.header.size = 1;
  p.header.command = Communications_protocol::CONFIGURATION;
  sendPacket(p);
}


void Communications::init()
{
  result_t result = RESULT_ERR;

  callbacks.bind(CONNECTED, [this](Packet p) {

#if DEBUG_LOG_N2_COMMUNICATIONS
    NRF_LOG_INFO("Get connected from %i", p.header.device);
#endif

    get_keyscanner_configuration();

    /* Send the Host connection info */
    sendPacketHostConnection( );

    if( conn_state == Connection_status::STATE_BLE_FAILED )
    {
        /* Reset the BLE connection process */
        _state_set( Connection_status::STATE_BLE_REENABLE );
    }
  });

  callbacks.bind(HOST_CONNECTION_STATUS, [this](Packet p)
  {
      //Keyscanner will ask for the host connection.
      //NRF_LOG_INFO("HOST CONNECTION ASKED");
      sendPacketHostConnection( );
  });

  callbacks.bind(MODE_LED, [this](Packet p)
  {
      //NRF_LOG_INFO("MODE LED ASKED");
      if( host_connected == true )
      {
          /* Refreshing only if the host is connected */
          LEDManager.led_effect_refresh(); //Send the mode to the KS.
      }
  });

  WiredCommunications::init();
  RFGWCommunications::init();

  /* Initialize the keyboard interface */
  result = kbdif_initialize( );
  ASSERT_DYGMA( result == RESULT_OK, "kbdif_initialize failed" );

  /* The connection mode is unknown initially */
  conn_mode = Connection_mode::CONN_MODE_UNKNOWN;

  /* Start with searching for the connection mode */
  _state_connection_mode_wait_set();

  UNUSED( result );
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

bool usb_check_connection()
{
    return  tud_ready();
}

bool Communications::is_host_connected()
{
    /*
     * Using the BleManager.is_enabled here is a hack to allow the bond pin to be entered"
     */
    return host_connected || BleManager.is_enabled();
}

INLINE void _host_connected_set( bool status )
{
    /* Set the new host_connected status */
    host_connected = status;
    Communications.sendPacketHostConnection( );
}

INLINE bool _mode_ble_resolve( void )
{
//    uint8_t bat_status_l;
//    uint8_t bat_status_r;
    bool isDygmaLeftWired;
    bool isDygmaRightWired;
    bool isWiredMode;
    bool isRFMode;

/*
 * NOTE: The Forced BLE is now set when the actual connection mode is not known yet and thus can trigger the Forced BLE
 *       mode without storing the flag in the BleManager memory and restarting the system. After the approach is approved,
 *       remove the commented force_ble_get condition and clean the code.
 */

    /* Check if the BLE is being forced */
//    if( BleManager.force_ble_get() == true )
//    {
//        return true;
//    }

    /* Check if the keyboard is wireless */
    if( FirmwareVersion.keyboard_is_wireless() == false )
    {
        return false;
    }

    /* Check if the USB is connected */
    if( usb_check_connection() == true )
    {
        return false;
    }

    /* Check if the BLE is being forced */
    if( forced_ble_flag == true )
    {
        return true;
    }

    /* Check if the USB connection timer has expired */
    if( timer_check( &timer_usb_connection ) == false )
    {
        return false;
    }

//    /* Check the cable connections
//     *
//     * Battery status explanation:
//     * ---------------------------
//     * 0 -> Side connected and powered from its battery or the other side's battery.
//     * 1 o 2 -> Side connected and powered from the N2 while it is connected to the PC via USB.
//     * 4 -> Side disconnected.
//     */
//
//    bat_status_l = Battery::get_battery_status_left();
//    bat_status_r = Battery::get_battery_status_right();

    isDygmaLeftWired = kbdManager.leftSideWiredConnection();
    isDygmaRightWired = kbdManager.rightSideWiredConnection();

    /*
     * These are special configurations to prevent keyboard going to the BLE advertising when computer goes to sleep.
     * The premise is that in Wired keyboard use the people have both keyscanners usually connected, in RF mode both
     * keyscanners are usually disconnected.
     */
    isWiredMode = isDygmaLeftWired && isDygmaRightWired;
    isRFMode = !isDygmaLeftWired && !isDygmaRightWired;

    /*
     * NOTE: We do not evaluate battery status any more and check the connections only with the state of the physical wired
     *       connection status. After the approach is approved, remove the commented battery condition and clean the code.
     */
//    if ( (bat_status_l == 1 || bat_status_l == 2 || bat_status_r == 1 || bat_status_r == 2) || isWiredMode || isRFMode)
    if ( isWiredMode || isRFMode)
    {
        //Both sides are connected or disconnected from Neuron. We will not start the BLE automatically.

        return false;
    }

    /* The bluetooth mode is resolved */
    return true;
}

INLINE bool _mode_usb_resolve( void )
{
    /* Simply check if the USB is connected */
    return usb_check_connection();
}

INLINE void _state_set( Connection_status connection_status )
{
    conn_state = connection_status;
    mcu_sleep_postpone();
}

INLINE void _state_connection_mode_wait_set( void )
{
    timer_set_ms( &timer_usb_connection, USB_CONNECTION_TIMEOUT_MS );
    _state_set( Connection_status::STATE_CONNECTION_MODE_WAIT );
}

INLINE void _state_usb_connection_start_set( void )
{
    /* Set the USB connection mode */
    conn_mode = Connection_mode::CONN_MODE_USB;

    _state_set( Connection_status::STATE_USB_CONNECTION_START );
}

INLINE void _state_usb_connected_set( void )
{
    _host_connected_set( true );
    _state_set( Connection_status::STATE_USB_CONNECTED );
}

INLINE void _state_usb_reconnection_wait_set( void )
{
    /*
     * We do not signal the host disconnection right away. Giving time to refresh
     */
    //_host_connected_set( false );
    timer_set_ms( &timer_usb_connection, USB_RECONNECTION_TIMEOUT_MS );
    _state_set( Connection_status::STATE_USB_RECONNECTION_WAIT );
}

INLINE void _state_ble_enable_set( void )
{
    /* Set the BLE connection mode */
    conn_mode = Connection_mode::CONN_MODE_BLE;

    _state_set( Connection_status::STATE_BLE_ENABLE );
}

INLINE void _state_ble_connection_wait_set( void )
{
    /*
     * This is HACK: in BLE mode, we fake the "host connected" state to prevent keyscanner going to sleep after its internal fixed timeout (40 seconds)
     *               we should eventually fix the logic of this both on Neuron and Keyscanner side.
     */
    _host_connected_set( true );
    _state_set( Connection_status::STATE_BLE_CONNECTION_WAIT );
}

INLINE void _state_ble_connected_set( void )
{
    _host_connected_set( true );
    _state_set( Connection_status::STATE_BLE_CONNECTED );
}

INLINE void _state_connection_mode_wait( void )
{
    if( _mode_ble_resolve() == true )
    {
        _state_ble_enable_set();
    }
    else if( _mode_usb_resolve() == true )
    {
        _state_usb_connection_start_set();
    }
}

INLINE void _state_usb_connection_start( void )
{
    /* Enable the RF communication channel */
    RadioManager.enable();

    comN2SideLeft.rf_enable( &comRfPipeLeft );
    comN2SideRight.rf_enable( &comRfPipeRight );

    /* Just move to the Connected state, the USB is started elsewhere by default */
    _state_usb_connected_set( );
}

INLINE void _state_usb_connected( void )
{
    if ( _mode_usb_resolve() == false )
    {
        /* The conditions for the USB mode are not valid any more. Giving chance for reconnection */
        _state_usb_reconnection_wait_set( );
        return;
    }
}

INLINE void _state_usb_reconnection_wait( void )
{
    if ( _mode_usb_resolve() == true )
    {
        /* The conditions for the USB mode have refreshed. Moving back to the Connected state */
        _state_usb_connected_set( );
        return;
    }
    else if( timer_check( &timer_usb_connection ) == false )
    {
        return;
    }

    /* The reconnection timeout has expired. We need to reset the device to let the connection mode re-evaluate */
    _state_set( Connection_status::STATE_RESET_NEEDED );
}

INLINE void _state_ble_enable( void )
{
    /* Disable the USB */
    usb_disable();

    /* Enable the BLE */
    BleManager.enable();
    BleManager.force_ble_set(false);

    comN2SideLeft.ble_enable();
    comN2SideRight.ble_enable();

    /* Wait for the BLE Host connection */
    _state_set( Connection_status::STATE_BLE_ENABLE_WAIT );
}

INLINE void _state_ble_enable_wait( void )
{
    if( BleManager.is_enabled() == false )
    {
        return;
    }

    /* Wait for the BLE Host connection */
    _state_ble_connection_wait_set();
}

INLINE void _state_ble_connection_wait( void )
{
    if( _mode_ble_resolve() == false )
    {
        /* The conditions for the BLE mode are not valid any more. We need to reset the device to let the connection mode re-evaluate */
        _state_set( Connection_status::STATE_RESET_NEEDED );
        return;
    }
    else if ( BleManager.is_enabled() == false )
    {
        /* This may happen if the BLE is either deliberately disabled from elsewhere or if its advertising failed in its search for a peer to connect to.
         * Move to the BLE Fail state and wait CONNECTED message from the keyscanner. */
        _state_set( Connection_status::STATE_BLE_FAILED );
        return;
    }
    else if( BleManager.is_connected() == false )
    {
        return;
    }

    _state_ble_connected_set( );
}

INLINE void _state_ble_connected( void )
{
    if( _mode_ble_resolve() == false )
    {
        /* The conditions for the BLE mode are not valid any more. We need to reset the device to let the connection mode re-evaluate */
        _state_set( Connection_status::STATE_RESET_NEEDED );
        return;
    }
    else if( BleManager.is_connected() == false )
    {
        /* Go back to the BLE Connection wait */
        _state_ble_connection_wait_set();

        return;
    }
}

INLINE void _state_ble_reenable( void )
{
    /* Re-enable the leds */
    LEDManager.leds_enable();

    /* Enable the BLE */
    BleManager.enable();

    /* Wait for the BLE Host connection */
    _state_set( Connection_status::STATE_BLE_ENABLE_WAIT );
}

INLINE void _state_reset_needed( void )
{
    /* Restart the device */
    reset_mcu();
}

void connection_state_machine ()
{
    switch( conn_state )
    {
        case Connection_status::STATE_CONNECTION_MODE_WAIT:

            _state_connection_mode_wait();

            break;

        case Connection_status::STATE_USB_CONNECTION_START:

            _state_usb_connection_start();

            break;

        case Connection_status::STATE_USB_CONNECTED:

            _state_usb_connected();

            break;

        case Connection_status::STATE_USB_RECONNECTION_WAIT:

            _state_usb_reconnection_wait();

            break;

        case Connection_status::STATE_BLE_ENABLE:

            _state_ble_enable();

            break;

        case Connection_status::STATE_BLE_ENABLE_WAIT:

            _state_ble_enable_wait();

            break;

        case Connection_status::STATE_BLE_CONNECTION_WAIT:

            _state_ble_connection_wait();

            break;

        case Connection_status::STATE_BLE_CONNECTED:

            _state_ble_connected();

            break;

        case Connection_status::STATE_BLE_FAILED:

            /* Waiting for the next keypress or CONNECTED message from the keyscanner */

            break;

        case Connection_status::STATE_BLE_REENABLE:

            _state_ble_reenable();

            break;

        case Connection_status::STATE_RESET_NEEDED:

            _state_reset_needed();

            break;

        default:

            ASSERT_DYGMA( false, "Unhandled communication connection status" );

            break;
    }
}

/****************************************************/
/*                Keyboard Interface                */
/****************************************************/

static INLINE kbdapi_event_result_t kbdif_key_event_mode_unknown_process( kbdapi_key_t * p_key )
{
    /* Accepting the Bluetooth pairing key when it is toggled on. */
    if ( p_key->type != KBDAPI_KEY_TYPE_BLUETOOTH_PAIRING || p_key->toggled_on == false )
    {
        return KBDAPI_EVENT_RESULT_IGNORED;  /* The key is ignored */
    }

    /* Check if the keyboard is wireless */
    if( FirmwareVersion.keyboard_is_wireless() == false )
    {
        /* The keyboard is not wireless - The BLE is not allowed */
        return KBDAPI_EVENT_RESULT_IGNORED;  /* The key is ignored */
    }

    ASSERT_DYGMA( conn_state == Connection_status::STATE_CONNECTION_MODE_WAIT, "Unexpected connection state while in the Unknown connection mode" );

    /* The Bluetooth pairing key was pressed while in the Unknown connection mode. Set the Forced BLE flag and trigger the BLE mode. */
    forced_ble_flag = true;

    _state_ble_enable_set();

//    /* The Bluetooth pairing key was pressed while in the Unknown connection mode. Prepare the Forced BLE mode and restart system. */
//    BleManager.force_ble_set( true );
//
//    _state_set( Connection_status::STATE_BLE_FORCE_SYSTEM_RESET );

    return KBDAPI_EVENT_RESULT_CONSUMED;
}

static INLINE kbdapi_event_result_t kbdif_key_event_mode_ble_process( kbdapi_key_t * p_key )
{
    if( conn_state == Connection_status::STATE_BLE_FAILED )
    {
        /* Reset the BLE connection process */
        _state_set( Connection_status::STATE_BLE_REENABLE );
    }

    return KBDAPI_EVENT_RESULT_IGNORED;
}

static kbdapi_event_result_t kbdif_key_event_callback( void * p_instance, kbdapi_key_t * p_key )
{
    kbdapi_event_result_t kbdapi_event_result = KBDAPI_EVENT_RESULT_IGNORED;

    ASSERT_DYGMA( p_instance == NULL, "Unexpected kbdif_key_event_callback p_instance - should be NULL" );

    switch( conn_mode )
    {
        case Connection_mode::CONN_MODE_UNKNOWN:

            kbdapi_event_result = kbdif_key_event_mode_unknown_process( p_key );

            break;

        case Connection_mode::CONN_MODE_USB:

            /* Currently no key action in the USB mode */

            break;

        case Connection_mode::CONN_MODE_BLE:

            kbdapi_event_result = kbdif_key_event_mode_ble_process( p_key );

            break;

        default:

            kbdapi_event_result = KBDAPI_EVENT_RESULT_IGNORED;

            break;
    }

    return kbdapi_event_result;
}

static const kbdif_handlers_t kbdif_handlers =
{
    .key_event_cb = kbdif_key_event_callback,
    .command_event_cb = NULL,
};

static INLINE result_t kbdif_initialize( void )
{
    result_t result = RESULT_ERR;
    kbdif_conf_t config;

    /* Prepare the kbdif configuration */
    config.p_instance = NULL;
    config.handlers = &kbdif_handlers;

    /* Initialize the kbdif */
    result = kbdif_init( &p_kbdif, &config );
    EXIT_IF_ERR( result, "kbdif_init failed" );

    /* Add the kbdif into the kbdif manager */
    result = kbdifmgr_add( p_kbdif );
    EXIT_IF_ERR( result, "kbdifmgr_add failed" );

_EXIT:
    return result;
}

/****************************************************/
/*                       API                        */
/****************************************************/

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
}

bool Communications::sendPacketHostConnection( void )
{
    Communications_protocol::Packet packet{};

    packet.header.command = Communications_protocol::HOST_CONNECTION;
    packet.header.size    = 4;
    // Send host connection status.
    packet.data[0]        = host_connected;
    packet.data[1]        = BleManager.is_enabled();
    // We will decide if Keyscanner is allowed to go to sleep if we don't have the host connected. This will depend on the Neuron connection to the KS sides.
    packet.data[2]        =  !kbdManager.leftSideWiredConnection() || !kbdManager.rightSideWiredConnection();   // Sleep is possible if at least one of the sides is not wire-connected
    packet.data[3]       = false; // Shutdown LEDs. This will be true only for the WN. We dont want to show the disconnected LED effect.

    return sendPacket(packet);
}

class Communications Communications;
