
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


#include "dl_middleware.h"
#include "Communications.h"
#include "Communications_side.h"
#include "CRC_wrapper.h"

#define CONNECT_WAIT_TIMEOUT_MS     100     /* Setting this shorter than KS IS_ALIVE wait (300 ms). This is for the case when the KS was already in the Connected state
                                               and sends its own IS_ALIVE message in response for the Neuron one. This way the Neuron repeated IS_ALIVE will reach
                                               KS side before it retries on its side */


ComSide::ComSide( com_side_type_t side_type )
{
    /* Side definition */
    this->side_type = side_type;

    /* Communications */
    p_spiPort = nullptr;
    p_rfPipe = nullptr;
    p_com_model = nullptr;

    com_model_wired_init( side_type );
    com_model_rf_init( side_type );
    com_model_ble_init( side_type );

    /* Side state */
    mode = SIDE_MODE_UNKNOWN;
    state = SIDE_STATE_DISCONNECTED;

    /* Flags */
    wired_enabled = true;     /* In general, Wired is currently always enabled */
    rf_enabled = false;
    ble_enabled = false;

    reconnect_needed = false;
}

void ComSide::ble_enable()
{
    ASSERT_DYGMA( rf_enabled == false, "The RF and BLE should not be enabled at the same time." );

    ble_enabled = true;
    rf_enabled = false;

    /* Request the reconnection */
    reconnect_needed = true;
}

void ComSide::rf_enable( ComRfPipe * p_rfPipe )
{
    ASSERT_DYGMA( ble_enabled == false, "The RF and BLE should not be enabled at the same time." );

    /* Register the RF pipe */
    this->p_rfPipe = p_rfPipe;

    ble_enabled = false;
    rf_enabled = true;

    /* Request the reconnection */
    reconnect_needed = true;
}

void ComSide::com_model_wired_init( com_side_type_t side_type )
{
    bool result;

    ComModelWired::com_model_wired_config_t config;

    config.side_type = side_type;
    config.p_instance = this;
    config.event_cb = com_model_event_cb;

    result = com_model_wired.init( &config );

    ASSERT_DYGMA( result == true, "com_model_wired.init failed" );
}

void ComSide::com_model_rf_init( com_side_type_t side_type )
{
    bool result;

    ComModelRf::com_model_rf_config_t config;

    config.side_type = side_type;
    config.p_instance = this;
    config.event_cb = com_model_event_cb;

    result = com_model_rf.init( &config );

    ASSERT_DYGMA( result == true, "com_model_rf.init failed" );
}

void ComSide::com_model_ble_init( com_side_type_t side_type )
{
    bool result;

    ComModelBle::com_model_ble_config_t config;

    config.side_type = side_type;
    config.p_instance = this;
    config.event_cb = com_model_event_cb;

    result = com_model_ble.init( &config );

    ASSERT_DYGMA( result == true, "com_model_ble.init failed" );
}


void ComSide::spi_port_register( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

bool ComSide::spi_is_connected( void )
{
    if( p_spiPort == nullptr )
    {
        return false;
    }

    return p_spiPort->is_connected();
}

bool ComSide::rf_is_connected( void )
{
    if( p_rfPipe == nullptr )
    {
        return false;
    }

    return p_rfPipe->is_connected();
}

/******************************************************************/
/*                         Communications                         */
/******************************************************************/

inline void ComSide::com_wired_start( void )
{
    /* Assign the SPI port to the wired communication model */
    com_model_wired.spi_port_set( p_spiPort );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_wired.com_model_get();

    /* Set the communication mode and start the connection */
    mode = SIDE_MODE_WIRED;
    state_set( SIDE_STATE_CONNECTION_START );
}

inline void ComSide::com_rf_start( void )
{
    /* Register the RF pipe to the RF communication model */
    com_model_rf.rf_pipe_set( p_rfPipe );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_rf.com_model_get();

    /* Set the communication mode and start the connection */
    mode = SIDE_MODE_RF;
    state_set( SIDE_STATE_CONNECTION_START );
}

inline void ComSide::com_ble_start( void )
{
    /* Assign the SPI port to the ble communication model */
    com_model_ble.spi_port_set( p_spiPort );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_ble.com_model_get();

    /* Set the communication mode and start the connection */
    mode = SIDE_MODE_BLE;
    state_set( SIDE_STATE_CONNECTION_START );
}

void ComSide::com_model_event_process( ComModel::com_model_event_t event )
{
    switch( event )
    {
        case ComModel::COM_MODEL_EVENT_RECONNECT_NEEDED:

            reconnect_needed = true;

            break;

        default:

            ASSERT_DYGMA( false, "Unhandled COM Model event detected." );

            break;
    }
}

void ComSide::com_model_event_cb( void * p_instance, ComModel::com_model_event_t event )
{
    ComSide * p_comSide = ( ComSide *)p_instance;

    p_comSide->com_model_event_process( event );
}

bool ComSide::com_packet_send( Packet &packet )
{
    /* Prepare the message according to the communication model*/
    if( p_com_model->msg_out_prepare( packet ) == false )
    {
        return false;
    }

    /* Calculate the CRC */
    packet.header.crc = 0;
    packet.header.crc = crc8( packet.buf, sizeof(Header) + packet.header.size );

    /* Send the packet using the actual communication model */
    return p_com_model->send_packet( packet );
}

bool ComSide::com_packet_read( Packet &packet )
{
    uint8_t crc_packet;
    uint8_t crc_calc;

    /* Prepare the message according to the communication model*/
    if ( p_com_model->read_packet( packet ) == false )
    {
        return false;
    }

    /* Save the packet CRC */
    crc_packet = packet.header.crc;

    /* Calculate the CRC */
    packet.header.crc = 0;
    crc_calc = crc8( packet.buf, sizeof(Header) + packet.header.size );

    /* Check the CRC */
    if( crc_calc != crc_packet )
    {
        return false;
    }

    /* Packet is valid - return true */
    return true;
}

inline void ComSide::state_set( side_state_t state )
{
    this->state = state;
}

inline void ComSide::state_set_disconnected()
{
    p_com_model = nullptr;
    state_set( SIDE_STATE_DISCONNECTED );
}

inline void ComSide::state_set_reconnect()
{
    ASSERT_DYGMA( reconnect_needed == true, "Unexpected COM Side reconnect behavior." );

    /*
     * We just initialize the DISCONNECT process which will prepare the system for the new connection.
     * The next connection attempt will be started automatically based on the state of the system.
     */

    state_set( SIDE_STATE_DISCONNECT );
}

inline void ComSide::state_disconnected_process( void )
{
    if( spi_is_connected() == true )
    {
        if( ble_enabled == true )
        {
            com_ble_start();
        }
        else if ( wired_enabled == true )
        {
            com_wired_start();
        }
    }
    else if( rf_enabled == true && rf_is_connected() == true )
    {
        com_rf_start();
    }
}

inline void ComSide::state_connection_start_process( void )
{
    Packet packet_send{};

    /* Check whether the connection is still active */
    if( p_com_model->is_connected() == false )
    {
        state_set_disconnected();
        return;
    }

    /* Prepare the IS_ALIVE packet */
    packet_send.header.size    = 0;
    packet_send.header.command = IS_ALIVE;

    if( com_packet_send( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_packet_send failure at this point" )
        return;
    }

    /* Wait for the CONNECT message */
    timer_set_ms( &connection_timer, CONNECT_WAIT_TIMEOUT_MS );
    state_set( SIDE_STATE_CONNECTION_WAIT );
}

inline void ComSide::state_connection_wait_process( void )
{
    Packet packet_read{};
    Packet packet_send{};

    /* Check the timeout for receiving the CONNECT message */
    if( reconnect_needed == true )
    {
        state_set_reconnect( );
        return;
    }
    else if( timer_check( &connection_timer ) == true )
    {
        /* Re-iterate the handshake process */
        state_set( SIDE_STATE_CONNECTION_START );
        return;
    }

    /* Try to read the packet */
    if( com_packet_read( packet_read ) == false )
    {
        return;
    }

    /* We wait for the CONNECTED message only */
    if( packet_read.header.command != CONNECTED )
    {
        return;
    }

    /* Prepare the CONNECTED response message and send it */
    packet_send.header.size    = 0;
    //packet.header.device  = p.header.device;  /* The device is filled inside the send function */
    packet_send.header.command = CONNECTED;

    if( com_packet_send( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_packet_send failure at this point" )
        return;
    }

    /* Set the CONNECTED state */
    state_set( SIDE_STATE_CONNECTED );

    /* Process the incoming CONNECTED message to the system */
    Communications.callbacks.call( packet_read.header.command, packet_read );
}

inline void ComSide::state_connected_process( void )
{
    Packet packet_read{};

    /* Check whether the connection is still active */
    if( p_com_model->is_connected() == false )
    {
        state_set( SIDE_STATE_DISCONNECT );
        return;
    }
    else if( mode == SIDE_MODE_RF && spi_is_connected() == true )
    {
        /*
         * The Wired communication is prioritized. Hence we disconnect the RF mode now.
         */
        state_set( SIDE_STATE_DISCONNECT );
        return;
    }
    else if( reconnect_needed == true )
    {
        state_set_reconnect( );
        return;
    }

    /* Try to read the packet */
    if( com_packet_read( packet_read ) == false )
    {
        return;
    }

    /* Check the incoming packet */
    if( packet_read.header.command == IS_ALIVE )
    {
        /* The peer has restarted, restart the connection process */
        state_set( SIDE_STATE_CONNECTION_START );
        return;
    }

    /* Process the incoming packet to the system */
    Communications.callbacks.call( packet_read.header.command, packet_read );
}

inline void ComSide::state_disconnect_process( void )
{
    Packet packet{};

    /* Prepare the DISCONNECTED message */
    packet.header.command = Communications_protocol::DISCONNECTED;
    packet.header.device  = p_com_model->dev_side_get();

    /* Perform the Com model-specific disconnect */
    p_com_model->disconnect();

    /* Remove the SPI port */
    if( mode == SIDE_MODE_WIRED || mode == SIDE_MODE_BLE )
    {
        /* We nullify the spiPort only for the SPI-related modes. An existence of a valid and connected SPI port is a reason
         * for switching from RF mode to the Wired one. Thus we must keep the SPI port available in such case. */
        p_spiPort = nullptr;
    }

    /* Remove the communication model */
    p_com_model = nullptr;

    /* Remove possible reconnection request flag */
    reconnect_needed = false;

    /* Move to the DISCONNECTED state */
    mode = SIDE_MODE_UNKNOWN;
    state_set( SIDE_STATE_DISCONNECTED );

    /* Inform the rest of the system about the DISCONNECTION event */
    Communications.callbacks.call(packet.header.command, packet);
}

inline void ComSide::machine( void )
{
    switch( state )
    {
        case SIDE_STATE_DISCONNECTED:

            state_disconnected_process();

            break;

        case SIDE_STATE_CONNECTION_START:

            state_connection_start_process();

            break;

        case SIDE_STATE_CONNECTION_WAIT:

            state_connection_wait_process();

            break;

        case SIDE_STATE_CONNECTED:

            state_connected_process();

            break;

        case SIDE_STATE_DISCONNECT:

            state_disconnect_process();

            break;

        default:

            ASSERT_DYGMA( false, "Invalid Side Communication state." );

            break;
    }
}

void ComSide::run( void )
{
    machine();
}

bool ComSide::sendPacket( Packet &packet )
{
    /*
     * We make a copy of the packet here. The reason is the packet gets modified along the communication models while its
     * source needs to be preserved intact for use outside of this function
     */
    Packet packet_send = packet;

    /* Check if it is possible to send the packet */
    if( state != SIDE_STATE_CONNECTED )
    {
        return false;
    }

    /* Send the packet using the actual communication model */
    return com_packet_send( packet_send );
}
