
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

#define CONNECT_WAIT_TIMEOUT_MS     100     /* Setting this shorter than KS IS_ALIVE wait (300 ms). This is for the case when the KS was already in the Connected state
                                               and sends its own IS_ALIVE message in response for the Neuron one. This way the Neuron repeated IS_ALIVE will reach
                                               KS side before it retries on its side */


ComSide::ComSide( com_side_type_t side_type )
{
    /* Side definition */
    this->side_type = side_type;

    /* Communications */
    p_spiPort = nullptr;
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
#warning "Needs to be implemented yet"

    com_model_rf.init( side_type );
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

bool ComSide::wired_is_connected( void )
{
    if( p_spiPort == nullptr )
    {
        return false;
    }

    return p_spiPort->is_connected();
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
    if( wired_is_connected() == true )
    {
        if( ble_enabled == true )
        {
            ASSERT_DYGMA( false, "Not implemented yet" )
        }
        else if ( wired_enabled == true )
        {
            com_wired_start();
        }
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

    if( p_com_model->send_packet( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_send_packet failure at this point" )
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
    if( p_com_model->read_packet( packet_read ) == false )
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

    if( p_com_model->send_packet( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_send_packet failure at this point" )
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
    else if( reconnect_needed == true )
    {
        state_set_reconnect( );
        return;
    }

    /* Try to read the packet */
    if( p_com_model->read_packet( packet_read ) == false )
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
    p_spiPort = nullptr;

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
    return p_com_model->send_packet( packet_send );
}
