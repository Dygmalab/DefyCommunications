
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

bool ComSide::init( void )
{
    /* Communications */
    p_com_model = nullptr;

    /* Side state */
    state = SIDE_STATE_DISCONNECTED;

    /* Flags */
    disconnect_request = false;
    reconnect_request = false;

    return true;
}

/******************************************************************/
/*                         Communications                         */
/******************************************************************/

void ComSide::com_model_event_process( ComModel::com_model_event_t event )
{
    switch( event )
    {
        case ComModel::COM_MODEL_EVENT_RECONNECT_NEEDED:

            reconnect_request = true;

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

inline bool ComSide::com_model_is_connected( void )
{
    if( p_com_model == nullptr )
    {
        return false;
    }

    return p_com_model->is_connected();
}

inline void ComSide::com_model_start( void )
{
    ComModel::com_model_event_cb_config_t event_cb_config;

    /* Register the communication model event callback */
    event_cb_config.p_instance = this;
    event_cb_config.event_cb = com_model_event_cb;

    p_com_model->event_cb_config( &event_cb_config );

    /* Set the communication mode and start the connection */
    state_set( SIDE_STATE_CONNECTION_START );
}

inline bool ComSide::disconnect_needed( void )
{
    /*
     * In this module when any of these conditions are met, the module should move to the disconnected state.
     * Possible reconnection needs to be triggered via the "connect" API function call outside of this module.
     */

    if ( disconnect_request == true || reconnect_request == true || com_model_is_connected( ) == false )
    {
        return true;
    }
    else
    {
        return false;
    }
}

/******************************************************************/
/*                         State machine                          */
/******************************************************************/

inline void ComSide::state_set( side_state_t state )
{
    this->state = state;
}

inline void ComSide::state_set_disconnected( void )
{
    /* Perform the Com model-specific disconnect */
    p_com_model->disconnect();

    /* Remove the communication model */
    p_com_model = nullptr;
    state_set( SIDE_STATE_DISCONNECTED );

    /* Clear the disconnect request flag */
    disconnect_request = false;
    reconnect_request = false;
}

inline void ComSide::state_disconnected_process( void )
{
    /*
     * Waiting for the API connect function call
     */
}

inline void ComSide::state_connection_start_process( void )
{
    Packet packet_send{};

    /* Check whether the connection is still active */
    if( disconnect_needed() == true )
    {
        /*
         * We go straight to the DISCONNECTED state as the CONNECT has not been reported to the system before
         */
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

    /* Check whether the connection is still active */
    if( disconnect_needed() == true )
    {
        /*
         * We go straight to the DISCONNECTED state as the CONNECT has not been reported to the system before
         */
        state_set_disconnected();
        return;
    }
    /* Check the timeout for receiving the CONNECT message */
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
    if( disconnect_needed() == true )
    {
        /*
         * The CONNECT has been processed to the system before. Hence we do the full DISCONNECT report in the DISCONNECT state.
         */
        state_set( SIDE_STATE_DISCONNECT );
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

    /* Move to the DISCONNECTED state */
    state_set_disconnected();

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

bool ComSide::connect( ComModel * p_com_model )     /* Will start the connection handshake process */
{
    ASSERT_DYGMA( state == SIDE_STATE_DISCONNECTED, "Invalid state for ComSide connection detected" );

    if( state != SIDE_STATE_DISCONNECTED )
    {
        return false;
    }

    if( p_com_model->is_connected() == false )
    {
        return false;
    }

    /* Register the communication model */
    this->p_com_model = p_com_model;

    /* Prapare the flags */
    disconnect_request = false;
    reconnect_request = false;

    /* Start the connection process */
    com_model_start( );

    return true;
}

void ComSide::disconnect( void )                    /* Will start the disconnect process */
{
    if( state == SIDE_STATE_DISCONNECTED )
    {
        /* If we already are in the DISCONNECTED state, we ignore the command */
        return;
    }

    disconnect_request = true;
}

bool ComSide::is_connected( void )
{
    return ( state == SIDE_STATE_CONNECTED && com_model_is_connected() == true ) ? true : false;
}

bool ComSide::is_disconnected( void )
{
    return ( state == SIDE_STATE_DISCONNECTED ) ? true : false;
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
