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

#include "CommunicationsWN_side.h"

ComWNSide::ComWNSide( com_side_type_t side_type )
{
    /* Side definition */
    this->side_type = side_type;

    /* Communications */
    p_spiPort = nullptr;

    com_model_wired_init( side_type );

    com_side_init();

    /* Side state */
    mode = WN_SIDE_MODE_UNKNOWN;
    state = WN_SIDE_STATE_DISCONNECTED;

    /* Flags */
    wired_enabled = true;     /* In general, Wired is currently always enabled */
    reconnect_request = false;
}

inline void ComWNSide::com_model_wired_init( com_side_type_t side_type )
{
    bool result;

    ComModelWired::com_model_wired_config_t config;

    config.side_type = side_type;

    result = com_model_wired.init( &config );

    ASSERT_DYGMA( result == true, "com_model_wired.init failed" );

    UNUSED( result );
}

inline void ComWNSide::com_side_init( void )
{
    bool result;

    result = com_side.init( );

    ASSERT_DYGMA( result == true, "com_side.init failed" );

    UNUSED( result );
}

void ComWNSide::spi_port_register( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

bool ComWNSide::spi_is_connected( void )
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

inline void ComWNSide::com_side_wired_connect( void )
{
    ComModel * p_com_model;

    /* Assign the SPI port to the wired communication model */
    com_model_wired.spi_port_set( p_spiPort );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_wired.com_model_get();

    /* Start the side connection process */
    if( com_side.connect( p_com_model ) == false )
    {
        ASSERT_DYGMA( false, "com_side.connect failed unexpectedly" );
        return;
    }

    /* Set the communication mode and start the connection */
    mode = WN_SIDE_MODE_WIRED;
    state_set( WN_SIDE_STATE_CONNECTING );
}

inline void ComWNSide::com_side_disconnect( void )
{
    com_side.disconnect();
    state_set( WN_SIDE_STATE_DISCONNECTING );
}

inline bool ComWNSide::reconnect_needed( void )
{
    /*
     * In this module when any of these conditions are met, the module should trigger the reconnection process.
     */
    if ( reconnect_request == true || com_side.is_disconnected( ) == true )
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

inline void ComWNSide::state_set( wn_side_state_t state )
{
    this->state = state;
}

inline void ComWNSide::state_set_disconnected( void )
{
    /* Remove the SPI port */
    p_spiPort = nullptr;

    /* Reset the reconnect flag */
    reconnect_request = false;

    /* Move to the disconnected state */
    mode = WN_SIDE_MODE_UNKNOWN;
    state_set( WN_SIDE_STATE_DISCONNECTED );
}

inline void ComWNSide::state_disconnected_process( void )
{
    if( wired_enabled == true && spi_is_connected() == true )
    {
        com_side_wired_connect();
    }
}

inline void ComWNSide::state_connecting_process( void )
{
    if( com_side.is_connected() == true )
    {
        state_set( WN_SIDE_STATE_CONNECTED );
    }
    else if( reconnect_needed() == true )
    {
        com_side_disconnect();
    }
}

inline void ComWNSide::state_connected_process( void )
{
    if( reconnect_needed() == true )
    {
        com_side_disconnect();
        return;
    }
}

inline void ComWNSide::state_disconnecting_process( void )
{
    /* Waiting until the communication side is disconnected */
    if( com_side.is_disconnected() == false )
    {
        return;
    }

    state_set_disconnected();
}

inline void ComWNSide::machine( void )
{
    switch( state )
    {
        case WN_SIDE_STATE_DISCONNECTED:

            state_disconnected_process( );

            break;

        case WN_SIDE_STATE_CONNECTING:

            state_connecting_process( );

            break;

        case WN_SIDE_STATE_CONNECTED:

            state_connected_process( );

            break;

        case WN_SIDE_STATE_DISCONNECTING:

            state_disconnecting_process( );

            break;

        default:

            ASSERT_DYGMA( false, "Invalid Communications Side WN state detected" );

            break;
    }
}

void ComWNSide::run( void )
{
    com_side.run();
    machine();
}

bool ComWNSide::sendPacket( Packet &packet )
{
    return com_side.sendPacket( packet );
}
