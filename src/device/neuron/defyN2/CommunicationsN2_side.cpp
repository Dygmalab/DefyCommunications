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

#include "CommunicationsN2_side.h"

ComN2Side::ComN2Side( com_side_type_t side_type )
{
    /* Side definition */
    this->side_type = side_type;

    /* Communications */
    p_spiPort = nullptr;
    p_rfPipe = nullptr;

    com_model_wired_init( side_type );
    com_model_rf_init( side_type );
    com_model_ble_init( side_type );

    com_side_init();

    /* Side state */
    mode = N2_SIDE_MODE_UNKNOWN;
    state = N2_SIDE_STATE_DISCONNECTED;

    /* Flags */
    wired_enabled = true;     /* In general, Wired is currently always enabled */
    rf_enabled = false;
    ble_enabled = false;

    reconnect_request = false;
}

void ComN2Side::ble_enable()
{
    ASSERT_DYGMA( rf_enabled == false, "The RF and BLE should not be enabled at the same time." );

    ble_enabled = true;
    rf_enabled = false;

    /* Request the reconnection */
    reconnect_request = true;
}

void ComN2Side::rf_enable( ComRfPipe * p_rfPipe )
{
    ASSERT_DYGMA( ble_enabled == false, "The RF and BLE should not be enabled at the same time." );

    /* Register the RF pipe */
    this->p_rfPipe = p_rfPipe;

    ble_enabled = false;
    rf_enabled = true;

    /* Request the reconnection */
    reconnect_request = true;
}

inline void ComN2Side::com_model_wired_init( com_side_type_t side_type )
{
    bool result;

    ComModelWired::com_model_wired_config_t config;

    config.side_type = side_type;

    result = com_model_wired.init( &config );

    ASSERT_DYGMA( result == true, "com_model_wired.init failed" );

    UNUSED( result );
}

inline void ComN2Side::com_model_rf_init( com_side_type_t side_type )
{
    bool result;

    ComModelRf::com_model_rf_config_t config;

    config.side_type = side_type;

    result = com_model_rf.init( &config );

    ASSERT_DYGMA( result == true, "com_model_rf.init failed" );

    UNUSED( result );
}

inline void ComN2Side::com_model_ble_init( com_side_type_t side_type )
{
    bool result;

    ComModelBle::com_model_ble_config_t config;

    config.side_type = side_type;

    result = com_model_ble.init( &config );

    ASSERT_DYGMA( result == true, "com_model_ble.init failed" );

    UNUSED( result );
}

inline void ComN2Side::com_side_init( void )
{
    bool result;

    result = com_side.init( );

    ASSERT_DYGMA( result == true, "com_side.init failed" );

    UNUSED( result );
}

void ComN2Side::spi_port_register( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

bool ComN2Side::spi_is_connected( void )
{
    if( p_spiPort == nullptr )
    {
        return false;
    }

    return p_spiPort->is_connected();
}

bool ComN2Side::rf_is_connected( void )
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

inline void ComN2Side::com_side_wired_connect( void )
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
    mode = N2_SIDE_MODE_WIRED;
    state_set( N2_SIDE_STATE_CONNECTING );
}

inline void ComN2Side::com_side_rf_connect( void )
{
    ComModel * p_com_model;

    /* Register the RF pipe to the RF communication model */
    com_model_rf.rf_pipe_set( p_rfPipe );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_rf.com_model_get();

    /* Start the side connection process */
    if( com_side.connect( p_com_model ) == false )
    {
        ASSERT_DYGMA( false, "com_side.connect failed unexpectedly" );
        return;
    }

    /* Set the communication mode and start the connection */
    mode = N2_SIDE_MODE_RF;
    state_set( N2_SIDE_STATE_CONNECTING );
}

inline void ComN2Side::com_side_ble_connect( void )
{
    ComModel * p_com_model;

    /* Assign the SPI port to the ble communication model */
    com_model_ble.spi_port_set( p_spiPort );

    /* Set the pointer to the communication model  */
    p_com_model =  com_model_ble.com_model_get();

    /* Start the side connection process */
    if( com_side.connect( p_com_model ) == false )
    {
        ASSERT_DYGMA( false, "com_side.connect failed unexpectedly" );
        return;
    }

    /* Set the communication mode and start the connection */
    mode = N2_SIDE_MODE_BLE;
    state_set( N2_SIDE_STATE_CONNECTING );
}

inline void ComN2Side::com_side_disconnect( void )
{
    com_side.disconnect();
    state_set( N2_SIDE_STATE_DISCONNECTING );
}

inline bool ComN2Side::reconnect_needed( void )
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

inline void ComN2Side::state_set( n2_side_state_t state )
{
    this->state = state;
}

inline void ComN2Side::state_set_disconnected( void )
{
    /* Remove the SPI port */
    if( mode == N2_SIDE_MODE_WIRED || mode == N2_SIDE_MODE_BLE )
    {
        /* We nullify the spiPort only for the SPI-related modes. An existence of a valid and connected SPI port is a reason
         * for switching from RF mode to the Wired one. Thus we must keep the SPI port available in such case. */
        p_spiPort = nullptr;
    }

    /* Reset the reconnect flag */
    reconnect_request = false;

    /* Move to the disconnected state */
    mode = N2_SIDE_MODE_UNKNOWN;
    state_set( N2_SIDE_STATE_DISCONNECTED );
}

inline void ComN2Side::state_disconnected_process( void )
{
    if( spi_is_connected() == true )
    {
        if( ble_enabled == true )
        {
            com_side_ble_connect();
        }
        else if ( wired_enabled == true )
        {
            com_side_wired_connect();
        }
    }
    else if( rf_enabled == true && rf_is_connected() == true )
    {
        com_side_rf_connect();
    }
}

inline void ComN2Side::state_connecting_process( void )
{
    if( com_side.is_connected() == true )
    {
        state_set( N2_SIDE_STATE_CONNECTED );
    }
    else if( reconnect_needed() == true )
    {
        com_side_disconnect();
    }
}

inline void ComN2Side::state_connected_process( void )
{
    if( reconnect_needed() == true )
    {
        com_side_disconnect();
        return;
    }

    if( mode == N2_SIDE_MODE_RF && spi_is_connected() == true )
    {
        /*
         * The Wired communication is prioritized. Hence we disconnect the RF mode now.
         */
        com_side_disconnect();
        return;
    }
}

inline void ComN2Side::state_disconnecting_process( void )
{
    /* Waiting until the communication side is disconnected */
    if( com_side.is_disconnected() == false )
    {
        return;
    }

    state_set_disconnected();
}

inline void ComN2Side::machine( void )
{
    switch( state )
    {
        case N2_SIDE_STATE_DISCONNECTED:

            state_disconnected_process( );

            break;

        case N2_SIDE_STATE_CONNECTING:

            state_connecting_process( );

            break;

        case N2_SIDE_STATE_CONNECTED:

            state_connected_process( );

            break;

        case N2_SIDE_STATE_DISCONNECTING:

            state_disconnecting_process( );

            break;

        default:

            ASSERT_DYGMA( false, "Invalid Communications Side N2 state detected" );

            break;
    }
}

void ComN2Side::run( void )
{
    com_side.run();
    machine();
}

bool ComN2Side::sendPacket( Packet &packet )
{
    return com_side.sendPacket( packet );
}
