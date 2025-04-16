
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

#include "Communication_model_ble.h"

const ComModelBle::com_model_def_t ComModelBle::p_com_model_def_array[] =
{
    { .side_type = COM_SIDE_TYPE_KS_LEFT,    .dev_side = BLE_DEFY_LEFT,  .dev_neuron = BLE_NEURON_2_DEFY },
    { .side_type = COM_SIDE_TYPE_KS_RIGHT,   .dev_side = BLE_DEFY_RIGHT, .dev_neuron = BLE_NEURON_2_DEFY },
};
#define get_com_model_def( def, id ) _get_def( def, p_com_model_def_array, com_model_def_t, side_type, id )

bool ComModelBle::com_model_init()
{
    ComModel::com_model_config_t config;

    config.p_instance = this;
    config.p_model_if = &com_model_if;

    return com_model.init( &config );
}

bool ComModelBle::init( com_model_ble_config_t * p_config )
{
    /* Initialize the Spi Port instance */
    p_spiPort = nullptr;

    /* Get the the Communication model definition */
    get_com_model_def( p_com_model_def, p_config->side_type );
    ASSERT_DYGMA( p_com_model_def != NULL, "Invalid COM Side type detected" );

    /* Save the event handler */
    p_instance = p_config->p_instance;
    event_cb = p_config->event_cb;

    /* Initialize the communication model */
    if( com_model_init() == false )
    {
        return false;
    };

    return true;
}

void ComModelBle::event_handler( ComModel::com_model_event_t event )
{
    if( event_cb == nullptr )
    {
        return;
    }

    event_cb( p_instance, event );
}

void ComModelBle::spi_port_set( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

ComModel * ComModelBle::com_model_get()
{
    return &com_model;
}

bool ComModelBle::send_packet( Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComModelBle::read_packet( Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComModelBle::is_connected( void )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

void ComModelBle::disconnect( void )
{
    ASSERT_DYGMA( false, "Not implemented" );
}

Communications_protocol::Devices ComModelBle::dev_side_get( void )
{
    return p_com_model_def->dev_side;
}

/**************************************************************/
/*                     Communication model                    */
/**************************************************************/

bool ComModelBle::com_model_send_packet( void * p_instance, Packet &packet )
{
    ComModelBle * p_com_model = ( ComModelBle *)p_instance;

    return p_com_model->send_packet( packet );
}

bool ComModelBle::com_model_read_packet( void * p_instance, Packet &packet )
{
    ComModelBle * p_com_model = ( ComModelBle *)p_instance;

    return p_com_model->read_packet( packet );
}

bool ComModelBle::com_model_is_connected( void * p_instance )
{
    ComModelBle * p_com_model = ( ComModelBle *)p_instance;

    return p_com_model->is_connected( );
}

void ComModelBle::com_model_disconnect( void * p_instance )
{
    ComModelBle * p_com_model = ( ComModelBle *)p_instance;

    p_com_model->disconnect( );
}

Communications_protocol::Devices ComModelBle::com_model_dev_side_get( void * p_instance )
{
    ComModelBle * p_com_model = ( ComModelBle *)p_instance;

    return p_com_model->dev_side_get( );
}

const ComModel::com_model_if_t ComModelBle::com_model_if =
{
    .send_packet_fn = com_model_send_packet,
    .read_packet_fn = com_model_read_packet,
    .is_connected_fn = com_model_is_connected,
    .disconnect_fn = com_model_disconnect,
    .dev_side_get_fn = com_model_dev_side_get,
};
