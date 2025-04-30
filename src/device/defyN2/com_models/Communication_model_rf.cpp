
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

#include "Communication_model_rf.h"

const ComModelRf::com_model_def_t ComModelRf::p_com_model_def_array[] =
{
    { .side_type = COM_SIDE_TYPE_KS_LEFT,    .dev_side = RF_DEFY_LEFT,  .dev_neuron = RF_NEURON_DEFY },
    { .side_type = COM_SIDE_TYPE_KS_RIGHT,   .dev_side = RF_DEFY_RIGHT, .dev_neuron = RF_NEURON_DEFY },
};
#define get_com_model_def( def, id ) _get_def( def, p_com_model_def_array, com_model_def_t, side_type, id )

#define COM_MODEL_DEF_ARRAY_LEN     ( sizeof( p_com_model_def_array ) / sizeof( com_model_def_t ) )

bool ComModelRf::com_model_init()
{
    ComModel::com_model_config_t config;

    config.p_instance = this;
    config.p_model_if = &com_model_if;

    return com_model.init( &config );
}

bool ComModelRf::init( com_model_rf_config_t * p_config )
{
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

void ComModelRf::event_handler( ComModel::com_model_event_t event )
{
    if( event_cb == nullptr )
    {
        return;
    }

    event_cb( p_instance, event );
}

void ComModelRf::rf_pipe_set( ComRfPipe * p_rfPipe )
{
    this->p_rfPipe = p_rfPipe;
}

ComModel * ComModelRf::com_model_get()
{
    return &com_model;
}

bool ComModelRf::msg_out_prepare( Packet &packet )
{
    /* Check if the packet is intended for the peer in the rf communication model.
     * We accept the model's side Device and broadcast (Unknown) */
    if( packet.header.device != p_com_model_def->dev_side && packet.header.device != UNKNOWN )
    {
        //ASSERT_DYGMA( false, "Unexpected Send Packet device type" );

        return false;
    }

    /* Set the wired model's Neuron Device */
    packet.header.device = p_com_model_def->dev_neuron;

    return true;
}

bool ComModelRf::send_packet( Packet &packet )
{
    return p_rfPipe->sendPacket( packet );
}

bool ComModelRf::read_packet( Packet &packet )
{
    /* Try to read the packet */
    if( p_rfPipe->readPacket( packet ) == false )
    {
        return false;
    }

    /* Check if the incoming packet fits the communication model we use */
    if( packet.header.device != p_com_model_def->dev_side )
    {
        ASSERT_DYGMA( false, "Unexpected Incoming Packet device type" );

        /* The communication model has changed. Request re-connection */
        event_handler( ComModel::COM_MODEL_EVENT_RECONNECT_NEEDED );

        return false;
    }

    return true;
}

bool ComModelRf::is_connected( void )
{
    if( p_rfPipe == nullptr )
    {
        return false;
    }

    return p_rfPipe->is_connected();
}

void ComModelRf::disconnect( void )
{
    /* Remove all the left packets */
    p_rfPipe->readClear();
    p_rfPipe->sendClear();

    /* Remove the spiPort instance */
    p_rfPipe = nullptr;
}

Devices ComModelRf::dev_side_get( void )
{
    return p_com_model_def->dev_side;
}

/**************************************************************/
/*                     Communication model                    */
/**************************************************************/

bool ComModelRf::com_model_msg_out_prepare_fn( void * p_instance, Packet &packet )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    return p_com_model->msg_out_prepare( packet );
}

bool ComModelRf::com_model_send_packet( void * p_instance, Packet &packet )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    return p_com_model->send_packet( packet );
}

bool ComModelRf::com_model_read_packet( void * p_instance, Packet &packet )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    return p_com_model->read_packet( packet );
}

bool ComModelRf::com_model_is_connected( void * p_instance )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    return p_com_model->is_connected( );
}

void ComModelRf::com_model_disconnect( void * p_instance )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    p_com_model->disconnect( );
}

Devices ComModelRf::com_model_dev_side_get( void * p_instance )
{
    ComModelRf * p_com_model = ( ComModelRf *)p_instance;

    return p_com_model->dev_side_get( );
}

const ComModel::com_model_if_t ComModelRf::com_model_if =
{
    .msg_out_prepare_fn = com_model_msg_out_prepare_fn,
    .send_packet_fn = com_model_send_packet,
    .read_packet_fn = com_model_read_packet,
    .is_connected_fn = com_model_is_connected,
    .disconnect_fn = com_model_disconnect,
    .dev_side_get_fn = com_model_dev_side_get,
};
