
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

#include "Communication_model_wired.h"

const ComModelWired::com_model_def_t ComModelWired::p_com_model_def_array[] =
{
    { .side_type = COM_SIDE_TYPE_KS_LEFT,    .dev_side = KEYSCANNER_DEFY_LEFT,  .dev_neuron = WIRED_NEURON_DEFY },
    { .side_type = COM_SIDE_TYPE_KS_RIGHT,   .dev_side = KEYSCANNER_DEFY_RIGHT, .dev_neuron = WIRED_NEURON_DEFY },
};
#define get_com_model_def( def, id ) _get_def( def, p_com_model_def_array, com_model_def_t, side_type, id )


bool ComModelWired::com_model_init()
{
    ComModel::com_model_config_t config;

    config.p_instance = this;
    config.p_model_if = &com_model_if;

    return com_model.init( &config );
}

bool ComModelWired::init( com_model_wired_config_t * p_config )
{
    /* Initialize the Spi Port instance */
    p_spiPort = nullptr;

    /* Get the the Communication model definition */
    get_com_model_def( p_com_model_def, p_config->side_type );
    ASSERT_DYGMA( p_com_model_def != NULL, "Invalid COM Side type detected" );

    /* Initialize the event handler */
    p_instance = nullptr;
    event_cb = nullptr;

    /* Initialize the communication model */
    if( com_model_init() == false )
    {
        return false;
    };

    return true;
}

void ComModelWired::event_handler( ComModel::com_model_event_t event )
{
    if( event_cb == nullptr )
    {
        return;
    }

    event_cb( p_instance, event );
}

void ComModelWired::spi_port_set( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

ComModel * ComModelWired::com_model_get()
{
    return &com_model;
}

void ComModelWired::event_cb_config( const ComModel::com_model_event_cb_config_t * p_event_cb_config )
{
    this->p_instance = p_event_cb_config->p_instance;
    this->event_cb = p_event_cb_config->event_cb;
}

bool ComModelWired::send_packet( Packet &packet )
{
    /* Check if the packet is intended for the peer in the wired communication model.
     * We accept the model's side Device and broadcast (Unknown) */
    if( packet.header.device != p_com_model_def->dev_side && packet.header.device != UNKNOWN )
    {
        //ASSERT_DYGMA( false, "Unexpected Send Packet device type" );

        return false;
    }

    /* Set the wired model's Neuron Device */
    packet.header.device = p_com_model_def->dev_neuron;

    return p_spiPort->sendPacket( packet );
}

bool ComModelWired::read_packet( Packet &packet )
{
    /* Try to read the packet */
    if( p_spiPort->readPacket( packet ) == false )
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

bool ComModelWired::is_connected( void )
{
    if( p_spiPort == nullptr )
    {
        return false;
    }

    return p_spiPort->is_connected();
}

void ComModelWired::disconnect( void )
{
    /* Remove all the left packets */
    p_spiPort->clearRead();
    p_spiPort->clearSend();

    /* Remove the spiPort instance */
    p_spiPort = nullptr;
}

Devices ComModelWired::dev_side_get( void )
{
    return p_com_model_def->dev_side;
}

/**************************************************************/
/*                     Communication model                    */
/**************************************************************/

void ComModelWired::com_model_event_cb_config_fn( void * p_instance, const ComModel::com_model_event_cb_config_t * p_event_cb_config )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    return p_com_model->event_cb_config( p_event_cb_config );
}

bool ComModelWired::com_model_send_packet( void * p_instance, Packet &packet )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    return p_com_model->send_packet( packet );
}

bool ComModelWired::com_model_read_packet( void * p_instance, Packet &packet )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    return p_com_model->read_packet( packet );
}

bool ComModelWired::com_model_is_connected( void * p_instance )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    return p_com_model->is_connected( );
}

void ComModelWired::com_model_disconnect( void * p_instance )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    p_com_model->disconnect( );
}

Devices ComModelWired::com_model_dev_side_get( void * p_instance )
{
    ComModelWired * p_com_model = ( ComModelWired *)p_instance;

    return p_com_model->dev_side_get( );
}

const ComModel::com_model_if_t ComModelWired::com_model_if =
{
    .event_cb_config_fn = com_model_event_cb_config_fn,
    .send_packet_fn = com_model_send_packet,
    .read_packet_fn = com_model_read_packet,
    .is_connected_fn = com_model_is_connected,
    .disconnect_fn = com_model_disconnect,
    .dev_side_get_fn = com_model_dev_side_get,
};
