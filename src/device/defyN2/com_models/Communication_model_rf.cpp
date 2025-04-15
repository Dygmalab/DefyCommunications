
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

bool ComModelRf::com_model_init()
{
    ComModel::com_model_config_t config;

    config.p_instance = this;
    config.p_model_if = &com_model_if;

    return com_model.init( &config );
}

bool ComModelRf::init( com_side_type_t side )
{
    /* Initialize the communication model */
    return com_model_init();
}

ComModel * ComModelRf::com_model_get()
{
    return &com_model;
}

bool ComModelRf::send_packet( Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComModelRf::read_packet( Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComModelRf::is_connected( void )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

void ComModelRf::disconnect( void )
{
    ASSERT_DYGMA( false, "Not implemented" );
}

/**************************************************************/
/*                     Communication model                    */
/**************************************************************/

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

const ComModel::com_model_if_t ComModelRf::com_model_if =
{
    .send_packet_fn = com_model_send_packet,
    .read_packet_fn = com_model_read_packet,
    .is_connected_fn = com_model_is_connected,
    .disconnect_fn = com_model_disconnect,
};
