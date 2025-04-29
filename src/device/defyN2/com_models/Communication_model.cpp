
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

#include "Communication_model.h"

bool ComModel::init( const com_model_config_t * p_config )
{
    p_instance = p_config->p_instance;
    p_model_if = p_config->p_model_if;

    return true;
}

bool ComModel::msg_out_prepare( Packet &packet )
{
    if( p_model_if->msg_out_prepare_fn == nullptr )
    {
        return false;
    }

    return p_model_if->msg_out_prepare_fn( p_instance, packet );
}

bool ComModel::send_packet( Packet &packet )
{
    if( p_model_if->send_packet_fn == nullptr )
    {
        return false;
    }

    return p_model_if->send_packet_fn( p_instance, packet );
}

bool ComModel::read_packet( Packet &packet )
{
    if( p_model_if->read_packet_fn == nullptr )
    {
        return false;
    }

    return p_model_if->read_packet_fn( p_instance, packet );
}

bool ComModel::is_connected( void )
{
    if( p_model_if->is_connected_fn == nullptr )
    {
        return false;
    }

    return p_model_if->is_connected_fn( p_instance );
}

void ComModel::disconnect( void )
{
    if( p_model_if->disconnect_fn == nullptr )
    {
        return;
    }

    p_model_if->disconnect_fn( p_instance );
}

Communications_protocol::Devices ComModel::dev_side_get( void )
{
    if( p_model_if->dev_side_get_fn == nullptr )
    {
        return UNKNOWN;
    }

    return p_model_if->dev_side_get_fn( p_instance );
}
