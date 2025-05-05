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

#include "Communications.h"
#include "Communications_protocol_rf.h"
#include "Communications_rf_pipe.h"
#include "CRC_wrapper.h"

/*****************************************************************/
/*                         Packet Queues                         */
/*****************************************************************/

uint16_t ComRfPipePacketQueue::size()
{
    return packet_count;
}

bool ComRfPipePacketQueue::empty()
{
    return ( packet_count == 0 ) ? true : false;
}

bool ComRfPipePacketQueue::full()
{
    return ( packet_count >= ( COM_RF_PIPE_PACKET_QUEUE_SIZE - 1 ) ) ? true : false;
}

bool ComRfPipePacketQueue::emplace( Packet & packet )
{
    if ( full( ) == true )
    {
        return false;
    }

    packets[write_pos].packet = packet;

    write_pos++;
    if ( write_pos >= COM_RF_PIPE_PACKET_QUEUE_SIZE )
    {
        write_pos = 0;
    }

    packet_count++;

    return true;
}

Communications_protocol_rf::WrapperPacket& ComRfPipePacketQueue::front()
{
    return packets[read_pos];
}

void ComRfPipePacketQueue::pop()
{
    if ( empty( ) == true )
    {
        return;
    }

    packet_count--;
    read_pos++;
    if ( read_pos >= COM_RF_PIPE_PACKET_QUEUE_SIZE )
    {
        read_pos = 0;
    }
}

void ComRfPipePacketQueue::clear()
{
    read_pos = 0;
    write_pos = 0;
    packet_count = 0;
}

/*****************************************************************/
/*                     RF Pipe Communication                     */
/*****************************************************************/

ComRfPipe::ComRfPipe( rfgw_pipe_id_t pipe_id )
{
    this->pipe_id = pipe_id;

    this->is_connected_ = false;
}

void ComRfPipe::parseErrProcess( buffer_t * p_buffer )
{
    /*
     * TODO: Possible parse error should be handled here by letting the upper layer know about the response failure
     */

    /* Initiate the search for new packet by skipping the first byte */
    buffer_update_read_pos( p_buffer, 1 );
}

void ComRfPipe::parseOkProcess( Communications_protocol_rf::parse_t * p_parse, buffer_t * p_buffer )
{
    if ( p_parse->status_code == Communications_protocol_rf::PARSE_STATUS_SUCCESS )
    {
        //Communications.callbacks.call( p_parse->pkt_cmd, p_parse->wrapperPacket.packet );

        /* Put the packet into the Rx queue */
        if( rx_messages.emplace( p_parse->wrapperPacket.packet ) == true )
        {
            /* Discard the already processed packet */
            buffer_update_read_pos( p_buffer, p_parse->pkt_size );
        }
    }
    else /* Packet parse failed with clear status code */
    {
        /*
         * TODO: Possible parse error should be handled here by letting the upper layer know about the response failure
         */

        parseErrProcess( p_buffer );
    }
}

void ComRfPipe::parseProcess( void )
{
    result_t result = RESULT_ERR;
    buffer_t * p_buffer_in;
    Communications_protocol_rf::parse_t parse;

    /* Get the buffer IN */
    result = rfgw_pipe_recv_buffer_get( pipe_id, &p_buffer_in );
    EXIT_IF_NOK( result );
    //ASSERT_DYGMA(result == RESULT_OK, "rf_pipe_recv_buffer_get failed");

    /* Parse and process the incoming data */
    result = parseBuffer( &parse, p_buffer_in );

    switch( result )
    {
        case RESULT_OK:

            parseOkProcess( &parse, p_buffer_in );

            break;

        case RESULT_ERR: /* Packet parse failed with un-clear status code */

            parseErrProcess( p_buffer_in );

            break;

        default:

            /*
             * No or incomplete packet.
             */

            break;
    }

    _EXIT:
    return;
}

void ComRfPipe::run()
{
    uint16_t pipe_send_loadsize = 0;
    uint16_t pipe_recv_loadsize = 0;

    if( is_connected_ == false )
    {
        return;
    }

    rfgw_pipe_get_send_freesize( pipe_id, &pipe_send_loadsize );
    rfgw_pipe_get_recv_loadsize( pipe_id, &pipe_recv_loadsize );
    parseProcess( );

    if ( !tx_messages.empty( ) )
    {
        Communications_protocol_rf::WrapperPacket & packet = tx_messages.front( );
        uint16_t size_to_transfer = packet.getSize( );
        if ( pipe_send_loadsize >= size_to_transfer )
        {
            rfgw_pipe_send( pipe_id, packet.buf, size_to_transfer );
            tx_messages.pop( );
        }
    }
}

void ComRfPipe::set_connected( void )
{
    this->is_connected_ = true;
}

void ComRfPipe::set_disconnected( void )
{
    this->is_connected_ = false;

    tx_messages.clear();
    rx_messages.clear();
}

bool ComRfPipe::is_connected(void)
{
    return is_connected_;
}

bool ComRfPipe::sendPacket( Packet & packet )
{
    if( is_connected_ == false )
    {
        return false;
    }

    packet.header.crc = 0;
    packet.header.crc = crc8( packet.buf, sizeof(Header) + packet.header.size );

    return tx_messages.emplace( packet );
}

bool ComRfPipe::readPacket( Packet & packet )
{
    if( is_connected_ == false || rx_messages.empty() == true )
    {
        return false;
    }

    /* Get the wrapper packet */
    Communications_protocol_rf::WrapperPacket & wrapper_packet = rx_messages.front( );

    /* Provide the "clean" packet */
    packet = wrapper_packet.packet;

    /* Clean the wrapper packet from the Rx queue */
    rx_messages.pop();

    return true;
}

void ComRfPipe::sendClear()
{
    if( is_connected_ == false )
    {
        return;
    }

    tx_messages.clear();

}

void ComRfPipe::readClear()
{
    if( is_connected_ == false )
    {
        return;
    }

    rx_messages.clear();
}
