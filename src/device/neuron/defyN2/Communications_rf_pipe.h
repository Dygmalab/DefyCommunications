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

#ifndef __COMMUNICATIONS_RF_PIPE_H_
#define __COMMUNICATIONS_RF_PIPE_H_

#include "Communications_protocol_rf.h"

class ComRfPipePacketQueue
{
#define COM_RF_PIPE_PACKET_QUEUE_SIZE       128

public:
    uint16_t size();

    bool empty();
    bool full();

    bool emplace( Packet & packet );
    Communications_protocol_rf::WrapperPacket& front();

    void pop();
    void clear();

private:
    Communications_protocol_rf::WrapperPacket packets[COM_RF_PIPE_PACKET_QUEUE_SIZE];

    uint16_t read_pos = 0;
    uint16_t write_pos = 0;
    uint16_t packet_count = 0;
};

class ComRfPipe
{
    public:

        ComRfPipe( rfgw_pipe_id_t pipe_id );

//        void init(void);

        void run(void);

        void set_connected();
        void set_disconnected();
        bool is_connected();

//        bool readPacket(Packet &packet);    /* Function will provide current packet and discard it from the queue*/
//        bool peekPacket(Packet &packet);    /* Function will provide current packet but keeps it in the queue */
//
        bool sendPacket(Packet &packet);
        bool readPacket(Packet &packet);    /* Function will provide current packet and discard it from the queue*/

        void sendClear();
        void readClear();

    private:

        rfgw_pipe_id_t pipe_id;

        bool is_connected_;

        /* Packet queues */
        ComRfPipePacketQueue tx_messages;
        ComRfPipePacketQueue rx_messages;

    private:

        void parseErrProcess( buffer_t * p_buffer );
        void parseOkProcess( Communications_protocol_rf::parse_t * p_parse, buffer_t * p_buffer );
        void parseProcess( void );
};

#endif /* __COMMUNICATIONS_RF_PIPE_H_ */
