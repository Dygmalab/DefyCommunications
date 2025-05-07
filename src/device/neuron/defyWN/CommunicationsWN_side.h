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

#ifndef __COMMUNICATIONSWN_SIDE_H_
#define __COMMUNICATIONSWN_SIDE_H_

#include "Communications_side.h"
#include "Communication_model_wired.h"

#include "SpiPort.h"

class ComWNSide
{
    public:

        ComWNSide( com_side_type_t side_type );

//        void init(void);

        void spi_port_register( SpiPort * p_spiPort );
        bool spi_is_connected( void );

//        bool readPacket(Packet &packet);      /* NOTE: Packets are distributed via the Communications.callbacks.call from within the Side module */
        bool sendPacket(Packet &packet);

        void run(void);

    private:

        typedef enum
        {
            WN_SIDE_MODE_UNKNOWN = 1,
            WN_SIDE_MODE_WIRED,
        } wn_side_mode_t;

        typedef enum
        {
            WN_SIDE_STATE_DISCONNECTED = 1,
            WN_SIDE_STATE_CONNECTING,
            WN_SIDE_STATE_CONNECTED,
            WN_SIDE_STATE_DISCONNECTING,
        } wn_side_state_t;

    private:

        /* Side definition */
        com_side_type_t side_type;

        /* Communications */
        SpiPort * p_spiPort;

        ComModelWired com_model_wired;

        ComSide com_side;

        /* Side state */
        wn_side_state_t state;
        wn_side_mode_t mode;

        /* Flags */
        bool wired_enabled;
        bool reconnect_request;

        /* Prototypes */
        inline void com_model_wired_init( com_side_type_t side_type );

        inline void com_side_init( void );
        inline void com_side_wired_connect( void );
        inline void com_side_disconnect( void );

        inline bool reconnect_needed( void );

        inline void state_set( wn_side_state_t state );
        inline void state_set_disconnected( void );
        inline void state_disconnected_process( void );
        inline void state_connecting_process( void );
        inline void state_connected_process( void );
        inline void state_disconnecting_process( void );

        inline void machine( void );
};

#endif /* __COMMUNICATIONSWN_SIDE_H_ */
