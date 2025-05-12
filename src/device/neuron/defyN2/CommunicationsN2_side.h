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

#ifndef __COMMUNICATIONSN2_SIDE_H_
#define __COMMUNICATIONSN2_SIDE_H_

#include "Communications_side.h"

#include "Communication_model_wired.h"
#include "Communication_model_rf.h"
#include "Communication_model_ble.h"

#include "SpiPort.h"

class ComN2Side
{
    public:

        ComN2Side( com_side_type_t side_type );

//        void init(void);
        void ble_enable();
        void rf_enable( ComRfPipe * p_rfPipe );

        void spi_port_register( SpiPort * p_spiPort );
        bool spi_is_connected( void );

        bool rf_is_connected( void );

//        bool readPacket(Packet &packet);      /* NOTE: Packets are distributed via the Communications.callbacks.call from within the Side module */
        bool sendPacket(Packet &packet);

        void run(void);

    private:

        typedef enum
        {
            N2_SIDE_MODE_UNKNOWN = 1,

            N2_SIDE_MODE_WIRED,
            N2_SIDE_MODE_RF,
            N2_SIDE_MODE_BLE,

        } n2_side_mode_t;

        typedef enum
        {
            N2_SIDE_STATE_DISCONNECTED = 1,
            N2_SIDE_STATE_CONNECTING,
            N2_SIDE_STATE_CONNECTED,
            N2_SIDE_STATE_DISCONNECTING,
        } n2_side_state_t;

    private:

        /* Side definition */
        com_side_type_t side_type;

        /* Communications */
        SpiPort * p_spiPort;
        ComRfPipe * p_rfPipe;

        ComModelWired com_model_wired;
        ComModelRf com_model_rf;
        ComModelBle com_model_ble;

        ComSide com_side;

        /* Side state */
        n2_side_state_t state;
        n2_side_mode_t mode;

        /* Flags */
        bool wired_enabled;
        bool rf_enabled;
        bool ble_enabled;

        bool reconnect_request;

        /* Prototypes */
        inline void com_model_wired_init( com_side_type_t side_type );
        inline void com_model_rf_init( com_side_type_t side_type );
        inline void com_model_ble_init( com_side_type_t side_type );

        inline void com_side_init( void );

        inline void com_side_wired_connect( void );
        inline void com_side_rf_connect( void );
        inline void com_side_ble_connect( void );

        inline void com_side_disconnect( void );

        inline bool reconnect_needed( void );

        inline void state_set( n2_side_state_t state );
        inline void state_set_disconnected( void );
        inline void state_disconnected_process( void );
        inline void state_connecting_process( void );
        inline void state_connected_process( void );
        inline void state_disconnecting_process( void );

        inline void machine( void );
};

#endif /* __COMMUNICATIONSN2_SIDE_H_ */
