
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

#ifndef __COMMUNICATIONS_SIDE_H_
#define __COMMUNICATIONS_SIDE_H_

#include "Communications_protocol.h"
#include "Communications_rf_pipe.h"
#include "Communications_types.h"

#include "Communication_model_wired.h"
#include "Communication_model_rf.h"
#include "Communication_model_ble.h"

#include "SpiPort.h"
#include "Time_counter.h"

class ComSide
{
    public:

        ComSide( com_side_type_t side_type );

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
            SIDE_MODE_UNKNOWN = 1,

            SIDE_MODE_WIRED,
            SIDE_MODE_RF,
            SIDE_MODE_BLE,

        } side_mode_t;

        typedef enum
        {
            SIDE_STATE_DISCONNECTED = 1,
            SIDE_STATE_CONNECTION_START,
            SIDE_STATE_CONNECTION_WAIT,
            SIDE_STATE_CONNECTED,
            SIDE_STATE_DISCONNECT,
        } side_state_t;

    private:

        /* Side definition */
        com_side_type_t side_type;

        /* Communications */
        SpiPort * p_spiPort;
        ComRfPipe * p_rfPipe;

        ComModelWired com_model_wired;
        ComModelRf com_model_rf;
        ComModelBle com_model_ble;

        ComModel * p_com_model;

        /* Side state */
        side_state_t state;
        side_mode_t mode;

        /* Flags */
        bool wired_enabled;
        bool rf_enabled;
        bool ble_enabled;

        bool reconnect_needed;

        /* Timers */
        dl_timer_t connection_timer;

        /* Prototypes */
        inline void com_model_wired_init( com_side_type_t side_type );
        inline void com_model_rf_init( com_side_type_t side_type );
        inline void com_model_ble_init( com_side_type_t side_type );

        inline void com_model_event_process( ComModel::com_model_event_t event );

        inline void com_wired_start( void );
        inline void com_rf_start( void );
        inline void com_ble_start( void );

        inline void state_set( side_state_t state );
        inline void state_set_disconnected();
        inline void state_set_reconnect();

        inline void state_disconnected_process( void );
        inline void state_connection_start_process( void );
        inline void state_connection_wait_process( void );
        inline void state_connected_process( void );
        inline void state_disconnect_process( void );

        inline void machine( void );

    private:

        static void com_model_event_cb( void * p_instance, ComModel::com_model_event_t event );

};

#endif /* __COMMUNICATIONS_SIDE_H_ */
