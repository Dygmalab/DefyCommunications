
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
#include "SpiPort.h"
#include "Time_counter.h"


typedef enum
{
    SIDE_TYPE_KS_LEFT = 1,
    SIDE_TYPE_KS_RIGHT,
} side_type_t;

class ComSide
{
    public:

        ComSide( side_type_t side_type );

//        void init(void);

        void spi_port_register( SpiPort * p_spiPort );

        bool wired_is_connected( void );

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

        typedef bool (* side_com_send_packet_fn)( ComSide * p_comSide, Packet &packet );
        typedef bool (* side_com_read_packet_fn)( ComSide * p_comSide, Packet &packet );
        typedef bool (* side_com_is_connected_fn)( ComSide * p_comSide );
        typedef void (* side_com_disconnect_fn)( ComSide * p_comSide );

        typedef struct
        {
            /* Communication */
            side_com_send_packet_fn send_packet_fn;
            side_com_read_packet_fn read_packet_fn;
            side_com_is_connected_fn is_connected_fn;
            side_com_disconnect_fn disconnect_fn;
        } side_com_driver_t;

        typedef struct
        {
            Communications_protocol::Devices    dev_side;
            Communications_protocol::Devices    dev_neuron;

            const side_com_driver_t * p_driver;
        } side_com_model_t;

        typedef struct
        {
            side_type_t type;

            side_com_model_t com_wired;
            side_com_model_t com_rf;
            side_com_model_t com_ble;
        } side_def_t;


    private:

        /* Side definition */
        const side_def_t * p_side_def;

        /* Communications */
        SpiPort * p_spiPort;
        const side_com_model_t * p_com_model;

        /* Side state */
        side_state_t state;
        side_mode_t mode;

        /* Flags */
        bool wired_enabled;
        bool rf_enabled;
        bool ble_enabled;

        /* Timers */
        dl_timer_t connection_timer;

        /* Prototypes */
        inline void com_wired_start( void );

        inline bool com_send_packet( Packet &packet );
        inline bool com_read_packet( Packet &packet );
        inline bool com_is_connected( void );
        inline void com_disconnect( void );

        inline bool com_read_packet_wired( Packet &packet );

        inline void state_set( side_state_t state );
        inline void state_set_disconnected();

        inline void state_disconnected_process( void );
        inline void state_connection_start_process( void );
        inline void state_connection_wait_process( void );
        inline void state_connected_process( void );
        inline void state_disconnect_process( void );

        inline void machine( void );

    private:

        /* Com drivers */
        static bool com_send_packet_wired( ComSide * p_comSide, Packet &packet );
        static bool com_send_packet_rf( ComSide * p_comSide, Packet &packet );
        static bool com_send_packet_ble( ComSide * p_comSide, Packet &packet );
        static bool com_read_packet_wired( ComSide * p_comSide, Packet &packet );
        static bool com_read_packet_rf( ComSide * p_comSide, Packet &packet );
        static bool com_read_packet_ble( ComSide * p_comSide, Packet &packet );
        static bool com_is_connected_wired( ComSide * p_comSide );
        static bool com_is_connected_rf( ComSide * p_comSide );
        static bool com_is_connected_ble( ComSide * p_comSide );
        static void com_disconnect_wired( ComSide * p_comSide );
        static void com_disconnect_rf( ComSide * p_comSide );
        static void com_disconnect_ble( ComSide * p_comSide );

        static const side_com_driver_t side_com_driver_wired;
        static const side_com_driver_t side_com_driver_rf;
        static const side_com_driver_t side_com_driver_ble;

        static const side_def_t p_side_def_array[];

};

#endif /* __COMMUNICATIONS_SIDE_H_ */
