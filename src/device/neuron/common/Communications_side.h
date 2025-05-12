
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

#include "Communication_model.h"

#include "Time_counter.h"

class ComSide
{

    public:

        bool init( void );

        bool connect( ComModel * p_com_model );     /* Will start the connection handshake process */
        void disconnect( void );                    /* Will request the disconnect process */

        bool is_connected( void );
        bool is_disconnected( void );

        bool sendPacket(Packet &packet);

        void run(void);

    private:

        typedef enum
        {
            SIDE_STATE_DISCONNECTED = 1,
            SIDE_STATE_CONNECTION_START,
            SIDE_STATE_CONNECTION_WAIT,
            SIDE_STATE_CONNECTED,
            SIDE_STATE_DISCONNECT,
        } side_state_t;

    private:

        ComModel * p_com_model;

        /* Side state */
        side_state_t state;

        /* Flags */
        bool disconnect_request;
        bool reconnect_request;

        /* Timers */
        dl_timer_t connection_timer;

        inline void com_model_event_process( ComModel::com_model_event_t event );

        inline bool com_model_is_connected( void );
        inline void com_model_start( void );

        inline bool disconnect_needed( void );

        inline void state_set( side_state_t state );
        inline void state_set_disconnected( void );

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
