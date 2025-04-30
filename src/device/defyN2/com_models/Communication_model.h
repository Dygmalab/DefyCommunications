
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

#ifndef __COMMUNICATION_MODEL_H_
#define __COMMUNICATION_MODEL_H_

#include "Communications_protocol.h"

using namespace Communications_protocol;

class ComModel
{
    public:

        typedef enum
        {
            COM_MODEL_EVENT_RECONNECT_NEEDED = 1,
        } com_model_event_t;

        typedef void (* com_model_event_cb)( void * p_instance, com_model_event_t event );

    public:

        /* ******************************************************************************/
        /* Low level area - called by the actual communication model for initialization */
        /* ******************************************************************************/

        typedef bool (* com_model_send_packet_fn)( void * p_instance, Packet &packet );
        typedef bool (* com_model_read_packet_fn)( void * p_instance, Packet &packet );
        typedef bool (* com_model_is_connected_fn)( void * p_instance );
        typedef void (* com_model_disconnect_fn)( void * p_instance );

        typedef Communications_protocol::Devices (* com_model_dev_side_get_fn)( void * p_instance );;

        /* Communication model interface */
        typedef struct
        {
            com_model_send_packet_fn send_packet_fn;
            com_model_read_packet_fn read_packet_fn;
            com_model_is_connected_fn is_connected_fn;
            com_model_disconnect_fn disconnect_fn;

            com_model_dev_side_get_fn dev_side_get_fn;
        } com_model_if_t;

        typedef struct
        {
            void * p_instance;

            /* Communication */
            const com_model_if_t * p_model_if;
        } com_model_config_t;

        bool init( const com_model_config_t * p_config );

    public:

        /* ***************************************************************************************************************/
        /* High level area - abstracting the communication models to their actual use in the higher communication levels */
        /* ***************************************************************************************************************/

        bool send_packet( Packet &packet );
        bool read_packet( Packet &packet );
        bool is_connected( void );
        void disconnect( void );

        Communications_protocol::Devices dev_side_get( void );

    private:

        void * p_instance;

        /* Model interface */
        const com_model_if_t * p_model_if;
};

#endif /* __COMMUNICATION_MODEL_H_ */
