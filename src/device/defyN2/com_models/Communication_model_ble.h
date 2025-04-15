
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

#ifndef __COMMUNICATION_MODEL_BLE_H_
#define __COMMUNICATION_MODEL_BLE_H_

#include "dl_middleware.h"

#include "Communication_model.h"
#include "Communications_types.h"

class ComModelBle
{
    public:

        bool init( com_side_type_t side );
        ComModel * com_model_get();

    private:

        /* Communication model instance */
        ComModel com_model;

        static const ComModel::com_model_if_t com_model_if;

    private:

        inline bool com_model_init();

        inline bool send_packet( Packet &packet );
        inline bool read_packet( Packet &packet );
        inline bool is_connected( void );
        inline void disconnect( void );

    private:

        static bool com_model_send_packet( void * p_instance, Packet &packet );
        static bool com_model_read_packet( void * p_instance, Packet &packet );
        static bool com_model_is_connected( void * p_instance );
        static void com_model_disconnect( void * p_instance );
};


#endif /* __COMMUNICATION_MODEL_BLE_H_ */
