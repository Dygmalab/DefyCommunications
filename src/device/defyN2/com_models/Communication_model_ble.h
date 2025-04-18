
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
#include "SpiPort.h"

class ComModelBle
{
    public:

        typedef struct
        {
            com_side_type_t side_type;

            /* Events */
            void * p_instance;
            ComModel::com_model_event_cb event_cb;
        } com_model_ble_config_t;

        bool init( com_model_ble_config_t * p_config );
        void spi_port_set( SpiPort * p_spiPort );

        ComModel * com_model_get();

    private:

        typedef struct
        {
            com_side_type_t side_type;

            Devices    dev_side;
            Devices    dev_neuron;
        } com_model_def_t;

    private:

        /* SPI */
        SpiPort * p_spiPort;

        /* Communication model instance */
        const com_model_def_t * p_com_model_def;
        ComModel com_model;

        /* Event handler */
        void * p_instance;
        ComModel::com_model_event_cb event_cb;

    private:

        inline bool com_model_init();

        inline void event_handler( ComModel::com_model_event_t event );

        inline bool send_packet( Packet &packet );
        inline bool read_packet( Packet &packet );
        inline bool is_connected( void );
        inline void disconnect( void );

        Devices dev_side_get( void );

    private:

        static bool com_model_send_packet( void * p_instance, Packet &packet );
        static bool com_model_read_packet( void * p_instance, Packet &packet );
        static bool com_model_is_connected( void * p_instance );
        static void com_model_disconnect( void * p_instance );

        static Devices com_model_dev_side_get( void * p_instance );

        static const com_model_def_t p_com_model_def_array[];
        static const ComModel::com_model_if_t com_model_if;
};


#endif /* __COMMUNICATION_MODEL_BLE_H_ */
