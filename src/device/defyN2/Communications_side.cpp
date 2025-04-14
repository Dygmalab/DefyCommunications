
#include "dl_middleware.h"
#include "Communications.h"
#include "Communications_side.h"

#define CONNECT_WAIT_TIMEOUT_MS     100     /* Setting this shorter than KS IS_ALIVE wait (300 ms). This is for the case when the KS was already in the Connected state
                                               and sends its own IS_ALIVE message in response for the Neuron one. This way the Neuron repeated IS_ALIVE will reach
                                               KS side before it retries on its side */


const ComSide::side_com_driver_t ComSide::side_com_driver_wired = { .send_packet_fn = com_send_packet_wired,    .read_packet_fn = com_read_packet_wired,
                                                                    .is_connected_fn = com_is_connected_wired,  .disconnect_fn = com_disconnect_wired };
const ComSide::side_com_driver_t ComSide::side_com_driver_rf    = { .send_packet_fn = com_send_packet_rf,       .read_packet_fn = com_read_packet_rf,
                                                                    .is_connected_fn = com_is_connected_rf,     .disconnect_fn = com_disconnect_rf };
const ComSide::side_com_driver_t ComSide::side_com_driver_ble   = { .send_packet_fn = com_send_packet_ble,      .read_packet_fn = com_read_packet_ble,
                                                                    .is_connected_fn = com_is_connected_ble,    .disconnect_fn = com_disconnect_ble };

const ComSide::side_def_t ComSide::p_side_def_array[] =
{
    { .type = SIDE_TYPE_KS_LEFT,
            .com_wired = { .dev_side = Communications_protocol::KEYSCANNER_DEFY_LEFT, .dev_neuron = NEURON_DEFY, .p_driver = &side_com_driver_wired },
            .com_rf    = { .dev_side = Communications_protocol::RF_DEFY_LEFT, .dev_neuron = RF_NEURON_DEFY, .p_driver = &side_com_driver_rf },
            .com_ble   = { .dev_side = Communications_protocol::BLE_DEFY_LEFT, .dev_neuron = BLE_NEURON_2_DEFY, .p_driver = &side_com_driver_ble }
    },

    { .type = SIDE_TYPE_KS_RIGHT,
            .com_wired = { .dev_side = Communications_protocol::KEYSCANNER_DEFY_RIGHT, .dev_neuron = NEURON_DEFY, .p_driver = &side_com_driver_wired },
            .com_rf    = { .dev_side = Communications_protocol::RF_DEFY_RIGHT, .dev_neuron = RF_NEURON_DEFY, .p_driver = &side_com_driver_rf },
            .com_ble   = { .dev_side = Communications_protocol::BLE_DEFY_RIGHT, .dev_neuron = BLE_NEURON_2_DEFY, .p_driver = &side_com_driver_ble }
    },
};
#define get_side_def( def, id ) _get_def( def, p_side_def_array, side_def_t, type, id )

ComSide::ComSide( side_type_t side_type )
{
    /* Side definition */
    get_side_def( this->p_side_def, side_type );
    ASSERT_DYGMA( p_side_def != NULL, "Invalid Communications Side definition" );

    /* Communications */
    this->p_spiPort = nullptr;
    this->p_com_model = nullptr;

    /* Side state */
    this->state = SIDE_STATE_DISCONNECTED;

    /* Flags */
    this->wired_enabled = true;     /* In general, Wired is currently always enabled */
    this->rf_enabled = false;
    this->ble_enabled = false;
}

void ComSide::spi_port_register( SpiPort * p_spiPort )
{
    this->p_spiPort = p_spiPort;
}

bool ComSide::wired_is_connected( void )
{
    if( this->p_spiPort == nullptr )
    {
        return false;
    }

    return this->p_spiPort->is_connected();
}

/******************************************************************/
/*                         Communications                         */
/******************************************************************/

inline void ComSide::com_wired_start( void )
{
    this->p_com_model = &this->p_side_def->com_wired;

    this->mode = SIDE_MODE_WIRED;
    state_set( SIDE_STATE_CONNECTION_START );
}

bool ComSide::com_send_packet_wired( ComSide * p_comSide, Packet &packet )
{
    return p_comSide->p_spiPort->sendPacket( packet );
}

bool ComSide::com_send_packet_rf( ComSide * p_comSide, Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComSide::com_send_packet_ble( ComSide * p_comSide, Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComSide::com_read_packet_wired( ComSide * p_comSide, Packet &packet )
{
    /* Try to read the packet */
    if( p_comSide->p_spiPort->readPacket( packet ) == false )
    {
        return false;
    }

    /* Check if the incoming packet fits the communication model we use */
    if( packet.header.device != p_comSide->p_com_model->dev_side )
    {
        ASSERT_DYGMA( false, "Unexpected Incoming Packet device type" );

        return false;
    }

    return true;
}

bool ComSide::com_read_packet_rf( ComSide * p_comSide, Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComSide::com_read_packet_ble( ComSide * p_comSide, Packet &packet )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComSide::com_is_connected_wired( ComSide * p_comSide )
{
    return p_comSide->wired_is_connected();
}

bool ComSide::com_is_connected_rf( ComSide * p_comSide )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

bool ComSide::com_is_connected_ble( ComSide * p_comSide )
{
    ASSERT_DYGMA( false, "Not implemented" );

    return false;
}

void ComSide::com_disconnect_wired( ComSide * p_comSide )
{
    /* Remove all the left packets */
    p_comSide->p_spiPort->clearRead();
    p_comSide->p_spiPort->clearSend();

    /* Remove the spiPort instance */
    p_comSide->p_spiPort = nullptr;
}

void ComSide::com_disconnect_rf( ComSide * p_comSide )
{
    ASSERT_DYGMA( false, "Not implemented" );
}

void ComSide::com_disconnect_ble( ComSide * p_comSide )
{
    ASSERT_DYGMA( false, "Not implemented" );
}

inline bool ComSide::com_send_packet( Packet &packet )
{
    Packet packet_send = packet;

    if( this->p_com_model == NULL )
    {
        ASSERT_DYGMA( this->p_com_model != NULL, "The Com Side communication model has not been set yet" );

        return false;
    }

    /* Check if the packet is intended for the peer in this communication model. We accept the model's side Device and broadcast (Unknown) */
    if( packet_send.header.device != this->p_com_model->dev_side && packet_send.header.device != UNKNOWN )
    {
        //ASSERT_DYGMA( false, "Unexpected Send Packet device type" );

        return false;
    }

    /* Set the model's Neuron Device */
    packet_send.header.device = this->p_com_model->dev_neuron;

    /* Send the packet */
    return this->p_com_model->p_driver->send_packet_fn( this, packet_send );
}

inline bool ComSide::com_read_packet( Packet &packet )
{
    if( this->p_com_model == NULL )
    {
        ASSERT_DYGMA( this->p_com_model != NULL, "The Com Side communication model has not been set yet" );

        return false;
    }

    /* Try to read the packet */
    if ( this->p_com_model->p_driver->read_packet_fn( this, packet ) == false )
    {
        return false;
    }

    return true;
}

inline bool ComSide::com_is_connected( void )
{
    if( this->p_com_model == NULL )
    {
        ASSERT_DYGMA( this->p_com_model != NULL, "The Com Side communication model has not been set yet" );

        return false;
    }

    return p_com_model->p_driver->is_connected_fn( this );
}

inline void ComSide::com_disconnect( void )
{
    if( this->p_com_model == NULL )
    {
        ASSERT_DYGMA( this->p_com_model != NULL, "The Com Side communication model has not been set yet" );

        return;
    }

    p_com_model->p_driver->disconnect_fn( this );
}

inline void ComSide::state_set( side_state_t state )
{
    this->state = state;
}

inline void ComSide::state_set_disconnected()
{
    p_com_model = nullptr;
    state_set( SIDE_STATE_DISCONNECTED );
}

inline void ComSide::state_disconnected_process( void )
{
    if( wired_is_connected() == true )
    {
        if( this->ble_enabled == true )
        {
            ASSERT_DYGMA( false, "Not implemented yet" )
        }
        else if ( this->wired_enabled == true )
        {
            p_com_model = &p_side_def->com_wired;
            state_set( SIDE_STATE_CONNECTION_START );
        }
    }
}

inline void ComSide::state_connection_start_process( void )
{
    Packet packet_send{};

    /* Check whether the connection is still active */
    if( com_is_connected() == false )
    {
        state_set_disconnected();
        return;
    }

    /* Prepare the IS_ALIVE packet */
    packet_send.header.size    = 0;
    packet_send.header.command = IS_ALIVE;

    if( com_send_packet( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_send_packet failure at this point" )
        return;
    }

    /* Wait for the CONNECT message */
    timer_set_ms( &connection_timer, CONNECT_WAIT_TIMEOUT_MS );
    state_set( SIDE_STATE_CONNECTION_WAIT );
}

inline void ComSide::state_connection_wait_process( void )
{
    Packet packet_read{};
    Packet packet_send{};

    /* Check the timeout for receiving the CONNECT message */
    if( timer_check( &connection_timer ) == true )
    {
        /* Re-iterate the handshake process */
        state_set( SIDE_STATE_CONNECTION_START );
        return;
    }

    /* Try to read the packet */
    if( com_read_packet( packet_read ) == false )
    {
        return;
    }

    /* We wait for the CONNECTED message only */
    if( packet_read.header.command != CONNECTED )
    {
        return;
    }

    /* Prepare the CONNECTED response message and send it */
    packet_send.header.size    = 0;
    //packet.header.device  = p.header.device;  /* The device is filled inside the send function */
    packet_send.header.command = CONNECTED;

    if( com_send_packet( packet_send ) == false )
    {
        ASSERT_DYGMA( false, "Unexpected com_send_packet failure at this point" )
        return;
    }

    /* Set the CONNECTED state */
    state_set( SIDE_STATE_CONNECTED );

    /* Process the incoming CONNECTED message to the system */
    Communications.callbacks.call( packet_read.header.command, packet_read );
}

inline void ComSide::state_connected_process( void )
{
    Packet packet_read{};

    /* Check whether the connection is still active */
    if( com_is_connected() == false )
    {
        state_set( SIDE_STATE_DISCONNECT );
        return;
    }

    /* Try to read the packet */
    if( com_read_packet( packet_read ) == false )
    {
        return;
    }

    /* Check the incoming packet */
    if( packet_read.header.device != p_com_model->dev_side )
    {
        ASSERT_DYGMA( false, "Unexpected communication model change" )

        /* The communication model has changed, disconnect at this point and let the process refresh the communication from the beginning */
        state_set( SIDE_STATE_DISCONNECT );
        return;
    }
    else if( packet_read.header.command == IS_ALIVE )
    {
        /* The peer has restarted, restart the connection process */
        state_set( SIDE_STATE_CONNECTION_START );
        return;
    }

    /* Process the incoming packet to the system */
    Communications.callbacks.call( packet_read.header.command, packet_read );
}

inline void ComSide::state_disconnect_process( void )
{
    Packet packet{};

    packet.header.command = Communications_protocol::DISCONNECTED;
    packet.header.device  = this->p_com_model->dev_side;
    Communications.callbacks.call(packet.header.command, packet);

    /* Perform the Com model-specific disconnect */
    com_disconnect();

    /* Remove the communication model */
    p_com_model = nullptr;

    /* Move to the DISCONNECTED state */
    state_set( SIDE_STATE_DISCONNECTED );
}

inline void ComSide::machine( void )
{
    switch( state )
    {
        case SIDE_STATE_DISCONNECTED:

            state_disconnected_process();

            break;

        case SIDE_STATE_CONNECTION_START:

            state_connection_start_process();

            break;

        case SIDE_STATE_CONNECTION_WAIT:

            state_connection_wait_process();

            break;

        case SIDE_STATE_CONNECTED:

            state_connected_process();

            break;

        case SIDE_STATE_DISCONNECT:

            state_disconnect_process();

            break;

        default:

            ASSERT_DYGMA( false, "Invalid Side Communication state." );

            break;
    }
}

void ComSide::run( void )
{
    machine();
}

bool ComSide::sendPacket( Packet &packet )
{
    /* Check if it is possible to send the packet */
    if( this->state != SIDE_STATE_CONNECTED )
    {
        return false;
    }

    /* Send the packet using the actual communication model */
    return com_send_packet( packet );
}
