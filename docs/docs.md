# Communication Defy Protocol

This documentation will explain the protocol used between the keyscanner and the neuron.

## Protocol

All the protocol packets and communications are declared in
the [Communications_protocol.h](https://github.com/Dygmalab/DefyCommunications/blob/main/src/Communications_protocol.h).

https://github.com/Dygmalab/DefyCommunications/blob/main/src/Communications_protocol.h

<div align="center">
  <img alt="UML Diagram" src="http://www.plantuml.com/plantuml/svg/fLF1Zjem43rlli8tsgvOhXKtRp90XB4ZEq3fghAymOMq9459wA7L_dljs0ifMlPINpBpPkRD-7ao6Y3s_4DdROC6eo0eLQ6Reyeq2gkYE9LvfjgyAXjvhAksoge3-XFSwV9KT75LRXiBt8MCKWXJY8mTnQBlJY6LB04goTaLCyRJdhy2Bc96ZJV3GivX8umP2ub9l09dHe1jjZ3P7IfZcW9VOUB1HnvFPoa58JnWMK9Ch_C8tW2tmGLEPCW8u-AVo996mAU4hQ-245qi-OZftBiB5a4lcc02QGgzOWAGcEyQpq4kZVKIUFqY_IlFj9FF3qlD015CDePmahu1TtSqE8KbPzHXwvXttQynugHOqZbbQ-hwPWVLD6YcrLRNNT_xp3qY6f-VqwBRRZG3-Xa3vC_zNZMog6ejpOhyr6PdnkYvgWwNX2R_RGjFUTb-bcs_RT9LkBPxDyBOpzBrLAsw5A8djC1VPCen5HDpVH5_1JIqvDNt3wxeupVBRlZj6GrGNEvrdRUgD9FN-k3MTfyVtLwx7BpRrNhNXTx8kR7n3rwmJyFXhP_YuQpUVx6yodsRn8lu7ecNzFNy1G00">
</div>

### Packet

![UML Diagram](packet.drawio.svg)

```cpp
struct Header {
  Commands command;
  Devices device;
  struct {
    uint8_t size : 7;
    bool has_more_packets : 1;
  };
};
static_assert(sizeof(Header) == (sizeof(uint8_t) * 3));

static const constexpr uint8_t MAX_TRANSFER_SIZE = 128;

union Packet {
  struct {
    Header header;
    uint8_t data[MAX_TRANSFER_SIZE - sizeof(Header)];
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};
static_assert(sizeof(Packet) == MAX_TRANSFER_SIZE);
```

A Packet is a buffer of max side 128Bytes composed of a header and the data.

#### Header

Composed by:

1. [Command](#commands)
2. [Device](#devices)
3. Size the number of bytes of the [data](#data)
4. Boolean that indicates if the neuron has more packets to send, this way the keyscanner can pool him until there is no
   more messages in the queue.

#### Data

Data is a buffer of max side 128 - the side of the header.
This will contain the actual message of to send, of example is the command is **HAS_KEYS**, the field size of the header
will be 5 and in the data buffer data[0..4] will be fill with the KeyMatrix data.

### Devices

Devices is the enum that declares which device is sending the packet.
For example in the case of the RF all the communication will go trough only one SPI line, with the device the neuron
will
be able to check if the received packet if from the left side or right side, also the same is applied to the KeyScanner
this way it will know if the communication is with a Defy Wired o Wireless.

```cpp
enum Devices : uint8_t {
  UNKNOWN = 0,
  KEYSCANNER_DEFY_LEFT,
  KEYSCANNER_DEFY_RIGHT,
  NEURON_DEFY_WIRED,
  NEURON_DEFY_WIRELESS,
};
```

### Commands

The commands is the api of communication between the keyscanner and the neuron. This way for example at startup the
keyscanner will send a **CONNECTED** command to the Neuron to indicate that it is the first time the connection has been
made.

Each 100ms a **IS_ALIVE** command will be sent to indicate that the communication is still open.
And the rest of the commands are almost self explanatory. But will be explain in detail down below.

```cpp
enum Commands : uint8_t {
IS_DEAD = 0,
IS_ALIVE,
CONNECTED,
DISCONNECTED,
SLEEP,
WAKE_UP,
GET_VERSION,
SET_ALIVE_INTERVAL,
//Keys
HAS_KEYS = 10,
SET_KEYSCAN_INTERVAL,
//LEDS
SET_BRIGHTNESS = 20,
SET_MODE_LED,
SET_LED,
SET_LED_BANK,
SET_PALETTE_COLORS,
SET_LAYER_KEYMAP_COLORS,
SET_LAYER_UNDERGLOW_COLORS,
GET_OPEN_LED,
GET_SHORT_LED,
};
```

#### IS_DEAD

**Description**: If at some point some device receive this kind of commands is that a error in the communication has
occur because this
is not a valid command.

#### IS_ALIVE

**Description**: Default message to let the neuron know the communication is still open.

**Status**: WORKING

**Data size**: 0 BYTES

#### CONNECTED

**Description**: API endpoint to indicate that it is the first time a communication has establish.
For example this is use to know that a new device its connected and send him the new layers colors.

**Status**: WORKING

**Data size**: 0 BYTES

#### DISCONNECTED

**Description**: API endpoint to indicate that it is the last communication and the keyscanner will shutdown.

**Status**:**TBD**

**Data size**: 0 BYTES

#### SLEEP

**Description**: API endpoint to indicate to the keyscanner that can enter in a sleep state.

**Status**: WORKING

**Data size**: 0 BYTES

#### WAKE_UP

API endpoint to indicate to the keyscanner that can needs to wake_up in a normal state.

**Status**: WORKING

**Data size**: 0 BYTES

#### GET_VERSION

**Description**: API endpoint that returns the actual version of the keyscanner.

**Status**:**TBD**

**Size**: 4 BYTES

#### SET_ALIVE_INTERVAL

API endpoint to configure the amount of time between IS_ALIVE commands.

**Status**:**TBD**

**Size**: 4 BYTES

#### HAS_KEYS

API endpoint to send the state of the key matrix with the actual key presses.

**Status**: WORKING

**Size**: 5 BYTES

#### SET_KEYSCAN_INTERVAL

API endpoint to configure the debouncing time.

**Status**: WORKING

**Size**: 1 BYTE

#### SET_BRIGHTNESS

API endpoint to configure the relative max brightness time.

**Status**: WORKING

**Size**: 1 BYTE

#### SET_MODE_LED

API endpoint to configure the actual led mode.

**Status**: WORKING

**Size**: Different for each led mode BYTE

#### SET_LED

API endpoint to configure the color of a single key.

**Status**: TBD

**Size**: 3 BYTES [ROW,COL,PALLET]

#### SET_LED_BANK

API endpoint to configure the color of a bank of LEDS.

**Status**: TBD

**Size**: X BYTES

SET_PALETTE_COLORS

#### SET_PALETTE_COLORS

API endpoint to configure the palette colors.

**Status**: WORKING

**Size**: 16(MAX PALETTE COLORS)*4(RGBW) = 64 BYTES

#### SET_LAYER_KEYMAP_COLORS

API endpoint to configure the layer colors with palette of just the keymap.

**Status**: WORKING

**Size**: 1(Layer Index) + 35(KeyMap) = 36 BYTES

#### SET_LAYER_UNDERGLOW_COLORS

API endpoint to configure the layer colors with palette of just the underglow.

**Status**: WORKING

**Size**: 1(Layer Index) + 53(KeyMap) = 36 BYTES

#### GET_OPEN_LED

API endpoint to get the open leds status.

**Status**: WORKING

**Size**: 33 BYTES

#### GET_SHORT_LED

API endpoint to get the short leds status.

**Status**: WORKING

**Size**: 33 BYTE