# Communication Defy Protocol

This documentation will explain the protocol used between the keyscanner and the neuron.

## Protocol

All the protocol packets and communications are declared in
the [Communications_protocol.h](https://github.com/Dygmalab/DefyCommunications/blob/main/src/Communications_protocol.h).

https://github.com/Dygmalab/DefyCommunications/blob/main/src/Communications_protocol.h

<div align="center">
  <img alt="UML Diagram" src="http://www.plantuml.com/plantuml/svg/fPHDivem58RN-Ykoi-qThpDTTTn5E6f6J9WaOcsda-5YghG23c0NlTF_NW88UD5kcXNdEPzvGp8P8VRoGmTvXaQJuJ3s8vsT_K0ZAucYInm6VXucSQREQP8dGN92hyE1ZYzHwVVZVLO0L2yYb0tOxjhO8Huqm6AKWYMX3R69w4FX0BYDjSKhK9lMze0BmcXZboqKeHAuXvq6Bx5GAzY91XZ3mhGVEUDaiPGKH1kxPZOefpDIzzl53aW9oc8EussEWtV0JSirTk_xDjG6ld3Ozgrx0L8n5-XDImF5ad5vGsUuQCrt1VF0wL4XiTo8FiOUeOi6yxd2jitDXWT_XlLvslfN66Xpd8F1XgueszAtqj5Y3pRCJVEvVEWirUpswgTSuMqeXGrdjFHq8-_XBU5Wt_NC7BZoZtrFVvI6LhEKUWGdFylGKljxdPPwF0NLpuwcpMzlwBwKh82rTatHfwEVgIX9jIgkqazTtAqfUacIKnkGXRzDuYMCyqygRtcG1XtScSOj2rNJ7Al9flM8vIX-xhU9w2jQuyzASap5l3WNGRu06fkUoVTtLTBxRwPwKT-ioGYH-AZJCFVZOaEfFbMt_XYUgsUXYi67GweFfUjEp8C7uxdMySDu_EXDUMv4_QygrrDuNAGMyLz5Qaclwoy0">
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
  uint8_t crc;
};
static_assert(sizeof(Header) == (sizeof(uint8_t) * 4));

static const constexpr uint8_t MAX_TRANSFER_SIZE = 32;

union Packet {
  struct {
    Header header;
    uint8_t data[MAX_TRANSFER_SIZE - sizeof(Header)];
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};
static_assert(sizeof(Packet) == MAX_TRANSFER_SIZE);
```

A Packet is a buffer of max side 32Bytes composed of a header and the data.

#### Header

Composed by:

1. [Command](#commands)
2. [Device](#devices)
3. Size the number of bytes of the [data](#data)
4. Boolean that indicates if the neuron has more packets to send, this way the keyscanner can pool him until there is no
   more messages in the queue.
5. CRC verification of the data

#### Data

Data is a buffer of max side 32 - the side of the header.
This will contain the actual message of to send, of example is the command is **HAS_KEYS**, the field size of the header
will be 5 and in the data buffer data[0..4] will be fill with the KeyMatrix data.

### Devices

Devices the enum that declares which device is sending the packet.
For example, in the case of the RF all the communication will go through only one SPI line, with the device the neuron
will be able to check if the received packet if from the left side or right side, also the same is applied to the KeyScanner
this way it will know if the communication is with a Defy Wired o Wireless.

```cpp
enum Devices : uint8_t {
  UNKNOWN = 0,
  KEYSCANNER_DEFY_LEFT,
  KEYSCANNER_DEFY_RIGHT,
  RF_DEFY_LEFT,
  RF_DEFY_RIGHT,
  NEURON_DEFY,
  RF_NEURON_DEFY,
  WIRED_NEURON_DEFY,
  BLE_NEURON_2_DEFY,
  BLE_DEFY_LEFT,
  BLE_DEFY_RIGHT,
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
  VERSION,
  ALIVE_INTERVAL,
  //Keys
  HAS_KEYS = 10,
  KEYSCAN_INTERVAL,
  //LEDS
  BRIGHTNESS = 20,
  MODE_LED,
  LED,
  PALETTE_COLORS,
  LAYER_KEYMAP_COLORS,
  LAYER_UNDERGLOW_COLORS,
  GET_OPEN_LED,
  GET_SHORT_LED,
  //Battery
  BATTERY_LEVEL = 40,
  BATTERY_STATUS,
  BATTERY_SAVING,
  //Config
  RF_ADDRESS = 50,
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

#### VERSION

**Description**: API endpoint that returns the actual version of the keyscanner.

**Status**: WORKING

**Size**: 4 BYTES

#### ALIVE_INTERVAL

API endpoint to configure the amount of time between IS_ALIVE commands.

**Status**:**TBD**

**Size**: 4 BYTES

#### HAS_KEYS

API endpoint to send the state of the key matrix with the actual key presses.

**Status**: WORKING

**Size**: 5 BYTES

#### KEYSCAN_INTERVAL

API endpoint to configure the debouncing time.

**Status**: WORKING

**Size**: 1 BYTE

#### BRIGHTNESS

API endpoint to configure the relative max brightness time.

**Status**: WORKING

**Size**: 1 BYTE

#### MODE_LED

API endpoint to configure the actual led mode.

**Status**: WORKING

**Size**: Different for each led mode BYTE

#### LED

API endpoint to configure the color of a single key.

**Status**: TBD

**Size**: 3 BYTES [ROW,COL,PALLET]

#### LED_BANK

API endpoint to configure the color of a bank of LEDS.

**Status**: TBD

**Size**: X BYTES

SET_PALETTE_COLORS

#### PALETTE_COLORS

API endpoint to configure the palette colors.

**Status**: WORKING

**Structure**: [Starting Index][Data]

**Size**: 16 colors*4(bytes of RGBW) splited in messages of 27 bytes

#### LAYER_KEYMAP_COLORS

API endpoint to configure the layer colors with palette of just the keymap.

**Status**: WORKING

**Structure**: [Layer][Data]

**Size**: 35 keys merged in 27 bytes as one color is repressed only by 4bits, then 35/2 is 18.

#### LAYER_UNDERGLOW_COLORS

API endpoint to configure the layer colors with palette of just the underglow.

**Structure**: [Layer][Data]

**Size**: 53 keys merged in 18 bytes as one color is repressed only by 4bits, then 53/2 is 28.


#### OPEN_LED

API endpoint to get the open leds status.

**Status**: WORKING

**Size**: 33 BYTES

#### SHORT_LED

API endpoint to get the short leds status.

**Status**: NOT WORKING (Overflow of data)

**Size**: 33 BYTE

#### SHORT_LED

API endpoint to get the short leds status.

**Status**: NOT WORKING (Overflow of data)

**Size**: 33 BYTE

#### BATTERY_LEVEL

API endpoint to get/send battery level in %

**Status**: WORKING

**Size**: 1 BYTE

#### BATTERY_STATUS

API endpoint to get/send battery status in %

**Status**: WORKING

**Size**: 1 BYTE

#### RF_ADDRESS

API endpoint to set the RF ADDRESS used to unique message sending between defys.

**Status**: WORKING

**Size**: 4 BYTE

## Library

### Callback

https://github.com/Dygmalab/DefyCommunications/blob/main/src/Callback.h

Callback is the library used to notify the system of the different commands received, it contains 2 classes Callback and
BindingCallback.

[Callback](#example-of-callback) takes as a diamond operator the variable of the callback, and in provides a method to
add a function call back
o to remove them, finally, and actualization can be notified with the () operator.

[BindingCallback](#example-of-bindingcallbacks) is an extension of takes 2 arguments as a diamond operator first the
variable of the and identifier
could be an enum, integer or string and second the actual type associated with this identifier.
The behaviour is a mix of a hashmap but with the output being the call of a callback.

#### Example of Callback

```cpp
void notifySystem(bool now_active) {
  printf("The system is now: ");
  if (now_active)
	printf("active\n");
  else
	printf("inactive\n");
}

int main() {
  Callback<bool> active_callback;

  active_callback.addListener(notifySystem);

  active_callback.addListener([](bool now_active) {
	printf("The motor is now: ");
	if (now_active)
	  printf("active\n");
	else
	  printf("inactive\n");
  });

  active_callback(false);
  active_callback(true);
}
```

#### Output

```bash
The system is now: inactive
The motor is now: inactive
The system is now: active
The motor is now: active
```

#### Example of BindingCallbacks

```cpp
void checkBrakeValue(uint32_t brake) {
  printf("Update of the force at the brake now at %u\n", brake);
}

int main() {
  BindingCallbacks<Commands, uint32_t> bc;

  bc.bind(MOTOR,
		  ([](uint32_t speed) {
			printf("Update of the speed of the motor now at %u\n", speed);
		  }));

  bc.bind(WHEEL,
		  ([](uint32_t rotation) {
			printf("Update of the rotation of the right wheel now at %u\n", rotation);
		  }));

  bc.bind(WHEEL,
		  ([](uint32_t rotation) {
			printf("Update of the rotation of the left wheel now at %u\n", rotation);
		  }));

  bc.bind(BRAKE, checkBrakeValue);

  bc.call(MOTOR, 60);
  bc.call(WHEEL, 90);
  bc.call(BRAKE, 10);
}
```

#### Output

```bash
Update of the speed of the motor now at 60
Update of the rotation of the right wheel now at 90
Update of the rotation of the left wheel now at 90
Update of the force at the brake now at 10
```

Keyscanner code using bindingCallbacks

```cpp
callbacks.bind(GET_SHORT_LED, [this](Packet p) {
p.header.device = device;
p.header.size   = IS31FL3743B::get_short_leds(p.data);
sendPacket(p);
});

callbacks.bind(GET_OPEN_LED, [this](Packet p) {
p.header.device = device;
p.header.size   = IS31FL3743B::get_open_leds(p.data);
sendPacket(p);
});

callbacks.bind(SET_BRIGHTNESS, [](Packet p) {
LEDManagement::setMaxBrightness(p.data[0]);
});

callbacks.bind(SET_MODE_LED, [](Packet p) {
LEDManagement::set_led_mode(p.data);
});
```

#### Output

```bash
Update of the speed of the motor now at 60
Update of the rotation of the right wheel now at 90
Update of the rotation of the left wheel now at 90
Update of the force at the brake now at 10
```

## Communication flow
The comunnications flow is based on callbacks to handle incoming requests and responses, this communication is the same for Wired RF and BLE, the only difeference is the physical transport layer.
- Wired: Uses the spi layer, and with the is_alive commands check if the flag has_more_packets is activated, this way the keyscanner will keep asking the neuron.
- RF: Uses the RFGW layer, as this communication is full duplex it does not need the has_more_packets.
- BLE: Uses the RFGW layer acting the left side as a host instead of a device and forwards the messages via SPI, the messages of the left goes via SPI.

This is how the normal communication will work.


![Example communications](http://www.plantuml.com/plantuml/svg/fPHTJuCm58Rl-HLDlD54apc_ZLdIGPrah2o0qrsHoihMECiidMR_NXJSq5WuYNU5yvod9tdRhfZI8X4h9ZKQyasUC2cvGZStQ7zuk6VRJgVVFK2zdhMgWPIlLIO_8NguXntdk68icMcEb9WlDChIkkyQqKoBT8FOKhnngrOvwdthhf4R8kov4s9LVuhQ3yUsRkYtVKg97H779DW13PFQMR58j2ZwUQxPqrBa2pvhwgXzAFwwBhfob6Go5VErOb-YL6KlrPoMQVf-u0xk8ah2q4ZWA2B1D0uZ7DstKXwP4AzLXISk7PYeCVP8mPdAjsuGHh67foI8XsGwmkFOzZq_WF3tr271mFCVuYeKrwUcFeyuySXtZ7DGybXf6udjbcMMIvv-HRc3oeNY-IgJinoz2xtubUECQMOwNBNyJCJF_5geChysLPcVWVhEpr0svWVW_ypFqBVDpr3VvOUwZSnsqnQkT9q0NIS0rodGTGfmdG5TPm3NET1r3d1T05qN0DSbq7K9S5q1NLS0rt47A2l0XXS_v78cv1mzymrA5apEY-U4RRWolJztEFoyOQqNv7_j7m00)
