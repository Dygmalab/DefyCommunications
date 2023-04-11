#ifdef NRF52_ARCH
#include "Communications.h"
#include "SpiPort.h"
#include "Time_counter.h"
#include "Usb_serial.h"


extern Time_counter time_counter;

#if COMPILE_SPI0_SUPPORT
static SpiPort spiPort0(0);
Devices spiPort0Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort0LastCommunication{0};
#endif

#if COMPILE_SPI1_SUPPORT
static SpiPort spiPort1(1);
static Devices spiPort1Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort1LastCommunication{0};
#endif

#if COMPILE_SPI2_SUPPORT
static SpiPort spiPort2(2);
static Devices spiPort2Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort2LastCommunication{0};
#endif

constexpr static uint32_t timeout = 200;

void checkActive();

void Communications::init() {

#if COMPILE_SPI0_SUPPORT
  spiPort0.init();
#endif

#if COMPILE_SPI1_SUPPORT
  spiPort1.init();
#endif

#if COMPILE_SPI2_SUPPORT
  spiPort2.init();
#endif
}


void Communications::run() {

  Packet packet{};
#if COMPILE_SPI0_SUPPORT
  if (spiPort0.readPacket(packet)) {
    spiPort0LastCommunication = time_counter.get_millis();
    spiPort0Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort1.readPacket(packet)) {
    spiPort1Device            = packet.header.device;
    spiPort1LastCommunication = time_counter.get_millis();
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort2.readPacket(packet)) {
    spiPort2LastCommunication = time_counter.get_millis();
    spiPort2Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif

  checkActive();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  packet.header.device   = Communications_protocol::NEURON_DEFY;
  if (device_to_send == UNKNOWN) {
#if COMPILE_SPI0_SUPPORT
    if (spiPort0Device != UNKNOWN)
      spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
    if (spiPort1Device != UNKNOWN)
      spiPort1.sendPacket(packet);
#endif
#if COMPILE_SPI2_SUPPORT
    if (spiPort2Device != UNKNOWN)
      spiPort2.sendPacket(packet);
#endif
  }

#if COMPILE_SPI0_SUPPORT
  if (spiPort0Device == device_to_send)
    spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
  if (spiPort1Device == device_to_send)
    spiPort1.sendPacket(packet);
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device == device_to_send)
    spiPort2.sendPacket(packet);
#endif
  return true;
}

void checkActive() {
  bool now_active;
  Packet packet;

#if COMPILE_SPI0_SUPPORT
  if (spiPort0Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort0LastCommunication <= timeout;
  if (!now_active) {
    spiPort0Device = UNKNOWN;
    while (spiPort0.readPacket(packet)) {}
    return;
  }
#endif

#if COMPILE_SPI1_SUPPORT
  if (spiPort1Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort1LastCommunication <= timeout;
  if (!now_active) {
    spiPort1Device = UNKNOWN;
    while (spiPort2.readPacket(packet)) {}
    return;
  }
#endif

#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort2LastCommunication <= timeout;
  if (!now_active) {
    spiPort2Device = UNKNOWN;
    while (spiPort2.readPacket(packet)) {}
    return;
  }
#endif
}


class Communications Communications;
#endif