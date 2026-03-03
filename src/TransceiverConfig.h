#pragma once

#include <stdint.h>

/// platforms with analogWriteFrequency support for PWM carrier modulation
#if defined(ESP32)
#define HAS_SET_PMW_FREQUENCY2
#define HAS_INTERRUPT_ARG
#define HAS_STL
#endif

#if defined(ESP8266)
#define HAS_SET_PMW_FREQUENCY1
#define HAS_INTERRUPT_ARG
#define HAS_STL
#endif


#if defined(ARDUINO_ARCH_STM32)
#define HAS_SET_PMW_FREQUENCY1
#define HAS_STL
#endif

/// platforms with attachInterruptArg support for cleaner interrupt handling
#if  defined(ARDUINO_ARCH_RP2040)
#define HAS_STL
#define HAS_SET_PMW_FREQUENCY1
#endif

#define USE_RAW_DELAY false

namespace pulsewire {

/// Default carrier frequency for IR transmission (38 kHz is common for
/// consumer IR)
static const uint32_t CARRIER_HZ = 38000;

/// Default bit frequency for Manchester encoding (1 kHz is a common starting
/// point) = baud rate
static const uint32_t DEFAULT_BIT_FREQ_HZ = 1000;

/// Default frame size for RX/TX drivers and single-char buffer
static const uint16_t DEFAULT_FRAME_SIZE = 256;

}  // namespace pulsewire

// For Arduino sketches, use pulsewire::Transceiver explicitly or add 'using' in
// your sketch if desired.
#ifdef ARDUINO
using namespace pulsewire;
#endif
