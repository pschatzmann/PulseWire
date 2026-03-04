### Bit Frequency

The Bit frequency (in Hz) determines how many bits are transmitted per second and is a key parameter for reliable communication, whether you are using wired transmission, infrared (IR), or “433 MHz RF modules”. For Manchester encoding, the bit frequency is equivalent to the baud rate.

#### Typical Values

- **Wired Data Transfer:**
  - Bit frequencies can range from a few hundred Hz up to several kHz, depending on cable quality, noise, and hardware capabilities. Lower frequencies are more robust against interference and signal degradation.

- **Infrared (IR) Communication:**
  - Consumer IR Remotes: 300–2400 Hz:
    Most TV, audio, and appliance remotes use bit frequencies in this range for robust operation and compatibility with standard IR receivers.
  - Arduino/ESP32 Projects: 1000–2000 Hz:
    These values are recommended for custom IR projects, balancing speed and reliability. Lower frequencies are easier for simple receivers to decode, while higher frequencies allow faster data transfer but require precise timing.
  - ESP32 RMT Hardware: Up to 4000 Hz:
    The ESP32 RMT peripheral can handle higher bit frequencies, but practical limits depend on receiver hardware and software. For reliable communication, stay within 1000–2000 Hz unless you have specialized receivers.

- **433 MHz RF Modules:**
  - Typical bit frequencies: 1000–4800 Hz:
    These modules are commonly used for wireless data transfer. Lower frequencies improve range and noise immunity, while higher frequencies allow faster data transfer but require better synchronization and signal quality.

#### Choosing a Bit Frequency
- Match the bit frequency between transmitter and receiver for all communication types.
- Use lower frequencies for longer range, better noise immunity, and simpler hardware.
- Use higher frequencies for faster data transfer, but ensure your receiver and transmission medium can handle it reliably.
- Consider environmental factors (noise, interference, signal attenuation) and hardware limitations when selecting the optimal bit frequency.


#### Example Code

```C++
#include "Transceiver.h"

Transceiver transceiver;
// strting with 1000 baud (bits/second)
transceiver.begin(1000);
```