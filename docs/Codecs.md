## Supported Codecs

### ManchesterCodec (Manchester Encoding)

Uses Manchester encoding: each bit is represented by a transition. '1' is LOW-to-HIGH, '0' is HIGH-to-LOW within the bit period. Ensures frequent transitions for reliable clock recovery.

### DifferentialManchesterCodec (Differential Manchester Encoding)

Implements Differential Manchester encoding, where each bit period contains a transition in the middle:

- Logical '0': transition at the start of the bit period
- Logical '1': no transition at the start of the bit period

This encoding is robust for clock recovery and is used in protocols such as IRDA and some industrial communication standards. The implementation maintains separate state for encoding and decoding to ensure correct transitions.

### NRZCodec (Non-Return-to-Zero)

Implements classic NRZ line coding, where '1' is HIGH and '0' is LOW. Efficient for serial-like data transmission, supports configurable stop bits, and uses a preamble for frame synchronization.

### PulseWidthCodec (Pulse Width Modulation)

Represents bits by varying pulse widths. Typically, a short pulse is '0' and a long pulse is '1'. Simple and robust for noisy channels.

### PulseDistanceCodec (Pulse Distance Modulation)

The PulseDistanceCodec encodes data by varying the distance (time interval) between consecutive pulses. Each bit is represented by a pulse, and the duration between pulses determines whether the bit is a '0' or a '1'. This method is robust against signal level shifts and is commonly used in infrared remote control protocols. It is well-suited for environments with variable signal amplitude or where only timing information is reliable.


#### Example Code


```C++
#include "Codecs.h"

ManchesterCodec codec;
```

Or you can set it up with a preamble:

```C++
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
```
