
### FramingMode

The FramingMode determines how data is transmitted and framed during communication. It defines the behavior of the transmitter when handling data and ensures that the receiver can correctly interpret the transmitted frames. The library supports the following framing modes:

- WriteBytes (Default Mode):

Data is transmitted immediately as it is written to the buffer.
Each write operation triggers the transmission of the corresponding data.
This mode is suitable for low-latency applications where data needs to be sent as soon as it is available.

- FixedSize:

Data is transmitted in fixed-size frames.
The frame size is pre-configured, and the transmitter waits until the buffer is full before sending the frame. 
This mode is ideal for applications where the frame size is known in advance and consistent.


#### Example Code


```C++
#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t rxPin = 22;
const uint8_t txPin = 23;
int baud = 8000;
DigitalSignal digitalSignal;
ManchesterCodec codec;
TxDriverArduino tx(codec, txPin, digitalSignal);
RxDriverArduino rx(codec, rxPin);
Transceiver transceiver(rx, tx);

// set up framing to Fixed Size 1024 bytes
transceiver.setFrameSize (1024);
transceiver.setFramingMode(FramingMode::FixedSize);
```

