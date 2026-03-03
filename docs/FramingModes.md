
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

- Flush:

Data is buffered until explicitly flushed.
The flush() method is used to finalize the frame and send the buffered data.
This mode is useful for scenarios where the frame size is dynamic or determined at runtime.

