### Preambles

A preamble is a unique sequence of pulses that is used to mark and recognize the beginning of a new frame:

- Preambles are protocol-specific and injected via a Preamble interface.
- Each codec uses its own preamble detector for reliable synchronization.
- Transmission uses the codec’s preamble generator logic for extensibility.

