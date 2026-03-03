// This example tests the encoding and decoding functionality of the
// ManchesterCodec, NRZCodec, PulseDistanceCodec, and PulseWidthCodec classes.

#include "Codecs.h"

// Define constants for the test
const uint8_t TEST_DATA[] = {0x55, 0xAA, 0xF0, 0x0F};  // Test data to encode

uint16_t speedHz = 100;

void testCodec(Codec& codec, uint16_t speedHz) {
  codec.begin(speedHz);

  // Test encoding
  Vector<OutputEdge> encodedEdges;
  for (uint8_t i = 0; i < sizeof(TEST_DATA); i++) {
    codec.encode(TEST_DATA[i], encodedEdges);
  }

  // Print encoded data
  Serial.print("Encoded Data: ");
  for (size_t i = 0; i < sizeof(TEST_DATA); i++) {
    Serial.print("0x");
    Serial.print(TEST_DATA[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Print encoded edges
  Serial.print("Encoded Edges: ");
  Serial.println(encodedEdges.size());
  // for (const auto& edge : encodedEdges) {
  //   Serial.print("Level: ");
  //   Serial.print(edge.level);
  //   Serial.print(", Pulse: ");
  //   Serial.println(edge.pulseUs);
  // }

  // Test decoding
  Vector<uint8_t> decodedData;
  for (const auto& edge : encodedEdges) {
    uint8_t result = 0;
    if (codec.decodeEdge(edge.pulseUs, edge.level, result)) {
      decodedData.push_back(result);
    }
  }

  // Print decoded data
  Serial.print("Decoded Data: ");
  for (const auto& byte : decodedData) {
    Serial.print("0x");
    Serial.print(byte, HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Verify if decoded data matches the original test data
  bool success = true;
  if (decodedData.size() != sizeof(TEST_DATA)) {
    success = false;
  } else {
    for (size_t i = 0; i < decodedData.size(); i++) {
      if (decodedData[i] != TEST_DATA[i]) {
        success = false;
        break;
      }
    }
  }

  // Print test result
  if (success) {
    Serial.println("Test Passed!");
  } else {
    Serial.println(
        "Test Failed: Decoded data does not match the original data.");
  }
}

void setup() { Serial.begin(115200); }

void loop() {
  Serial.println("Testing codecs at speed: " + String(speedHz) + " Hz");
  Serial.println("- ManchesterCodec");
  ManchesterCodec manchester;
  testCodec(manchester, speedHz);

  Serial.println("- NRZCodec");
  NRZCodec nrz;
  testCodec(nrz, speedHz);

  Serial.println("- PulseDistanceCodec");
  PulseDistanceCodec pd;
  testCodec(pd, speedHz);

  Serial.println("- PulseWidthCodec");
  PulseWidthCodec pw;
  testCodec(pw, speedHz);

  speedHz += 10;
  Serial.println("----------------------------------------");
  delay(1000);
}