// This example tests the encoding and decoding functionality of the
// ManchesterCodec inclding a preamble.

#include "Codecs.h"
#include "Preambles.h"

// Define constants for the test
const uint8_t TEST_DATA[] = {0x01, 0x02, 0x03, 0x04};  // Test data to encode

uint16_t speedHz = 100;
ManchesterPreamble preamble;
ManchesterCodec manchester(preamble);

void testCodec(Codec& codec, uint16_t speedHz) {
  codec.begin(speedHz);


  // Test encoding
  Vector<OutputEdge> encodedEdges;
  codec.encodePreamble(encodedEdges);
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
  for (const auto& edge : encodedEdges) {
    Serial.print("Level: ");
    Serial.print(edge.level);
    Serial.print(", Pulse: ");
    Serial.println(edge.pulseUs);
  }

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

void setup() {
  Serial.begin(115200);
  delay(3000);
  Logger::setLogLevel(Logger::LOG_LEVEL_DEBUG);
  Serial.println("Testing codecs at speed: " + String(speedHz) + " Hz");
  Serial.println("- ManchesterCodec");
  testCodec(manchester, speedHz);
}

void loop() {}