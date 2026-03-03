
#include "Preambles.h"

uint16_t speedHz = 100;

void setup() { Serial.begin(115200); }

void testPreamble(Preamble& preamble, uint16_t speedHz) {
  preamble.begin(speedHz);

  Vector<OutputEdge> edges;
  int len = preamble.getEdges(edges);

  for (size_t i = 0; i < edges.size(); ++i) {
    Serial.print("Edge ");
    Serial.print(i);
    Serial.print(": level=");
    Serial.print(edges[i].level);
    Serial.print(", pulseUs=");
    Serial.println(edges[i].pulseUs);

    bool success = preamble.detect(edges[i]);
    
    if ((success && i == edges.size() - 1) || !success) {
      Serial.println("Test Passed");
    } else {
      Serial.println("Test Failed: data does not match the original data.");
    }
  }
}

void loop() {
  Serial.println("Testing at speed: " + String(speedHz) + " Hz");
  Serial.println("- Manchester");
  ManchesterPreamble manchesterPreamble;
  testPreamble(manchesterPreamble, speedHz);

  Serial.println("- CustomPreamble");
  CustomPreamble customPreamble;
  customPreamble.addEdge(true, 1);   // HIGH for 1 pulse
  customPreamble.addEdge(false, 2);  // LOW for 2 pulses
  customPreamble.addEdge(true, 3);   // HIGH for 3 pulses
  customPreamble.addEdge(false, 4);  // LOW for 4 pulses
  testPreamble(customPreamble, speedHz);

  Serial.println("- NEC");
  testPreamble(IRProtocolNEC, speedHz);

  speedHz += 10;
}