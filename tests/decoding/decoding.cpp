
#include "Codecs.h"
#include "Preambles.h"

using namespace pulsewire;

int baud = 9000;
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);

const int edges[][2] = {
    {0, 3034837}, {1, 62}, {0, 94},  {1, 81},  {0, 63},  {1, 94}, {0, 62},
    {1, 62},      {0, 62},  {1, 78},  {0, 62},  {1, 62},  {0, 65}, {1, 84},
    {0, 68},      {1, 67},  {0, 62},  {1, 83},  {0, 61},  {1, 62}, {0, 62},
    {1, 62},      {0, 62},  {1, 62},  {0, 62},  {1, 62},  {0, 62}, {1, 62},
    {0, 63},      {1, 61},  {0, 62},  {1, 62},  {0, 118}, {1, 91}, {0, 62},
    {1, 62},      {0, 63},  {1, 61},  {0, 62},  {1, 62},  {0, 62}, {1, 62},
    {0, 62},      {1, 62},  {0, 62},  {1, 118}, {0, 119}, {1, 63}, {0, 62},
    {1, 62},      {0, 63},  {1, 61},  {0, 62},  {1, 62},  {0, 62}, {1, 62},
    {0, 62},      {1, 118}, {0, 119}, {1, 61},  {0, 63},  {1, 62}, {0, 62},
    {1, 62},      {0, 62},  {1, 80},  {0, 62},  {1, 62},  {0, 62}, {1, 62},
    {0, 63},      {1, 117}, {0, 62},  {1, 62},  {0, 118}, {1, 64}, {0, 62},
    {1, 62},      {0, 62},  {1, 62},  {0, 62},  {1, 62},  {0, 62}, {1, 118},
    {0, 118},     {1, 62},  {0, 62}};

void setup() {
  Serial.begin(115200);
  Logger::setLogLevel(Logger::LOG_LEVEL_DEBUG);
}

void loop() {
  Serial.print("Decoding test: baud=");
  Serial.println(baud);

  codec.begin(baud);
  Vector<uint8_t> result_chars;
  for (const auto& edge : edges) {
    uint8_t result = 0;
    if (codec.decodeEdge(edge[1], edge[0], result)) {
      result_chars.push_back(result);
    }
  }
  // print results
  Serial.print("Decoded bytes:");
  for (uint8_t byte : result_chars) {
    Serial.print("0x");
    Serial.print(byte, HEX);
    Serial.print(" ");
  }
  Serial.println();

  while (true) delay(1000);
}
