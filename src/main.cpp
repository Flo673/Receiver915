#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ==== LoRa Setup (Adafruit Feather M0 LoRa / RFM95, antenna on U.FL) ====
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define LORA_FREQUENCY 915E6  // must match both transmitters

// Board 1 (calibrationBoard1): MPRLS pressure + SHT45 temp/humidity + INIR-ME100
// (UART+analog) + battery.
#define BOARD1_FIELD_COUNT 8
// Board 2 (calibrationBoard2): INIR-CD100 (CO2) + INIR-ME100 (analog-only) + battery.
#define BOARD2_FIELD_COUNT 7
// Board 3 (Transmitter915): bare RSSI-test transmitter, no sensors -- just a
// packet counter + battery voltage so packets are easy to count for loss/RSSI checks.
#define BOARD3_FIELD_COUNT 2

// The LoRa library's packetRssi() is a simplified "PktRssi register - 157", which drops
// both terms the SX1276 datasheet (section 5.5.5) asks for: the 16/15 scaling when
// SNR >= 0, and the SNR term when SNR < 0. That is only worth a few dB, but when the
// whole point of a packet is comparing antennas, report the datasheet number too.
// rawPktRssi recovers the register value the library already subtracted 157 from
// (157 = the HF-port offset it applies above 525 MHz, so 915 MHz included).
float datasheetRssi(int libraryRssi, float snr) {
  int rawPktRssi = libraryRssi + 157;
  if (snr < 0) {
    return -157.0f + rawPktRssi + snr;
  }
  return -157.0f + (16.0f / 15.0f) * rawPktRssi;
}

// Splits s on commas into up to maxFields entries of fields[], and returns the
// total number of fields found (which may exceed maxFields if s has more
// commas than expected -- callers should check the count before using fields).
int splitCsv(const String &s, String *fields, int maxFields) {
  int count = 0;
  int start = 0;
  while (true) {
    int comma = s.indexOf(',', start);
    if (comma == -1) {
      if (count < maxFields) fields[count] = s.substring(start);
      count++;
      break;
    }
    if (count < maxFields) fields[count] = s.substring(start, comma);
    count++;
    start = comma + 1;
  }
  return count;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
  while (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed, retrying...");
    delay(1000);
  }

  Serial.println("# board 1: board_id,pressure_hpa,sht_temp_c,sht_humidity_pct,inir_a0_voltage,inir_ch4_analog_pctvol,inir_ch4_digital_pctvol,inir_temp_c,battery_voltage,rssi,snr");
  Serial.println("# board 2: board_id,inircd100_voltage,inircd100_pctvol,inircd100_digital_pctvol,inircd100_temp_c,inirme100_voltage,inirme100_pctvol,battery_voltage,rssi,snr");
  Serial.println("# board 3: board_id,packet_count,battery_voltage,rssi,snr,rssi_datasheet,noise_floor");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return;
  }

  String payload;
  while (LoRa.available()) {
    payload += (char)LoRa.read();
  }

  // Payload format: "<board_id>,<board-specific fields...>" -- board_id "1" or "2"
  // selects which fixed-width schema the rest of the payload is parsed as.
  int firstComma = payload.indexOf(',');
  if (firstComma == -1) {
    return;
  }
  String boardId = payload.substring(0, firstComma);
  String rest = payload.substring(firstComma + 1);

  long rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (boardId == "1") {
    String fields[BOARD1_FIELD_COUNT];
    if (splitCsv(rest, fields, BOARD1_FIELD_COUNT) != BOARD1_FIELD_COUNT) {
      return;
    }

    float pressureHpa = fields[0].toFloat();
    float shtTempC = fields[1].toFloat();
    float shtHumidityPct = fields[2].toFloat();
    float inirA0Voltage = fields[3].toFloat();
    float inirCh4AnalogPctVol = fields[4].toFloat();
    float inirCh4DigitalPctVol = fields[5].toFloat();
    float inirTempC = fields[6].toFloat();
    float batteryVoltage = fields[7].toFloat();

    Serial.print(boardId);
    Serial.print(",");
    Serial.print(pressureHpa, 2);
    Serial.print(",");
    Serial.print(shtTempC, 2);
    Serial.print(",");
    Serial.print(shtHumidityPct, 2);
    Serial.print(",");
    Serial.print(inirA0Voltage, 3);
    Serial.print(",");
    Serial.print(inirCh4AnalogPctVol, 2);
    Serial.print(",");
    Serial.print(inirCh4DigitalPctVol, 4);
    Serial.print(",");
    Serial.print(inirTempC, 1);
    Serial.print(",");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.println(snr, 2);
  } else if (boardId == "2") {
    String fields[BOARD2_FIELD_COUNT];
    if (splitCsv(rest, fields, BOARD2_FIELD_COUNT) != BOARD2_FIELD_COUNT) {
      return;
    }

    float inircd100Voltage = fields[0].toFloat();
    float inircd100PctVol = fields[1].toFloat();
    float inircd100DigitalPctVol = fields[2].toFloat();
    float inircd100TempC = fields[3].toFloat();
    float inirme100Voltage = fields[4].toFloat();
    float inirme100PctVol = fields[5].toFloat();
    float batteryVoltage = fields[6].toFloat();

    Serial.print(boardId);
    Serial.print(",");
    Serial.print(inircd100Voltage, 3);
    Serial.print(",");
    Serial.print(inircd100PctVol, 2);
    Serial.print(",");
    Serial.print(inircd100DigitalPctVol, 4);
    Serial.print(",");
    Serial.print(inircd100TempC, 1);
    Serial.print(",");
    Serial.print(inirme100Voltage, 3);
    Serial.print(",");
    Serial.print(inirme100PctVol, 2);
    Serial.print(",");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.println(snr, 2);
  } else if (boardId == "3") {
    String fields[BOARD3_FIELD_COUNT];
    if (splitCsv(rest, fields, BOARD3_FIELD_COUNT) != BOARD3_FIELD_COUNT) {
      return;
    }

    long packetCount = fields[0].toInt();
    float batteryVoltage = fields[1].toFloat();

    // Ambient channel level now that the packet is out of the FIFO. If this sits only
    // a few dB below the packet RSSI, the link is noise-limited; if the packet RSSI is
    // implausibly low while the noise floor is far below it, suspect the RF path
    // (U.FL seating, coax, antenna placement) rather than the radio config.
    int noiseFloor = LoRa.rssi();

    Serial.print(boardId);
    Serial.print(",");
    Serial.print(packetCount);
    Serial.print(",");
    Serial.print(batteryVoltage, 2);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.print(snr, 2);
    Serial.print(",");
    Serial.print(datasheetRssi(rssi, snr), 1);
    Serial.print(",");
    Serial.println(noiseFloor);
  }
  // Unknown board_id: silently ignore.
}
