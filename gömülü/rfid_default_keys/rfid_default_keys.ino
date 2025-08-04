#include <SPI.h>
#include <MFRC522.h>

// ---- Pinler ----
static const uint8_t SS_PIN   = D10;  // RC522 SDA
static const uint8_t RST_PIN  = D4;   // RC522 RST
static const uint8_t SCK_PIN  = D5;   // RC522 SCK
static const uint8_t MISO_PIN = D6;   // RC522 MISO
static const uint8_t MOSI_PIN = D7;   // RC522 MOSI

// ---- Nesne ----
MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  delay(200);

  // SPI başlat
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  // RC522 başlat
  mfrc522.PCD_Init();
  delay(50);

  Serial.println(F("RC522 hazir! Bir kart okutun..."));
  mfrc522.PCD_DumpVersionToSerial();
}

void loop() {
  // Kart var mı?
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // UID okunabiliyor mu?
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // UID yazdır
  Serial.print(F("Kart UID: "));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(i == mfrc522.uid.size - 1 ? "" : " ");
  }
  Serial.println();

  // Kart tipi
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  Serial.print(F("Kart tipi: "));
  Serial.println(mfrc522.PICC_GetTypeName(piccType));

  // Kart ile işi bitir
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
