#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <Keypad.h>

#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN);

Servo myServo;

// ----- Keypad Setup -----
const byte ROWS = 4; 
const byte COLS = 4; 
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// Pin assignment (rows = D2, D4, D5, D6; cols = D7, D8, A0, A3)
byte rowPins[ROWS] = {2, 4, 5, 6}; 
byte colPins[COLS] = {7, 8, A0, A3}; 

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ----- Access Data -----
String card1 = "E5 39 B6 C3";   // Replace with your first card UID
String card2 = "AB CD EF 90";   // Replace with your second card UID
String correctPassword = "1234"; // Keypad PIN
String enteredPassword = "";

// ----- LED Pins -----
const int greenLED = A1;  // Green LED
const int redLED   = A2;  // Red LED

bool isOpen = false;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  myServo.attach(3);   // Servo on D3
  closeServo();        // Start locked

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  Serial.println("Scan RFID card or enter PIN:");
}

void loop() {
  // --- RFID Check ---
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uidString = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      uidString += String(rfid.uid.uidByte[i], HEX);
      if (i < rfid.uid.size - 1) uidString += " ";
    }
    uidString.toUpperCase();

    Serial.print("Card detected: ");
    Serial.println(uidString);

    if (uidString == card1 || uidString == card2) {
      Serial.println("Valid RFID card! Access granted.");
      accessGranted();
    } else {
      Serial.println("Invalid RFID card!");
      accessDenied();
    }
    rfid.PICC_HaltA();
  }

  // --- Keypad Check ---
  char key = keypad.getKey();
  if (key) {
    if (key == '#') {  // Enter key
      if (enteredPassword == correctPassword) {
        Serial.println("Correct PIN! Access granted.");
        accessGranted();
      } else {
        Serial.println("Wrong PIN!");
        accessDenied();
      }
      enteredPassword = "";
    } 
    else if (key == '*') { // Clear input
      enteredPassword = "";
      Serial.println("PIN cleared.");
    } 
    else if (key == 'A') { // Manual close
      Serial.println("Manual close requested.");
      closeServo();
    }
    else {
      enteredPassword += key;
      Serial.print("*");  // Masked input
    }
  }
}

// ----- Functions -----
void accessGranted() {
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, LOW);
  myServo.write(90);   // Unlock
  isOpen = true;
}

void accessDenied() {
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  delay(2000);
  digitalWrite(redLED, LOW);
}

void closeServo() {
  myServo.write(0);   // Lock
  isOpen = false;
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
}
