/*
  This allows sending and receiving half-duplex serial commands
  to/from the Hiwonder xArm servo motors.

  According to this...

      https://wiki.seeedstudio.com/Seeeduino-XIAO/

  ... the Seeeduino XIAO's UART interface uses pin 6 for TX,
  pin 7 for RX, and is controlled using the Serial1 object.

  Pin 6 should be connected via a 4.7k resistor to pin 7, and
  pin 7 should be connected to the xArm servo signal line.
  This will cause all transmitted data to be echoed, but the
  echoed bytes will be discarded by the code.
*/


#define UART_TX_PIN 6


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // Set the UART TX pin high so we can receive data from the xArm
  digitalWrite(UART_TX_PIN, true);
}


void loop() {
  fromComputerToArm();
  fromArmToComputer();
}


void fromComputerToArm() {
  if (!Serial.available()) {
    return;
  }

  // Clear the xArm read buffer
  while (Serial1.available()) {
    Serial1.read();
  }

  // Get the number of bytes that need to be read from the computer and sent to the xArm
  const unsigned int data_length = Serial.read();

  // Transfer the specified number of bytes
  for (int i = 0; i < data_length; i++) {
    // Send the byte from the computer to the xArm
    while (!Serial.available());
    Serial1.write(Serial.read());
    Serial1.flush();

    // Discard the echoed byte
    while (!Serial1.available());
    Serial1.read();
  }

  // Set the UART TX pin high so we can receive data from the xArm
  digitalWrite(UART_TX_PIN, true);

  // Let the computer know that we have sent the data
  Serial.write(0xAA);
}


void fromArmToComputer() {
  while (Serial1.available()) {
    // Wait for the send buffer to the computer to open up
    while (!Serial.availableForWrite());

    // Send the byte from the xArm to the computer
    Serial.write(Serial1.read());
  }
}
