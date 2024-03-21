/*
 * AMT24_Single_Turn_RS485_Sample_Code_Mega.ino
 * Company: CUI Devices
 * Author: Damon Tarry
 * Date: March 19, 2024
 *
 * This sample code can be used with the Arduino Mega to control the AMT24 encoder.
 * It uses one UART to control the encoder and the the another UART to report back to the PC
 * via the Arduino Serial Monitor.
 * For more information or assistance contact CUI Devices for support.
 *
 * After uploading code to Arduino Mega 2560 open the open the Serial Monitor under the Tools
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT24.
 *
 * This code is compatible with most Arduino boards, but it is important to verify the board
 * to be used has an available serial port for the RS485 connection.
 *
 * Arduino Pin Connections
 * TX:          Pin D19
 * RX:          Pin D18
 * REn:         Pin D8
 * DE:          Pin D9
 *
 *
 * AMT24 Pin Connections
 * NC:          Pin  1
 * NC:          Pin  2
 * A:           Pin  3
 * B:           Pin  4
 * NC:          Pin  5
 * NC:          Pin  6
 * GND:         Pin  7
 * Vdd (5V):    Pin  8
 *
 *
 * Required Equipment Note:
 * The AMT24 requires a high speed RS485 transceiver for operation. At the time this sample
 * code was written Arduino does not sell any compatible RS485 transceivers. For this demo
 * the SP3485 RS485 transceiver was used on a breakout board.
 *
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/* Serial rates for UART */
#define USB_BAUDRATE         115200
#define RS485_BAUDRATE       2000000

/* We will use this define macro so we can write code once compatible with 12 or 14 bit encoders */
#define RESOLUTION            14

/* The AMT24 encoder is able to have a range of different values for its node address. This allows there
 * to be multiple encoders on on the RS485 bus. The encoder will listen for its node address, and that node
 * address doubles as the position request command. This simplifies the protocol so that all the host needs
 * to do is transmit the node address and the encoder will immediately respond with the position. The node address
 * can be any 8-bit number, where the bottom 2 bits are both 0. This means that there are 63 available addresses.
 * The bottom two bits are left as zero, because those bit slots are used to indicate extended commands. Taking
 * the node address and adding 0x01 changes the command to reading the turns counter (multi-turn only), and adding
 * 0x02 changes the command to zero the position. 0x03 is used to reset the encoder.
 */
#define RS485_ENC0            0x54
#define RS485_ENC1            0x58

#define RS485_POS             0x00 //this is unnecessary to use but it helps visualize the process for using the other modifiers
#define RS485_TURNS           0x01
#define RS485_ZERO            0x02
#define RS485_RESET           0x03

/* The RS485 transceiver uses 2 pins to control the state of the differential RS485 lines A/B. To control
 * the transevier we will put those pins on our digital IO pins. We will define those pins here now. We will get
 * into more information about this later on, but it is important to understand that because of the high speed of the
 * AMT24 encoder, the toggling of these pins must occur very quickly. More information available in the walkthrough online.
 * Receive enable, drive enable, data in, received data out. Typically the RE and DE pins are connected together and controlled
 * with 1 IO pin but we'll connect them all up for full control.
 */
#define RS485_T_RE            8
#define RS485_T_DE            9
#define RS485_T_DI            18
#define RS485_T_RO            19

/* For ease of reading, we will create a helper function to set the mode of the transceiver. We will send that funciton
 * these arguments corresponding with the mode we want.
 */
#define RS485_T_TX            0 //transmit: receiver off, driver on
#define RS485_T_RX            1 //receiver: driver off, transmit on

/* Connect a push-button between pin 13 and GND. When this button is pressed, send the zero command to the encoder
 */
#define ZERO_BUTTON           13

void setup()
{
  //Set the modes for the RS485 transceiver
  pinMode(RS485_T_RE, OUTPUT);
  pinMode(RS485_T_DE, OUTPUT);
  pinMode(RS485_T_RO, INPUT_PULLUP); //enable pull-up on RX pin so that it is not left floating when the transceiver is in transmit-mode

  pinMode(ZERO_BUTTON, INPUT_PULLUP); //enable pull-up resistor so that we can detect when the button is pressed, which connects to GND

  //Initialize the UART serial connection to the PC
  Serial.begin(USB_BAUDRATE);

  //Initialize the UART link to the RS485 transceiver
  Serial1.begin(RS485_BAUDRATE);
}

void loop()
{
  //create an array of encoder addresses so we can use them in a loop
  //uint8_t addresses[2] = {RS485_ENC0, RS485_ENC1};
  uint8_t addresses[1] = {RS485_ENC0};

  //send the zero command to the encoders if the button is pressed
  if(digitalRead(ZERO_BUTTON) == LOW)
  {
    for(int encoder = 0; encoder < sizeof(addresses); ++encoder)
    {
      Serial.println("Zeroing...");
      sendCommandRS485(addresses[encoder] | RS485_ZERO);
      delay(100); //wait for zero
      
      Serial.println("Zero done. Resetting...");
      sendCommandRS485(addresses[encoder] | RS485_RESET);
      delay(500); //wait for reset to complete
    }

    //wait for the button to be released
    while(digitalRead(ZERO_BUTTON) == LOW)
    {
      delay(100);
    }
  }

  for(int encoder = 0; encoder < sizeof(addresses); ++encoder)
  {
    //clear whatever is in the read buffer in case there's nonsense in it
    while (Serial1.available()) Serial1.read();

    sendCommandRS485(addresses[encoder] | RS485_POS);

    //We need to give the encoder enough time to respond, but not too long. In a tightly controlled application we would want to use a timeout counter
    //to make sure we don't have any issues, but for this demonstration we will just have an arbitrary delay before checking to see if we have data to read.
    delayMicroseconds(400);
    
    //Response from encoder should be exactly 2 bytes
    int bytes_received = Serial1.available();
    if (bytes_received == 2)
    {
      uint16_t currentPosition = Serial1.read(); //low byte comes first
      currentPosition |= Serial1.read() << 8;    //high byte next, OR it into our 16 bit holder but get the high bit into the proper placeholder

      if (verifyChecksumRS485(currentPosition))
      {
        //we got back a good position, so just mask away the checkbits
        currentPosition &= 0x3FFF;

        //If the resolution is 12-bits, then shift position
        if (RESOLUTION == 12)
        {
          currentPosition = currentPosition >> 2;
        }
        Serial.print("Encoder #");
        Serial.print(encoder, DEC);
        Serial.print(" position: ");
        Serial.println(currentPosition, DEC); //print the position in decimal format
      }
      else
      {
        Serial.print("Encoder #");
        Serial.print(encoder, DEC);
        Serial.println(" error: Invalid checksum.");
      }
    }
    else
    {
      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" error: Expected to receive 2 bytes. Actually received ");
      Serial.print(bytes_received, DEC);
      Serial.println(" bytes.");
    }

    //flush the received serial buffer just in case anything extra got in there
    while (Serial1.available()) Serial1.read();
  }

  //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
  delay(500);
}

bool verifyChecksumRS485(uint16_t message)
{
  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent.
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}

void sendCommandRS485(uint8_t command)
{
  /*
  * This sets the state of the RS485 transceiver.  To be fast, we are not using the digitalWrite functions but instead will
  * access the avr io directly. I have shown the direct access method and left commented the equivalent digitalWrite method.
  */
  PORTH |= 0b01100000; //put the transciver into transmit mode
  //digitalWrite(RS485_RE, HIGH); //ph5
  //digitalWrite(RS485_DE, HIGH); //ph6
  delayMicroseconds(10); //IO operations take time, let's throw in an arbitrary 10 microsecond delay to make sure the transeiver is ready
  Serial1.write(command); //transmit the command byte

  //Here we will make sure the data has been transmitted and then toggle the pins for the transceiver
  //Here we are accessing the avr library to make sure this happens very fast. We could use Serial.flush() which waits for the output to complete
  //but it takes about 2 microseconds, which gets pretty close to our 3 microsecond window. Instead we want to wait until the serial transmit flag USCR1A completes.
  while (!(UCSR1A & _BV(TXC1))); //wait for transmit to finish

  PORTH &= 0b10011111; //set the transceiver back into receive mode for the encoder response
  //digitalWrite(RS485_RE, LOW);
  //digitalWrite(RS485_DE, LOW);
}
