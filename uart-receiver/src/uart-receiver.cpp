// Based on SPI master-slave example by Michael Stoops
// https://github.com/raspberrypi/pico-examples/tree/master/spi/spi_master_slave
// Extensively modified for use with Arduino-Pico and testing the reliability of UART communications
#include <Arduino.h>

// Debug Signal outputs, to allow us to observe any error conditions on the scope
#define DEBUG_PIN2 (6u)
#define DEBUG_PIN3 (7u)
#define DEBUG_PIN4 (8u)
#define DEBUG_PIN5 (9u)
#define DEBUG_PIN_INITIAL_STATE (HIGH)

// Serial data output and debugging options
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages

#define UART_INSTANCE Serial1 // Serial1 is UART 0 on the Pico, Serial2 is UART1
#define UART_FIFO_SIZE (256)
#define UART_BAUDRATE (921600)

#ifdef ARDUINO_ARCH_RP2040
    #define UART_TX  (0u) // spi1 default:PIN_SPI1_MOSI (15u)  SPI TX pin, MOSI is TX from the SPI Master device perspective
    #define UART_RX  (1u) // spi1 default:PIN_SPI1_MISO (12u)  SPI RX pin, MISO is RX from the SPI Master device perspective
#else
    #define UART_TX  (0u) // spi1 default:PIN_SPI1_MOSI (15u)  SPI TX pin, MOSI is TX from the SPI Master device perspective
    #define UART_RX  (1u) // spi1 default:PIN_SPI1_MISO (12u)  SPI RX pin, MISO is RX from the SPI Master device perspective
#endif

#define BUF_LEN         0xFF // 255 byte buffer
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];
unsigned int seconds = 0, lastSeconds = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, errorCount = 0;
uint8_t expectedByteCount = 0, lastReceivedByteCount = 0;
unsigned long startMillis = 0;
unsigned long currentMillis = 0;

void printbuf(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            Serial.printf("%02X \r\n", buf[i]);
        else
            Serial.printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        Serial.println("   ");
    }
}

bool verifyInBuffer(unsigned int page) {
    bool success = true;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        if (in_buf[i] != i + 1) {
            Serial.printf("Error page: %07u index: %03u expected: 0x%02x received 0x%02x\r\n", page, i, i + 1, in_buf[i]);
            errorCount++;
            success = false;
        }
    }
    return success;
}

void clearbuf(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}


void setup() {
    // Setup Serial Comms
    Serial.begin(921600); // Baud rate is ignored because pico has built in USB-UART
    // Wait for Serial Comms or Timeout after 5 seconds
    //while (!Serial && millis() < 5000);
    int startupDelay = 9; // wait a further 5 seconds
    for (int i = 1; i <= startupDelay; ++i) {
        Serial.printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    Serial.printf("\e[2J\e[H"); // clear screen and go to home position
    Serial.printf("UART receiver using Baud Rate: %d \r\n", UART_BAUDRATE);
    #ifdef ARDUINO_ARCH_RP2040
        Serial.printf("rp2040_chip_version: %d \r\n", rp2040_chip_version());
        Serial.printf("rp2040_rom_version: %d \r\n", rp2040_rom_version());
        Serial.printf("get_core_num: %u \r\n\r\n", get_core_num());
        //pinMode(23, OUTPUT);
        //digitalWrite(23, HIGH); // Set the SMPS Power Save pin high, forcing the regulator into Pulse Width Modulation (PWM) mode, less output ripple
    #endif

    pinMode(LED_BUILTIN, OUTPUT);
    // Setup Debug output pins
    pinMode(DEBUG_PIN2, OUTPUT);
    digitalWrite(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN3, OUTPUT);
    digitalWrite(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN4, OUTPUT);
    digitalWrite(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);
    pinMode(DEBUG_PIN5, OUTPUT);
    digitalWrite(DEBUG_PIN5, DEBUG_PIN_INITIAL_STATE);

    // Configure the UART
    UART_INSTANCE.setRX(UART_RX);
    UART_INSTANCE.setTX(UART_TX);

    UART_INSTANCE.setFIFOSize(UART_FIFO_SIZE);
    UART_INSTANCE.begin(UART_BAUDRATE);
    // Workaround for the issue where available() is reporting bytes available at the start when there are none.
    //while(UART_INSTANCE.available()) UART_INSTANCE.read();

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        out_buf[i] = ~i;
    }

    Serial.printf("UART receiver says: When reading from the sender, the following %u byte Output buffer will be sent back to the sender:\r\n", BUF_LEN);
    printbuf(out_buf, BUF_LEN);

    startMillis = millis();
}

void serialEvent() {
  //statements
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;

    // Read the expected byte count a simple way of verifying the last data transfer
    int bytesAvailable = UART_INSTANCE.available();
    if (bytesAvailable > 0) {
        digitalWrite(DEBUG_PIN2, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
        // Read the expected byte count from the sender
        expectedByteCount = UART_INSTANCE.read();
        //  Read from the UART
        UART_INSTANCE.readBytes(in_buf, expectedByteCount); // If the expected bytes are not all received the function will timeout subject to setTimeout(), default 1000mS
        expectedByteCount;
        digitalWrite(DEBUG_PIN2, HIGH);

        digitalWrite(DEBUG_PIN3, LOW);
        // Send data back to the sender...
        // Send the data length value on the UART so the receiver knows what to expect next
        UART_INSTANCE.write(BUF_LEN);
        // Write the output buffer to the UART
        UART_INSTANCE.write(out_buf, BUF_LEN);
        digitalWrite(DEBUG_PIN3, HIGH);

        // Keep track of seconds since start
        currentMillis =  millis();
        seconds = (currentMillis - startMillis) / 1000;
        if (seconds - lastSeconds > 0) {
            lastSeconds = seconds;
            receiveRate = receiveCounter - lastReceiveCount;
            lastReceiveCount = receiveCounter;
        }
        delayMicroseconds(10);
        digitalWrite(DEBUG_PIN3, LOW);
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                Serial.printf("\e[H\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); // move to the home position, at the upper left of the screen and then down
            }
            Serial.printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
            Serial.printf("receiveCounter: %07u         \r\n", receiveCounter);
            Serial.printf("receiveRate: %07u         \r\n", receiveRate);
            Serial.printf("Receive errorCount: %03u         \r\n", errorCount);
            Serial.printf("Data Received: UART_INSTANCE.available() bytes was: %d                                         \r\n", bytesAvailable);
            Serial.printf("UART receiver says: read page %u from the sender, byteCount: %u lastReceivedByteCount: %u \r\n", receiveCounter, expectedByteCount, lastReceivedByteCount);
            // Write the buffer out to the USB serial port
            printbuf(in_buf, BUF_LEN);
            Serial.printf("UART receiver says: Responded with Output buffer page %u, buffer size: %03u \r\n", receiveCounter, BUF_LEN);
            Serial.printf("UART Receiver says: Verifying received data... \r\n");
            if (expectedByteCount != BUF_LEN) {
                printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u\r\n", receiveCounter, expectedByteCount, BUF_LEN);
            }
            verifyInBuffer(receiveCounter);
        }
        clearbuf(in_buf, BUF_LEN);
        lastReceivedByteCount = expectedByteCount;
        receiveCounter++;
        digitalWrite(DEBUG_PIN3, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
    }


}
