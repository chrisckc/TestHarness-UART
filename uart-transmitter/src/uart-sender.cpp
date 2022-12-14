// Based on SPI master-slave example by Michael Stoops
// https://github.com/raspberrypi/pico-examples/tree/master/spi/spi_master_slave
// Extensively modified for use with Arduino-Pico and testing the reliability of UART communications
#include <Arduino.h>

// Debug Signal outputs
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)
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

bool ledState = false;
unsigned int receiveCounter = 0, sendCounter = 0, lastSendCount = 0, sendRate = 0, errorCount = 0;
uint8_t previousTransferSize = 0;
unsigned long startMillis = 0;
unsigned long currentMillis = 0, seconds = 0;
unsigned int lastSendMillis = 0, sendInterval = 100; // send every 100 milliseconds


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
        uint8_t inverted = (uint8_t)~i;
        if (in_buf[i] != inverted) {
            Serial.printf("Error page: %07u index: %03u expected: 0x%02X received 0x%02X    \r\n", page, i, inverted, in_buf[i]);
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
    int startupDelay = 10; // wait a further 6 seconds
    for (int i = 1; i <= startupDelay; ++i) {
        Serial.printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    Serial.printf("\e[2J\e[H"); // clear screen and go to home position
    Serial.printf("UART transmitter using Baud Rate: %d \r\n", UART_BAUDRATE);
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
        // The values should be: {0x01, 0x02, 0x03...}
        out_buf[i] = i + 1;
    }

    Serial.printf("UART Sender says: The value 0x%02X (%u) followed immediately by the buffer printed below will be written to the receiver:\r\n", BUF_LEN, BUF_LEN);
    printbuf(out_buf, BUF_LEN);
    Serial.printf("\r\n");
    Serial.printf("The value 0x%02X (%u) is expected to be returned followed by a reversed version of the above buffer\r\n\r\n", BUF_LEN, BUF_LEN);

    startMillis = millis();
}

void sendTestData(uint8_t length) {
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        Serial.printf("UART Sender says: Sending Output buffer...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    }
    // Send the data length value on the UART so the receiver knows what to expect next
    UART_INSTANCE.write(length);
    // Write the output buffer to the UART
    UART_INSTANCE.write(out_buf, length);
    sendCounter++;
    if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
        Serial.printf("UART sender says: Output buffer page %u sent, buffer size: %03u \r\n", sendCounter, length);
    }
}

void loop() {
    static unsigned long lastSecondMillis = 0;
    static unsigned int loopCounter = 0, lastLoopCounter = 0;
    loopCounter++;
    currentMillis =  millis();
    seconds = (currentMillis - startMillis) / 1000;

    // Send data every sendInterval
    if (currentMillis >= lastSendMillis + sendInterval) {
        lastSendMillis = currentMillis;
        digitalWrite(LED_BUILTIN, HIGH);
        lastSecondMillis = currentMillis;
        lastLoopCounter = loopCounter;
        loopCounter = 0;
        sendRate = sendCounter - lastSendCount;
        lastSendCount = sendCounter;
        digitalWrite(LED_BUILTIN, HIGH);
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                Serial.printf("\e[H\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); // move to the home position, at the upper left of the screen
            }
            Serial.printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
            Serial.printf("LoopRate: %07u         \r\n", lastLoopCounter); // how many loops per second
            Serial.printf("sendRate: %07u         \r\n", sendRate);
            Serial.printf("sendCounter: %07u         \r\n", sendCounter);
            Serial.printf("receiveCounter: %07u         \r\n", receiveCounter);
            Serial.printf("previousTransferSize: %03u         \r\n", previousTransferSize);
            Serial.printf("Receive errorCount: %03u         \r\n", errorCount); // how many loops per second
        }

        digitalWrite(DEBUG_PIN3, LOW);
        //Send data...
        sendTestData(BUF_LEN);
        digitalWrite(DEBUG_PIN3, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
    }

    int bytesAvailable = UART_INSTANCE.available();
    if (bytesAvailable > 0) {
        receiveCounter++;
        digitalWrite(DEBUG_PIN2, LOW);
        // Read the expected byte count from the sender
        previousTransferSize = UART_INSTANCE.read();
        //  Read from the UART
        UART_INSTANCE.readBytes(in_buf, previousTransferSize); // If the expected bytes are not all received the function will timeout subject to setTimeout(), default 1000mS
        digitalWrite(DEBUG_PIN2, HIGH);
        delayMicroseconds(10);
        digitalWrite(DEBUG_PIN2, LOW);
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            Serial.printf("Data Received: UART_INSTANCE.available() bytes was: %d                            \r\n", bytesAvailable);
            Serial.printf("UART sender says: read page %u back from the receiver, previousTransferSize: %03u \r\n", sendCounter, previousTransferSize);
            // Write the buffer out to the USB serial port
            printbuf(in_buf, BUF_LEN);
            Serial.printf("UART Sender says: Verifying received data...                                         \r\n");
            if (previousTransferSize != BUF_LEN) {
                Serial.printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u\r\n", receiveCounter, previousTransferSize, BUF_LEN);
            }
            verifyInBuffer(sendCounter);
        }
        clearbuf(in_buf, BUF_LEN);
        digitalWrite(DEBUG_PIN2, HIGH);
    }
}
