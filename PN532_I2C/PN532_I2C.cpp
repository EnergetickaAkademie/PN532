/**
 * @modified picospuch
 */

#include "PN532_I2C.h"
#include "PN532_debug.h"
#include "Arduino.h"

#define PN532_I2C_ADDRESS       (0x48 >> 1)

#if defined(I2C_BUFFER_LENGTH)
#define PN532_I2C_WIRE_BUFFER_LENGTH I2C_BUFFER_LENGTH
#elif defined(BUFFER_LENGTH)
#define PN532_I2C_WIRE_BUFFER_LENGTH BUFFER_LENGTH
#else
#define PN532_I2C_WIRE_BUFFER_LENGTH 32
#endif


PN532_I2C::PN532_I2C(TwoWire &wire)
{
    _wire = &wire;
    command = 0;
}

void PN532_I2C::begin()
{
    _wire->begin();
}

void PN532_I2C::wakeup()
{
    delay(500); // wait for all ready to manipulate pn532
}

int8_t PN532_I2C::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    if (header == NULL || hlen == 0) {
        return PN532_INVALID_FRAME;
    }

    command = header[0];
    _wire->beginTransmission(PN532_I2C_ADDRESS);
    
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);
    
    uint8_t length = hlen + blen + 1;   // length of data field: TFI + DATA
    write(length);
    write(~length + 1);                 // checksum of length
    
    write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532;    // sum of TFI + DATA
    
    DMSG("write: ");
       
    for (uint8_t i = 0; i < hlen; i++) {
        if (write(header[i])) {
            sum += header[i];
            
            DMSG_HEX(header[i]);
        } else {
            DMSG("\nToo many data to send, I2C doesn't support such a big packet\n");     // I2C max packet: 32 bytes
            return PN532_INVALID_FRAME;
        }
    }

    for (uint8_t i = 0; i < blen; i++) {
        if (write(body[i])) {
            sum += body[i];
            
            DMSG_HEX(body[i]);
        } else {
            DMSG("\nToo many data to send, I2C doesn't support such a big packet\n");     // I2C max packet: 32 bytes
            return PN532_INVALID_FRAME;
        }
    }
  
    uint8_t checksum = ~sum + 1;            // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);
    
    if (_wire->endTransmission() != 0) {
        return PN532_INVALID_FRAME;
    }
    
    DMSG('\n');

    return readAckFrame();
}

int16_t PN532_I2C::getResponseLength(uint8_t buf[], uint8_t len, uint16_t timeout) {
    const uint8_t PN532_NACK[] = {0, 0, 0xFF, 0xFF, 0, 0};
    uint16_t time = 0;

    do {
        const size_t expectedLength = 6;
        const size_t receivedLength = _wire->requestFrom(
                (uint16_t) PN532_I2C_ADDRESS, (uint8_t) expectedLength);

        if (receivedLength == expectedLength) {
            int status = read();
            if (status >= 0 && (status & 1)) {  // check first byte --- status
                break;                         // PN532 is ready
            }
        }

        // A PN532 that is waking up can transiently NACK or return a short
        // status frame. Discard it and keep polling within the caller's
        // bounded timeout instead of failing the whole command immediately.
        while (_wire->available()) {
            read();
        }

        delay(1);
        time++;
        if ((0 != timeout) && (time > timeout)) {
            return PN532_TIMEOUT;
        }
    } while (1); 
    
    int preamble = read();
    int startCode1 = read();
    int startCode2 = read();
    int frameLength = read();
    int lengthChecksum = read();

    if (preamble != 0x00 ||
            startCode1 != 0x00 ||
            startCode2 != 0xFF ||
            frameLength < 0 ||
            lengthChecksum < 0 ||
            (uint8_t)(frameLength + lengthChecksum) != 0) {
        return PN532_INVALID_FRAME;
    }

    // request for last respond msg again
    _wire->beginTransmission(PN532_I2C_ADDRESS);
    for (uint16_t i = 0; i < sizeof(PN532_NACK); ++i) {
      write(PN532_NACK[i]);
    }
    if (_wire->endTransmission() != 0) {
        return PN532_INVALID_FRAME;
    }

    return frameLength;
}

int16_t PN532_I2C::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    uint16_t time = 0;
    // Keep transport errors signed. Narrowing a timeout (-1/-2) to uint8_t
    // would turn it into a 255-byte frame and overflow common Wire buffers.
    int16_t responseLength = getResponseLength(buf, len, timeout);
    if (responseLength < 0) {
        return responseLength;
    }
    if (responseLength < 2) {
        return PN532_INVALID_FRAME;
    }
    if (responseLength > (int16_t) len + 2) {
        return PN532_NO_SPACE;
    }

    const size_t requestLength = (size_t) responseLength + 8;
    if (requestLength > PN532_I2C_WIRE_BUFFER_LENGTH) {
        return PN532_NO_SPACE;
    }

    // [RDY] 00 00 FF LEN LCS (TFI PD0 ... PDn) DCS 00
    do {
        const size_t receivedLength = _wire->requestFrom(
                (uint16_t) PN532_I2C_ADDRESS, (uint8_t) requestLength);

        if (receivedLength == requestLength) {
            int status = read();
            if (status >= 0 && (status & 1)) {  // check first byte --- status
                break;                         // PN532 is ready
            }
        }

        while (_wire->available()) {
            read();
        }

        delay(1);
        time++;
        if ((0 != timeout) && (time > timeout)) {
            return PN532_TIMEOUT;
        }
    } while (1); 
    
    int preamble = read();
    int startCode1 = read();
    int startCode2 = read();
    int frameLength = read();
    int lengthChecksum = read();
    if (preamble != 0x00 ||
            startCode1 != 0x00 ||
            startCode2 != 0xFF ||
            frameLength < 2 ||
            frameLength != responseLength ||
            lengthChecksum < 0 ||
            (uint8_t)(frameLength + lengthChecksum) != 0) {
        return PN532_INVALID_FRAME;
    }

    uint8_t length = (uint8_t) frameLength;
    uint8_t cmd = command + 1;               // response command
    int direction = read();
    int responseCommand = read();
    if (direction != PN532_PN532TOHOST || responseCommand != cmd) {
        return PN532_INVALID_FRAME;
    }
    
    length -= 2;
    if (length > len) {
        return PN532_NO_SPACE;  // not enough space
    }
    
    DMSG("read:  ");
    DMSG_HEX(cmd);
    
    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length; i++) {
        int value = read();
        if (value < 0) {
            return PN532_INVALID_FRAME;
        }
        buf[i] = (uint8_t) value;
        sum += buf[i];
        
        DMSG_HEX(buf[i]);
    }
    DMSG('\n');
    
    int checksum = read();
    if (checksum < 0) {
        return PN532_INVALID_FRAME;
    }
    if (0 != (uint8_t)(sum + checksum)) {
        DMSG("checksum is not ok\n");
        return PN532_INVALID_FRAME;
    }
    if (read() != PN532_POSTAMBLE) {
        return PN532_INVALID_FRAME;
    }
    
    return length;
}

int8_t PN532_I2C::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
    uint8_t ackBuf[sizeof(PN532_ACK)];
    
    DMSG("wait for ack at : ");
    DMSG(millis());
    DMSG('\n');
    
    uint16_t time = 0;
    do {
        const size_t ackLength = sizeof(PN532_ACK) + 1;
        const size_t receivedLength = _wire->requestFrom(
                (uint16_t) PN532_I2C_ADDRESS, (uint8_t) ackLength);

        if (receivedLength == ackLength) {
            int status = read();
            if (status >= 0 && (status & 1)) {  // check first byte --- status
                break;                         // PN532 is ready
            }
        }

        while (_wire->available()) {
            read();
        }

        delay(1);
        time++;
        if (time > PN532_ACK_WAIT_TIME) {
            DMSG("Time out when waiting for ACK\n");
            return PN532_TIMEOUT;
        }
    } while (1); 
    
    DMSG("ready at : ");
    DMSG(millis());
    DMSG('\n');
    

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        int value = read();
        if (value < 0) {
            return PN532_INVALID_ACK;
        }
        ackBuf[i] = (uint8_t) value;
    }
    
    if (memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK))) {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    
    return 0;
}
