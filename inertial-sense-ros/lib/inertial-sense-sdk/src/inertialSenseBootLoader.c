/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <math.h>
#include "inertialSenseBootLoader.h"
#include "ISConstants.h"
#include "ISUtilities.h"

#define ENABLE_HEX_BOOT_LOADER 1
#define MAX_SEND_COUNT 510
#define MAX_VERIFY_CHUNK_SIZE 1024
#define BOOTLOADER_TIMEOUT_DEFAULT 1000
#define SAM_BA_BAUDRATE 115200
#define SAM_BA_FLASH_PAGE_SIZE 512
#define SAM_BA_FLASH_START_ADDRESS 0x00400000
#define SAM_BA_BOOTLOADER_SIZE 16384

#define X_SOH 0x01
#define X_EOT 0x04
#define X_ACK 0x06
#define X_NAK 0x15
#define X_CAN 0x18
#define CRC_POLY 0x1021
#define XMODEM_PAYLOAD_SIZE 128

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE 65536

static int bootloadUpdateBootloaderSendFile(bootload_params_t* p);

typedef struct
{
    int version;
    int firstPageSkipBytes;
    int verifyChunkSize;
    bootload_params_t* param;
} bootloader_state_t;

PUSH_PACK_1

typedef struct
{
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[XMODEM_PAYLOAD_SIZE];
    uint16_t crc;
} xmodem_chunk_t;

POP_PACK

#if PLATFORM_IS_WINDOWS

#define bootloader_snprintf _snprintf

#else

#define bootloader_snprintf snprintf

#endif

#define bootloader_perror(s, ...) \
    if ((s)->error != 0 && BOOTLOADER_ERROR_LENGTH > 0) \
    { \
        bootloader_snprintf((s)->error + strlen((s)->error), BOOTLOADER_ERROR_LENGTH - strlen((s)->error), __VA_ARGS__); \
    }

#define bootloader_min(a, b) (a < b ? a : b)
#define bootloader_max(a, b) (a > b ? a : b)

static const unsigned char hexLookupTable[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

// Fastest to slowest
static const int s_baudRateList[] = { IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_LEGACY, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

static uint16_t crc_update(uint16_t crc_in, int incr)
{
    uint16_t crc = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr)
    {
        out++;
    }

    if (crc)
    {
        out ^= CRC_POLY;
    }

    return out;
}


static uint16_t crc16(uint8_t* data, uint16_t size)
{
    uint16_t crc, i;

    for (crc = 0; size > 0; size--, data++)
    {
        for (i = 0x80; i; i >>= 1)
        {
            crc = crc_update(crc, *data & i);
        }
    }

    for (i = 0; i < 16; i++)
    {
        crc = crc_update(crc, 0);
    }

    return crc;
}

static int xModemSend(serial_port_t* s, uint8_t* buf, size_t len)
{
    int ret;
    uint8_t eot = X_EOT;
    uint8_t answer;
    xmodem_chunk_t chunk;

    chunk.block = 1;
    chunk.start = X_SOH;

    // wait for receiver ping
    do
    {
        if (serialPortRead(s, &answer, sizeof(uint8_t)) != sizeof(uint8_t))
        {
            return -1;
        }
    } while (answer != 'C');

    // write up to one sector
    while (len)
    {
        size_t z = 0;
        int next = 0;
//         char status;

        z = _MIN(len, sizeof(chunk.payload));
        memcpy(chunk.payload, buf, z);
        memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);

        chunk.crc = SWAP16(crc16(chunk.payload, sizeof(chunk.payload)));
        chunk.block_neg = 0xff - chunk.block;

        ret = serialPortWrite(s, (const uint8_t*)&chunk, sizeof(xmodem_chunk_t));
        if (ret != sizeof(xmodem_chunk_t))
        {
            return -1;
        }

        ret = serialPortReadTimeout(s, &answer, sizeof(uint8_t), BOOTLOADER_TIMEOUT_DEFAULT);
        if (ret != sizeof(uint8_t))
        {
            return -1;
        }

        switch (answer)
        {
        case X_NAK:
//             status = 'N';
            break;
        case X_ACK:
//             status = '.';
            next = 1;
            break;
        case X_CAN:
            return -1;
        default:
//             status = '?';
            break;
        }

        if (next)
        {
            chunk.block++;
            len -= z;
            buf += z;
        }
    }

    ret = serialPortWrite(s, &eot, sizeof(uint8_t));
    if (ret != sizeof(uint8_t))
    {
        return -1;
    }
    serialPortReadChar(s, &eot);
    return 0;
}

int bootloaderCycleBaudRate(int baudRate)
{
	for (unsigned int i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList); i++)
	{	// Find current baudrate
		if (baudRate == s_baudRateList[i])
		{
			// Get next baudrate
			if (i + 1 < _ARRAY_ELEMENT_COUNT(s_baudRateList))
			{
				return s_baudRateList[i + 1];
			}
		}
	}

	return s_baudRateList[0];
}

// Finds the closest bootloader supported baudrate 
int bootloaderClosestBaudRate(int baudRate)
{
	for (unsigned int i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList); i++)
	{
		if (baudRate >= s_baudRateList[i])
		{
			return s_baudRateList[i];
		}
	}

	return IS_BAUD_RATE_BOOTLOADER_SLOW;
}

// negotiate the bootloader version, once a 'U' character has been read, we read another character, timing out after 500 milliseconds
// if nothing comes back, we are using version 1, otherwise the version is the number sent back
static int bootloaderNegotiateVersion(bootloader_state_t* state)
{
    unsigned char v = 0;

    do {
        if (serialPortReadCharTimeout(state->param->port, &v, 500) == 0)
        {
            v = '1';
        }
    } while (v == 'U');

    if (v == '1')
    {
        // version 1
        state->version = 1;
        state->firstPageSkipBytes = 8192;
    }
    else if (v >= '2' && v <= '4')
    {
        // version 2, 3 (which sent v2), 4
        state->version = v-'0';
        state->firstPageSkipBytes = 16384;
    }
    else
    {
        bootloader_perror(state->param->port, "Invalid version sent from bootloader: 0x%02X\n", (int)v);
        return 0;
    }

#if PLATFORM_IS_WINDOWS

    // EvalTool and multiple bootloads under Windows 10 have issues with dropped data if verify runs too fast
    state->verifyChunkSize = 125;

#else

    state->verifyChunkSize = MAX_VERIFY_CHUNK_SIZE;

#endif

    return 1;
}

static int serialPortOpenInternal(serial_port_t* s, int baudRate, char* error, int errorLength)
{
    if (error != 0 && errorLength > 0)
    {
        *error = '\0';
    }
    serialPortClose(s);
    s->error = error;
    s->errorLength = errorLength;
    if (serialPortOpenRetry(s, s->port, baudRate == 0 ? IS_BAUD_RATE_BOOTLOADER : baudRate, 1) == 0)
    {
        bootloader_perror(s, "Unable to open serial port at %s\n", s->port);
        return 0;
    }

    return 1;
}

// calculate checksum - start is INCLUSIVE and end is EXCLUSIVE, if checkSumPosition is not 0, the checkSum is written to ptr + checkSumPosition
static int bootloaderChecksum(int checkSum, unsigned char* ptr, int start, int end, int checkSumPosition, int finalCheckSum)
{
    unsigned char c1, c2;
    unsigned char* currentPtr = (unsigned char*)(ptr + start);
    unsigned char* endPtr = (unsigned char*)(ptr + end - 1);
    unsigned char b;

    while (currentPtr < endPtr)
    {
        c1 = *(currentPtr++) | 0x20;
        c1 = (c1 <= '9' ? c1 + 0xD0 : c1 + 0xA9);
        c2 = *(currentPtr++) | 0x20;
        c2 = (c2 <= '9' ? c2 + 0xD0 : c2 + 0xA9);
        b = (c1 << 4) | c2;
        checkSum += b;
    }

    if (finalCheckSum)
    {
        checkSum = (unsigned char)(~checkSum + 1);
    }
    if (checkSumPosition != 0)
    {
        bootloader_snprintf((char*)(ptr + checkSumPosition), 3, "%.2X", checkSum);
    }

    return checkSum;
}

static int bootloaderEraseFlash(serial_port_t* s)
{
    // give the device this many seconds to erase flash before giving up
    static const int eraseFlashTimeoutMilliseconds = 60000;

    unsigned char selectFlash[24];

    memcpy(selectFlash, ":03000006030000F4CC\0\0\0\0\0", 24);
    bootloaderChecksum(0, selectFlash, 1, 17, 17, 1);
    if (serialPortWriteAndWaitForTimeout(s, selectFlash, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        bootloader_perror(s, "Failed to select flash memory to erase\n");
        return 0;
    }

    memcpy(selectFlash, ":0200000400FFFBCC\0", 18);
    bootloaderChecksum(0, selectFlash, 1, 15, 15, 1);
    if (serialPortWriteAndWaitForTimeout(s, selectFlash, 17, (unsigned char*)".\r\n", 3, eraseFlashTimeoutMilliseconds) == 0)
    {
        bootloader_perror(s, "Failed to perform erase flash memory operation\n");
        return 0;
    }

    return 1;
}

static int bootloaderSelectPage(serial_port_t* s, int page)
{
    // Atmel select page command (0x06) is 4 bytes and the data is always 0301xxxx where xxxx is a 16 bit page number in hex
    unsigned char changePage[24];
    bootloader_snprintf((char*)changePage, 24, ":040000060301%.4XCC", page);
    bootloaderChecksum(0, changePage, 1, 17, 17, 1);

    if (serialPortWriteAndWaitForTimeout(s, changePage, 19, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        bootloader_perror(s, "Failed to change to page %d\n", page);
        return 0;
    }

    return 1;
}

static int bootloaderBeginProgramForCurrentPage(serial_port_t* s, int startOffset, int endOffset)
{
    // Atmel begin program command is 0x01, different from standard intel hex where command 0x01 is end of file
    // After the 0x01 is a 00 which means begin writing program
    // The begin program command uses the current page and specifies two 16 bit addresses that specify where in the current page
    //  the program code will be written
    unsigned char programPage[24];
    bootloader_snprintf((char*)programPage, 24, ":0500000100%.4X%.4XCC", startOffset, endOffset);
    bootloaderChecksum(0, programPage, 1, 19, 19, 1);

    if (serialPortWriteAndWaitForTimeout(s, programPage, 21, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        bootloader_perror(s, "Failed to select offset %X to %X\n", startOffset, endOffset);
        return 0;
    }

    return 1;
}

static int bootloaderReadLine(FILE* file, char line[1024])
{
    char c;
    char* currentPtr = line;
    char* endPtr = currentPtr + 1023;

    while (currentPtr != endPtr)
    {
        // read one char
        c = (char)fgetc(file);
        if (c == '\r')
        {
            // eat '\r' chars
            continue;
        }
        else if (c == '\n' || c == (char)EOF)
        {
            // newline char, we have a line
            break;
        }
        *currentPtr++ = c;
    }

    *currentPtr = '\0';

    // TODO: Figure out why ARM64 bootloader hits this...
    if (currentPtr - line == 1023)
    {
        return 0;
    }
    return (int)(currentPtr - line);
}

static int bootloaderUploadHexDataPage(serial_port_t* s, unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    int i;

    if (byteCount == 0)
    {
        return 1;
    }

    // create a program request with just the hex characters that will fit on this page
    unsigned char programLine[12];
    bootloader_snprintf((char*)programLine, 12, ":%.2X%.4X00", byteCount, *currentOffset);
    if (serialPortWrite(s, programLine, 9) != 9)
    {
        bootloader_perror(s, "Failed to write start page at offset %d\n", *currentOffset);
        return 0;
    }

    // add the previously written chars to the checksum
    int checkSum = bootloaderChecksum(0, programLine, 1, 9, 0, 0);

    // write all of the hex chars
    int charsForThisPage = byteCount * 2;
    if (serialPortWrite(s, hexData, charsForThisPage) != charsForThisPage)
    {
        bootloader_perror(s, "Failed to write page data at offset %d\n", *currentOffset);
        return 0;
    }

    // calculate verification checksum for this data
    for (i = 0; i < charsForThisPage; i++)
    {
        *verifyCheckSum = ((*verifyCheckSum << 5) + *verifyCheckSum) + hexData[i];
    }

    checkSum = bootloaderChecksum(checkSum, hexData, 0, charsForThisPage, 0, 1);
    unsigned char checkSumHex[3];
    bootloader_snprintf((char*)checkSumHex, 3, "%.2X", checkSum);
    if (serialPortWriteAndWaitForTimeout(s, checkSumHex, 2, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT) == 0)
    {
        bootloader_perror(s, "Failed to write checksum %s at offset %d\n", checkSumHex, *currentOffset);
        return 0;
    }

    *totalBytes += byteCount;
    *currentOffset += byteCount;

    return 1;
}

static int bootloaderUploadHexData(serial_port_t* s, unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum)
{
    if (charCount > MAX_SEND_COUNT)
    {
        bootloader_perror(s, "Unexpected char count of %d for page %d at offset %X\n", charCount, *currentPage, *currentOffset);
        return 0;
    }
    else if (charCount == 0)
    {
        return 1;
    }

    int byteCount = charCount / 2;

    // check if we will overrun the current page
    if (*currentOffset + byteCount > FLASH_PAGE_SIZE)
    {
        int pageByteCount = FLASH_PAGE_SIZE - *currentOffset;
        if (bootloaderUploadHexDataPage(s, hexData, pageByteCount, currentOffset, totalBytes, verifyCheckSum) == 0)
        {
            bootloader_perror(s, "Failed to upload %d bytes to page %d at offset %d\n", pageByteCount, *currentPage, *currentOffset);
            return 0;
        }
        hexData += (pageByteCount * 2);
        charCount -= (pageByteCount * 2);

        // change to the next page
        *currentOffset = 0;
        (*currentPage)++;
        if (!bootloaderSelectPage(s, *currentPage) || !bootloaderBeginProgramForCurrentPage(s, 0, FLASH_PAGE_SIZE - 1))
        {
            bootloader_perror(s, "Failure issuing select page command for upload\n");
            return 0;
        }
    }

    if (charCount != 0 && bootloaderUploadHexDataPage(s, hexData, charCount / 2, currentOffset, totalBytes, verifyCheckSum) == 0)
    {
        bootloader_perror(s, "Failed to upload %d bytes to page %d at offset %d\n", charCount / 2, *currentPage, *currentOffset);
        return 0;
    }

    return 1;
}

static int bootloaderFillCurrentPage(serial_port_t* s, int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum)
{
    if (*currentOffset < FLASH_PAGE_SIZE)
    {
        unsigned char hexData[256];
        memset(hexData, 'F', 256);

        while (*currentOffset < FLASH_PAGE_SIZE)
        {
            int byteCount = (FLASH_PAGE_SIZE - *currentOffset) * 2;
            if (byteCount > 256)
            {
                byteCount = 256;
            }
            memset(hexData, 'F', byteCount);

            if (bootloaderUploadHexDataPage(s, hexData, byteCount / 2, currentOffset, totalBytes, verifyCheckSum) == 0)
            {
                bootloader_perror(s, "Failed to fill page %d with %d bytes at offset %d\n", *currentPage, byteCount, *currentOffset);
                return 0;
            }
        }
    }

    return 1;
}

static int bootloaderDownloadData(serial_port_t* s, int startOffset, int endOffset)
{
    // Atmel download data command is 0x03, different from standard intel hex where command 0x03 is start segment address
    unsigned char programLine[24];
    int n;
    n = bootloader_snprintf((char*)programLine, 24, ":0500000300%.4X%.4XCC", startOffset, endOffset);
    programLine[n] = 0;
    bootloaderChecksum(0, programLine, 1, 19, 19, 1);
    if (serialPortWrite(s, programLine, 21) != 21)
    {
        bootloader_perror(s, "Failed to attempt download offsets %X to %X\n", startOffset, endOffset);
        return 0;
    }

    return 1;
}


static int bootloaderVerify(int lastPage, int checkSum, bootloader_state_t* state)
{
    int verifyChunkSize = state->verifyChunkSize;
    int chunkSize = bootloader_min(FLASH_PAGE_SIZE, verifyChunkSize);
    int realCheckSum = 5381;
    int totalCharCount = state->firstPageSkipBytes * 2;
    int grandTotalCharCount = (lastPage + 1) * FLASH_PAGE_SIZE * 2; // char count
    int i, pageOffset, readCount, actualPageOffset, pageChars, chunkIndex, lines;
    int verifyByte = -1;
    unsigned char chunkBuffer[(MAX_VERIFY_CHUNK_SIZE * 2) + 64]; // extra space for overhead
    float percent = 0.0f;
    unsigned char c=0;
    FILE* verifyFile = 0;

    if (state->param->verifyFileName != 0)
    {

#ifdef _MSC_VER

        fopen_s(&verifyFile, state->param->verifyFileName, "wb");

#else

        verifyFile = fopen(state->param->verifyFileName, "wb");

#endif

    }

    for (i = 0; i <= lastPage; i++)
    {
        if (bootloaderSelectPage(state->param->port, i) == 0)
        {
            bootloader_perror(state->param->port, "Failure issuing select page command for verify\n");
            return 0;
        }
        pageOffset = (i == 0 ? state->firstPageSkipBytes : 0);
        while (pageOffset < FLASH_PAGE_SIZE)
        {
            readCount = bootloader_min(chunkSize, FLASH_PAGE_SIZE - pageOffset);

            // range is inclusive on the uINS, so subtract one
            if (bootloaderDownloadData(state->param->port, pageOffset, pageOffset + readCount - 1) == 0)
            {
                bootloader_perror(state->param->port, "Failure issuing download data command\n");
                return 0;
            }

            // each line has 7 overhead bytes, plus two bytes (hex) for each byte on the page and max 255 bytes per line
            lines = (int)ceilf((float)readCount / 255.0f);
            readCount = serialPortReadTimeout(state->param->port, chunkBuffer, (7 * lines) + (readCount * 2), BOOTLOADER_TIMEOUT_DEFAULT);
            chunkIndex = 0;

            while (chunkIndex < readCount)
            {
                if (chunkIndex > readCount - 5)
                {
                    bootloader_perror(state->param->port, "Unexpected start line during validation\n");
                    return 0;
                }

                // skip the first 5 chars, they are simply ####=
                if (chunkBuffer[chunkIndex] == 'X')
                {
                    bootloader_perror(state->param->port, "Invalid checksum during validation\n");
                    return 0;
                }
                else if (chunkBuffer[chunkIndex += 4] != '=')
                {
                    bootloader_perror(state->param->port, "Unexpected start line during validation\n");
                    return 0;
                }
                chunkBuffer[chunkIndex] = '\0';
                actualPageOffset = strtol((char*)(chunkBuffer + chunkIndex - 4), 0, 16);
                if (actualPageOffset != pageOffset)
                {
                    bootloader_perror(state->param->port, "Unexpected offset during validation\n");
                    return 0;
                }
                pageChars = 0;
                chunkIndex++;
                while (chunkIndex < readCount)
                {
                    c = chunkBuffer[chunkIndex++];
                    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))
                    {
                        pageChars++;
                        totalCharCount++;
                        realCheckSum = ((realCheckSum << 5) + realCheckSum) + c;

                        if (verifyFile != 0)
                        {
                            if (verifyByte == -1)
                            {
                                verifyByte = ((c >= '0' && c <= '9') ? c - '0' : c - 'A' + 10) << 4;
                            }
                            else
                            {
                                verifyByte |= (c >= '0' && c <= '9') ? c - '0' : c - 'A' + 10;
                                fputc(verifyByte, verifyFile);
                                verifyByte = -1;
                            }
                        }
                    }
                    else if (c == '\r')
                    {
                        continue;
                    }
                    else if (c == '\n')
                    {
                        break;
                    }
                    else
                    {
                        bootloader_perror(state->param->port, "Unexpected hex data during validation: 0x%02x [%c]\n", c, c);
                        return 0;
                    }
                }

                if (c != '\n')
                {
                    bootloader_perror(state->param->port, "Unexpected end line character found during validation: 0x%02x [%c]\n", c, c);
                    return 0;
                }

                // increment page offset
                pageOffset += (pageChars / 2);

                if (state->param->verifyProgress != 0)
                {
                    percent = (float)totalCharCount / (float)grandTotalCharCount;
                    if (state->param->verifyProgress(state->param->obj, percent) == 0)
                    {
                        bootloader_perror(state->param->port, "Validate firmware cancelled\n");
                        return 0;
                    }
                }
            }
        }
    }

    if (verifyFile != 0)
    {
        fclose(verifyFile);
    }

    if (realCheckSum != checkSum)
    {
        bootloader_perror(state->param->port, "Download checksum 0x%08x != calculated checksum 0x%08x\n", realCheckSum, checkSum);
        return 0;
    }

    return 1;
}


static int bootloaderProcessHexFile(FILE* file, bootloader_state_t* state)
{

#define BUFFER_SIZE 1024

    int currentPage = 0;
    int currentOffset = state->firstPageSkipBytes;
    int lastSubOffset = currentOffset;
    int subOffset;
    int totalBytes = state->firstPageSkipBytes;

    int verifyCheckSum = 5381;
    int lineLength;
    float percent = 0.0f;
    char line[BUFFER_SIZE];
    unsigned char output[BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed
    unsigned char* outputPtr = output;
    const unsigned char* outputPtrEnd = output + (BUFFER_SIZE * 2);
    int outputSize;
    int page;
    int pad;
    int fileSize;
    unsigned char tmp[5];
    int i;

    fseek(file, 0, SEEK_END);
    fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    while ((lineLength = bootloaderReadLine(file, line)) != 0)
    {
        if (lineLength > 12 && line[7] == '0' && line[8] == '0')
        {
            if (lineLength > BUFFER_SIZE * 4)
            {
                bootloader_perror(state->param->port, "Line length of %d was too large\n", lineLength);
                return 0;
            }

            // we need to know the offset that this line was supposed to be stored at so we can check if offsets are skipped
            memcpy(tmp, line + 3, 4);
            tmp[4] = '\0';
            subOffset = strtol((char*)tmp, 0, 16);

            // check if we skipped an offset, the intel hex file format can do this, in which case we need to make sure
            // that the bytes that were skipped get set to something
            if (subOffset > lastSubOffset)
            {
                // pad with FF bytes, this is an internal implementation detail to how the device stores unused memory
                pad = (subOffset - lastSubOffset);
                if (outputPtr + pad >= outputPtrEnd)
                {
                    bootloader_perror(state->param->port, "FF padding overflowed output buffer\n");
                    return 0;
                }

                while (pad-- != 0)
                {
                    *outputPtr++ = 'F';
                    *outputPtr++ = 'F';
                }
            }

            // skip the first 9 chars which are not data, then take everything else minus the last two chars which are a checksum
            // check for overflow
            pad = lineLength - 11;
            if (outputPtr + pad >= outputPtrEnd)
            {
                bootloader_perror(state->param->port, "Line data overflowed output buffer\n");
                return 0;
            }

            for (i = 9; i < lineLength - 2; i++)
            {
                *outputPtr++ = line[i];
            }

            // set the end offset so we can check later for skipped offsets
            lastSubOffset = subOffset + ((lineLength - 11) / 2);
            outputSize = (int)(outputPtr - output);

            // we try to send the most allowed by this hex file format
            if (outputSize < MAX_SEND_COUNT)
            {
                // keep buffering
                continue;
            }

            // upload this chunk
            else if (!bootloaderUploadHexData(state->param->port, output, (outputSize > MAX_SEND_COUNT ? MAX_SEND_COUNT : outputSize), &currentOffset, &currentPage, &totalBytes, &verifyCheckSum))
            {
                return 0;
            }

            outputSize -= MAX_SEND_COUNT;

            if (outputSize < 0 || outputSize > BUFFER_SIZE)
            {
                bootloader_perror(state->param->port, "Output size of %d was too large\n", outputSize);
                return 0;
            }
            else if (outputSize > 0)
            {
                // move the left-over data to the beginning
                memmove(output, output + MAX_SEND_COUNT, outputSize);
            }

            // reset output ptr back to the next chunk of data
            outputPtr = output + outputSize;
        }
        else if (strncmp(line, ":020000048", 10) == 0 && strlen(line) >= 13)
        {
            memcpy(tmp, line + 10, 3);
            tmp[3] = '\0';
            page = strtol((char*)tmp, 0, 16);

            if (page != 0)
            {
                // we found a change page command beyond the first, let's finish up this page and fill it to the max
                lastSubOffset = 0;
                outputSize = (int)(outputPtr - output);

                if (outputSize < 0 || outputSize > BUFFER_SIZE)
                {
                    bootloader_perror(state->param->port, "Output size of %d was too large\n", outputSize);
                    return 0;
                }

                // flush the remainder of data to the page
                else if (bootloaderUploadHexData(state->param->port, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) == 0)
                {
                    return 0;
                }

                // fill the remainder of the current page, the next time that bytes try to be written the page will be automatically incremented
                else if (bootloaderFillCurrentPage(state->param->port, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) == 0)
                {
                    return 0;
                }

                // set the output ptr back to the beginning, no more data is in the queue
                outputPtr = output;
            }
        }

        if (state->param->uploadProgress != 0)
        {
            percent = (float)ftell(file) / (float)fileSize;	// Dummy line to call ftell() once
            percent = (float)ftell(file) / (float)fileSize;
            if (state->param->uploadProgress(state->param->obj, percent) == 0)
            {
                bootloader_perror(state->param->port, "Upload firmware cancelled\n");
                return 0;
            }
        }
    }

    // upload any left over data
    outputSize = (int)(outputPtr - output);
    if (bootloaderUploadHexData(state->param->port, output, outputSize, &currentOffset, &currentPage, &totalBytes, &verifyCheckSum) == 0)
    {
        return 0;
    }

    // pad the remainder of the page with fill bytes
    if (currentOffset != 0 && bootloaderFillCurrentPage(state->param->port, &currentPage, &currentOffset, &totalBytes, &verifyCheckSum) == 0)
    {
        return 0;
    }

    if (state->param->uploadProgress != 0 && percent != 1.0f)
    {
        state->param->uploadProgress(state->param->obj, 1.0f);
    }

    if (state->param->flags.bitFields.enableVerify && state->param->verifyProgress != 0)
    {
        if (state->param->statusText)
            state->param->statusText(state->param->obj, "Verifying flash...");

        if (state->param->verifyProgress != 0 && bootloaderVerify(currentPage, verifyCheckSum, state) == 0)
        {
            return 0;
        }
    }

    // send the "reboot to program mode" command and the device should start in program mode
    if(state->param->statusText)
        state->param->statusText(state->param->obj, "Rebooting unit...");
    serialPortWrite(state->param->port, (unsigned char*)":020000040300F7", 15);
    serialPortSleep(state->param->port, 250);

    return 1;
}

static int bootloaderProcessBinFile(FILE* file, bootload_params_t* p)
{
    bootloader_state_t state;
    memset(&state, 0, sizeof(state));
    unsigned char c;
    int dataLength;
    int verifyCheckSum;
    int lastPage;
    int commandCount;
    int commandLength;
    unsigned char buf[16];
    unsigned char commandType;

    float percent = 0.0f;
    fseek(file, 0, SEEK_END);
    int fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    // verify checksum is first four bytes
    verifyCheckSum = fgetc(file) | (fgetc(file) << 8) | (fgetc(file) << 16) | (fgetc(file) << 24);

    // last page is next two bytes
    lastPage = fgetc(file) | (fgetc(file) << 8);

    // the file is formatted like this:
    // 4 bytes  : verify checksum
    // 2 bytes  : last page
    // LOOP:
    // 1 byte - command type (0 = data, 1 = raw)
    // 2 bytes - number of commands
    // n commands...
    // Data command (0)
    //	2 bytes : Offset
    //  1 byte  : Data Length
    //  n bytes : Data
    //  1 byte  : Checksum
    // Raw command (1)
    //  1 byte  : Command length
    //  n bytes : Command characters
    //  1 byte  : wait for character, '\0' for none
    //  1 byte  : sleep time after command is sent, in 250 millisecond intervals, 0 for no sleep
    //  1 byte  : wait for character timeout time in 250 millisecond intervals, 0 for no wait
    // if not EOF, go to LOOP

    while (ftell(file) != fileSize)
    {
        commandType = (unsigned char)fgetc(file);
        commandCount = fgetc(file) | (fgetc(file) << 8);

        if (commandType == 0)
        {
            while (commandCount-- > 0)
            {
                // write the :, data length, offset and 00 (data command)
                commandLength = (fgetc(file) | (fgetc(file) << 8));
                dataLength = fgetc(file); // data length
                bootloader_snprintf((char*)buf, 16, ":%.2X%.4X00", dataLength, commandLength);
                serialPortWrite(p->port, buf, 9);

                // write the data - we use the fact that the checksum byte is after the data bytes to loop it in with this loop
                while (dataLength-- > -1)
                {
                    c = (unsigned char)fgetc(file);
                    buf[0] = hexLookupTable[(c >> 4) & 0x0F];
                    buf[1] = hexLookupTable[(c & 0x0F)];
                    serialPortWrite(p->port, buf, 2);
                }

                serialPortWaitForTimeout(p->port, (unsigned char*)".\r\n", 3, BOOTLOADER_TIMEOUT_DEFAULT);

                if (p->uploadProgress != 0)
                {
                    percent = (float)ftell(file) / (float)fileSize;
                    p->uploadProgress(p->obj, percent);
                }
            }
        }
        else if (commandType == 1)
        {
            // raw command, send it straight to the serial port as is
            while (commandCount-- > 0)
            {
                commandLength = fgetc(file);
                c = (unsigned char)fgetc(file);

                // handshake char, ignore
                if (commandLength == 1 && c == 'U')
                {
                    // read sleep interval and timeout interval, ignored
                    c = (unsigned char)fgetc(file);
                    c = (unsigned char)fgetc(file);
                    continue;
                }

                // write command to serial port
                serialPortWrite(p->port, &c, 1);
                while (--commandLength > 0)
                {
                    c = (unsigned char)fgetc(file);
                    serialPortWrite(p->port, &c, 1);
                }

                // read sleep interval and sleep
                c = (unsigned char)fgetc(file);
                commandLength = fgetc(file) * 250;
                if (commandLength != 0)
                {
                    serialPortSleep(p->port, commandLength);
                }

                // read timeout interval
                commandLength = fgetc(file) * 250;
                if (commandLength != 0)
                {
                    unsigned char waitFor[4];
                    bootloader_snprintf((char*)waitFor, 4, "%c\r\n", c);
                    serialPortWaitForTimeout(p->port, waitFor, 3, commandLength);
                }
            }

            if (p->uploadProgress != 0)
            {
                percent = (float)ftell(file) / (float)fileSize;
                p->uploadProgress(p->obj, percent);
            }
        }
        else
        {
            bootloader_perror(p->port, "Invalid data in .bin file\n");
            return 0;
        }
    }

    if (p->uploadProgress != 0 && percent != 1.0f)
    {
        p->uploadProgress(p->obj, 1.0f);
    }

    // re-purpose this variable for the return code
    dataLength = 1;

    if (state.param->flags.bitFields.enableVerify && state.param->verifyProgress != 0)
    {
        dataLength = bootloaderVerify(lastPage, verifyCheckSum, &state);
    }

    if (dataLength == 1)
    {
        // send the "reboot to program mode" command and the device should start in program mode
        serialPortWrite(state.param->port, (unsigned char*)":020000040300F7", 15);
    }

    serialPortSleep(state.param->port, 250);

    return dataLength;
}

static void bootloaderRestartAssist(serial_port_t* s)
{
    // USE WITH CAUTION! This will put in bootloader assist mode allowing a new bootloader to be put on, i.e. SAM-BA

    // restart bootloader assist command
    serialPortWrite(s, (unsigned char*)":020000040700F3", 15);

    // give the device time to start up
    serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
}

static void bootloaderRestart(serial_port_t* s)
{
    // restart bootloader command
    serialPortWrite(s, (unsigned char*)":020000040500F5", 15);

    // give the device time to start up
    serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
}

static int bootloaderSync(serial_port_t* s)
{
    static const unsigned char handshakerChar = 'U';

    //Most usages of this function we do not know if we can communicate (still doing auto-baud or checking to see if the bootloader or application is running) so trying to reset unit here does not make sense.
    //This was probably added because the PC bootloader was doing multiple syncs in a row but the hardware bootloader only allowed one.
    //Will leave it just in case
    // reboot the device in case it is stuck
    bootloaderRestart(s);

    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++)
    {
        if (serialPortWriteAndWaitForTimeout(s, &handshakerChar, 1, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY))
        {
            serialPortSleep(s, BOOTLOADER_REFRESH_DELAY);
            return 1;
        }
    }

    return 0;
}

static int bootloaderHandshake(bootload_params_t* p)
{
    //Port should already be closed, but make sure
    serialPortClose(p->port);

#if ENABLE_BOOTLOADER_BAUD_DETECTION

    // ensure that we start off with a valid baud rate
    if (p->baudRate == 0)
    {
        p->baudRate = bootloaderCycleBaudRate(p->baudRate);
    }

    // try handshaking at each baud rate
    for (unsigned int i = 0; i < _ARRAY_ELEMENT_COUNT(s_baudRateList) + 1; i++)

#endif // ENABLE_BOOTLOADER_BAUD_DETECTION

    {
        if (serialPortOpenInternal(p->port, p->baudRate, p->error, BOOTLOADER_ERROR_LENGTH) == 0)
        {
            // can't open the port, fail
            return 0;
        }
        else if (bootloaderSync(p->port))
        {
            // success, reset baud rate for next round of handshaking
            p->baudRate = 0;
            return 1;
        }

#if ENABLE_BOOTLOADER_BAUD_DETECTION

        // retry at a different baud rate
        serialPortClose(p->port);
        p->baudRate = bootloaderCycleBaudRate(p->baudRate);

#endif // ENABLE_BOOTLOADER_BAUD_DETECTION

    }

    // failed to handshake, fatal error
    bootloader_perror(p->port, "Unable to handshake with bootloader\n");
    return 0;
}

static void bootloadGetVersion(serial_port_t* s, int* major, char* minor, int* sambaAvaliable)
{
	//purge buffer
#define BUF_SIZE    100
	unsigned char buf[BUF_SIZE];
	while (serialPortRead(s, buf, BUF_SIZE)){}

	//Send command
	serialPortWrite(s, (unsigned char*)":020000041000EA", 15);

	//Read Version, SAM-BA Available, and ok (.\r\n) response
	int count = serialPortReadTimeout(s, buf, 8, 1000);

    if (count != 8 || buf[0] != 0xAA || buf[1] != 0x55 || buf[5] != '.' || buf[6] != '\r' || buf[7] != '\n')
    {
        *major = 0;
        *minor = 0;
        *sambaAvaliable = 1;
        return;
    }

    *major = buf[2];
    *minor = buf[3];
    *sambaAvaliable = buf[4] == 0x1;
}

static int bootloadFileInternal(FILE* file, bootload_params_t* p)
{
    if (p->statusText)
        p->statusText(p->obj, "Starting bootloader...");
    if (!enableBootloader(p->port, p->baudRate, p->error, BOOTLOADER_ERROR_LENGTH, p->bootloadEnableCmd))
    {
        //If we have an error, exit
        if (p->error[0] != 0)
            return 0;

        if (p->statusText)
            p->statusText(p->obj, "Unable to find bootloader.");

        if (p->bootName != 0)
        {
            if (p->statusText)
                p->statusText(p->obj, "Attempting to reload bootloader...");

            //Send file
            bootload_params_t params;
            memset(&params, 0, sizeof(params));
            params.fileName = p->bootName;
            params.port = p->port;
            memset(params.error, 0, sizeof(BOOTLOADER_ERROR_LENGTH));
            params.obj = p->obj;
            params.uploadProgress = p->uploadProgress;
            params.verifyProgress = p->verifyProgress;
            params.statusText = p->statusText;
            params.numberOfDevices = 1;
            params.flags.bitFields.enableVerify = 1;

            if (!bootloadUpdateBootloaderSendFile(&params))
                return 0;

            //Restart process now bootloader is updated
            SLEEP_MS(1000);
            return -1;
        }

        return 0;
    }

    // Sync with bootloader
    if (!bootloaderHandshake(p))
    {
        return 0;
    }

    int fileNameLength = (int)strnlen(p->fileName, 255);
    if (fileNameLength > 4 && strncmp(p->fileName + fileNameLength - 4, ".bin", 4) == 0)
    {
        //At this time, binary files are not supported for updating firmware
        bootloader_snprintf(p->error, BOOTLOADER_ERROR_LENGTH, "Binary firmware files are not supported.");
        return 0;

        // it's a .bin file, we will use a far more optimized and memory efficient uploader
        //return bootloaderProcessBinFile(file, p);
    }
    else
    {

#if ENABLE_HEX_BOOT_LOADER

        bootloader_state_t state;
        memset(&state, 0, sizeof(state));
        state.param = p;

		if(!bootloaderNegotiateVersion(&state)) // negotiate version
		{
			return 0;
		}

        int blVerMajor, sambaSupport;
        char blVerMinor;
        bootloadGetVersion(p->port, &blVerMajor, &blVerMinor, &sambaSupport);
        if (blVerMajor > 0 && p->statusText)
        {
            char str[100];
            bootloader_snprintf(str, 100, "Bootloader version %d%c found. SAM-BA is %s on port.", blVerMajor, blVerMinor, sambaSupport == 1 ? "supported" : "NOT supported");
                p->statusText(p->obj, str);
        }

        if ((p->bootName != 0))
        {
            int fileVerMajor = 0;
            char fileVerMinor = 0;
            //Retrieve version from file
            //Only check if we find valid version info in file
            if (bootloadGetBootloaderVersionFromFile(p->bootName, &fileVerMajor, &fileVerMinor) || p->forceBootloaderUpdate > 0)
            {
                //printf("Bootloader file version: %d%c.\n", fileVerMajor, fileVerMinor);

                //Check bootloader version against file
                if (blVerMajor < fileVerMajor || blVerMinor < fileVerMinor || p->forceBootloaderUpdate > 0)
                {
                    if (sambaSupport)
                    {
                        if (p->statusText)
                        {
                            char str[100];
                            if(blVerMajor > 0)
                                bootloader_snprintf(str, 100, "Updating bootloader to version %d%c...", fileVerMajor, fileVerMinor);
                            else
                                bootloader_snprintf(str, 100, "Updating bootloader...");
                            p->statusText(p->obj, str);
                        }

                        //Start SAM-BA
                        bootloaderRestartAssist(p->port);

                        //Send file
                        bootload_params_t params;
                        memset(&params, 0, sizeof(params));
                        params.fileName = p->bootName;
                        params.port = p->port;
                        memset(params.error, 0, sizeof(BOOTLOADER_ERROR_LENGTH));
                        params.obj = p->obj;
                        params.uploadProgress = p->uploadProgress;
                        params.verifyProgress = p->verifyProgress;
                        params.statusText = p->statusText;
                        params.numberOfDevices = 1;
                        params.flags.bitFields.enableVerify = 1;

                        if (!bootloadUpdateBootloaderSendFile(&params))
                            return 0;

                        //Restart process now bootloader is updated
                        SLEEP_MS(1000);
                        return -1;
                    }
                    else
                    {   //Bootloader is out of date but SAM-BA is not supported on port
                        if (p->statusText)
                            p->statusText(p->obj, "WARNING!  Bootloader is out of date. Port used to communicate with bootloader does not support updating the bootloader. To force firmware update, run update without including the bootloader file.");
                        bootloader_snprintf(p->error, BOOTLOADER_ERROR_LENGTH, "Bootloader is out of date but current port does not support updating bootloader. To force update, run firmware update without including the bootloader file.\n");
                        return 0;
                    }
                }
            }            
        }
 
        if (p->statusText)
            p->statusText(p->obj, "Erasing flash...");
        if ( !bootloaderEraseFlash(p->port) /*erase all flash */ || !bootloaderSelectPage(p->port, 0) /*select the first page*/)
            return 0;


        if (p->statusText)
            p->statusText(p->obj, "Programming flash...");
        // begin programming the first page
        if(!bootloaderBeginProgramForCurrentPage(p->port, state.firstPageSkipBytes, FLASH_PAGE_SIZE - 1))
            return 0;

        return bootloaderProcessHexFile(file, &state);

#else

        bootloader_snprintf(lastError, ERROR_BUFFER_SIZE, "Hex bootloader is disabled");
        return 0;

#endif

    }
}

static int samBaWriteWord(serial_port_t* port, uint32_t address, uint32_t value)
{
    unsigned char buf[32];
    int count = SNPRINTF((char*)buf, sizeof(buf), "W%08x,%08x#", address, value);
    return (serialPortWrite(port, buf, count) == count);
}

static uint32_t samBaReadWord(serial_port_t* port, uint32_t address)
{
    unsigned char buf[16];
    int count = SNPRINTF((char*)buf, sizeof(buf), "w%08x,#", address);
    if (serialPortWrite(port, buf, count) == count &&
        (serialPortReadTimeout(port, buf, sizeof(uint32_t), BOOTLOADER_TIMEOUT_DEFAULT)) == sizeof(uint32_t))
    {
        uint32_t val = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
        return val;
    }
    return 0;
}

static uint32_t samBaFlashWaitForReady(serial_port_t* port)
{
    uint32_t status = 0;
    for (int i = 0; i < 10; i++)
    {
        status = samBaReadWord(port, 0x400e0c08);
        if (status & 1)
        {
            break;
        }
        serialPortSleep(port, 20);
    }
    return status;
}

static int samBaSetBootFromFlash(serial_port_t* port)
{
    // EEFC_FCR, EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG_BOOT | EEFC_FCR_FCMD_SGPB
    if (samBaWriteWord(port, 0x400e0c04, 0x5a000000 | 0x00000100 | 0x0000000b))
    {
        return (int)samBaFlashWaitForReady(port);
    }
    return 0;
}

static int samBaFlashEraseWritePage(serial_port_t* port, size_t offset,
                                    unsigned char buf[SAM_BA_FLASH_PAGE_SIZE], int isUSB)
{
    uint16_t page = (uint16_t)(offset / SAM_BA_FLASH_PAGE_SIZE);
    unsigned char _buf[32];
    int count;

    // start lots of data
    if (isUSB)
    {
        count = SNPRINTF((char*)_buf, sizeof(_buf), "S%08x,%08x#",
                         (unsigned int)(SAM_BA_FLASH_START_ADDRESS + offset),
                         (unsigned int)SAM_BA_FLASH_PAGE_SIZE);
        serialPortWrite(port, _buf, count);
        serialPortWrite(port, buf, SAM_BA_FLASH_PAGE_SIZE);
    }
    else
    {
        count = SNPRINTF((char*)_buf, sizeof(_buf), "S%08x,#", (unsigned int)(SAM_BA_FLASH_START_ADDRESS + offset));
        serialPortWrite(port, _buf, count);

        // send page data
        if (xModemSend(port, buf, SAM_BA_FLASH_PAGE_SIZE))
        {
            return 0;
        }
    }

    // EEFC FCR: finish write page
    count = SNPRINTF((char*)_buf, sizeof(_buf), "W%08x,5a%04x03#", 0x400e0c04, page);
    serialPortWrite(port, _buf, count);
    return samBaFlashWaitForReady(port);
}

static int samBaVerify(serial_port_t* port, uint32_t checksum, const void* obj, pfnBootloadProgress verifyProgress)
{
    uint32_t checksum2 = 0;
    uint32_t nextAddress;
    unsigned char buf[512];
    int count;

    for (uint32_t address = SAM_BA_FLASH_START_ADDRESS; address < (SAM_BA_FLASH_START_ADDRESS + SAM_BA_BOOTLOADER_SIZE); )
    {
        nextAddress = address + SAM_BA_FLASH_PAGE_SIZE;
        while (address < nextAddress)
        {
            count = SNPRINTF((char*)buf, sizeof(buf), "w%08x,#", address);
            serialPortWrite(port, buf, count);
            address += sizeof(uint32_t);
            serialPortSleep(port, 2); // give device time to process command
        }
        count = serialPortReadTimeout(port, buf, SAM_BA_FLASH_PAGE_SIZE, BOOTLOADER_TIMEOUT_DEFAULT);
        if (count == SAM_BA_FLASH_PAGE_SIZE)
        {
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum2 ^= *ptr;
            }
        }
        else
        {
            return -1;
        }
        if (verifyProgress != 0)
        {
            verifyProgress(obj, (float)(address - SAM_BA_FLASH_START_ADDRESS) / (float)SAM_BA_BOOTLOADER_SIZE);
        }
    }
    if (checksum != checksum2)
    {
        return -2;
    }
    return 0;
}

static int samBaSoftReset(serial_port_t* port)
{
    // RSTC_CR, RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST
    uint32_t status;
    if (!samBaWriteWord(port, 0x400e1800, 0xa5000000 | 0x00000001))
    {
        return 0;
    }
    for (int i = 0; i < 100; i++)
    {
        status = samBaReadWord(port, 0x400e1804); // RSTC_SR
        if (!(status & 0x00020000)) // RSTC_SR_SRCMP
        {
            return 1;
        }
        serialPortSleep(port, 20);
    }
    return 0;
}

int bootloadFile(serial_port_t* port, const char* fileName, const char * bootName,
    const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress)
{
    bootload_params_t params;
    memset(&params, 0, sizeof(params));
    params.fileName = fileName;
    params.bootName = bootName;
    params.forceBootloaderUpdate = 0;
    params.port = port;
	memset(params.error, 0, sizeof(BOOTLOADER_ERROR_LENGTH));
    params.obj = obj;
    params.uploadProgress = uploadProgress;
    params.verifyProgress = verifyProgress;
    params.numberOfDevices = 1;
    params.flags.bitFields.enableVerify = (verifyProgress != 0);

    return bootloadFileEx(&params);
}

int bootloadFileEx(bootload_params_t* params)
{
    strncpy(params->bootloadEnableCmd, "BLEN", 4);
    int result = 0;

    if (params->error != 0 && BOOTLOADER_ERROR_LENGTH > 0)
    {
        *params->error = '\0';
    }
    
    if (strstr(params->fileName, "EVB") != NULL)
    {   // Enable EVB-2 bootloader
        strncpy(params->bootloadEnableCmd, "EBLE", 4);
    }

    // open the file
    FILE* file = 0;

#ifdef _MSC_VER

    fopen_s(&file, params->fileName, "rb");

#else

    file = fopen(params->fileName, "rb");

#endif

    if (file == 0)
    {
        if (params->error != 0)
        {
            bootloader_snprintf(params->error, BOOTLOADER_ERROR_LENGTH, "Unable to open file: %s", params->fileName);
        }
        return result;
    }

    // If bootloader file was specified, check it is openable
    if (params->bootName && params->bootName[0] != 0)
    {
        FILE* blfile = 0;

#ifdef _MSC_VER

        fopen_s(&blfile, params->bootName, "rb");

#else

        blfile = fopen(params->bootName, "rb");

#endif
        if (blfile == 0)
        {
            bootloader_snprintf(params->error, BOOTLOADER_ERROR_LENGTH, "Unable to open bootloader file: %s", params->bootName);
            return result;
        }

        fclose(blfile);
    }

    result = bootloadFileInternal(file, params);

    //Restart if we updated bootloader
    if (result == -1)
    {
        params->forceBootloaderUpdate = 0;
        result = bootloadFileInternal(file, params);
    }

    if (result == 0)
    {
        // reboot the device back into boot-loader mode
        serialPortWrite(params->port, (unsigned char*)":020000040500F5", 15);
        serialPortSleep(params->port, 500);
    }

    fclose(file);
    serialPortClose(params->port);

    return result;
}

int bootloadGetBootloaderVersionFromFile(const char* bootName, int* verMajor, char* verMinor)
{
    FILE* blfile = 0;

#ifdef _MSC_VER

    fopen_s(&blfile, bootName, "rb");

#else

    blfile = fopen(bootName, "rb");

#endif

    if (blfile == 0)
        return 0;

    fseek(blfile, 0x3DFC, SEEK_SET);
    unsigned char ver_info[4];
	size_t n = fread(ver_info, 1, 4, blfile);
	(void)n;
    fclose(blfile);

    //Check for marker for valid version info
    if (ver_info[0] == 0xAA && ver_info[1] == 0x55)
    {
        if (verMajor)
            *verMajor = ver_info[2];
        if (verMinor)
            *verMinor = ver_info[3];
        return 1;
    }

    //No version found
    return 0;
}

int bootloadUpdateBootloader(serial_port_t* port, const char* fileName,
    const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress)
{
    bootload_params_t params;
    memset(&params, 0, sizeof(params));
    params.fileName = fileName;
    params.port = port;
	memset(params.error, 0, sizeof(BOOTLOADER_ERROR_LENGTH));
    params.obj = obj;
    params.uploadProgress = uploadProgress;
    params.verifyProgress = verifyProgress;
    params.numberOfDevices = 1;
    params.flags.bitFields.enableVerify = (verifyProgress != 0);

    return bootloadUpdateBootloaderEx(&params);
}

static int bootloadUpdateBootloaderSendFile(bootload_params_t* p)
{
    //Setup serial port for correct baud rate
    serialPortClose(p->port);
    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_115200, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }

    // https://github.com/atmelcorp/sam-ba/tree/master/src/plugins/connection/serial
    // https://sourceforge.net/p/lejos/wiki-nxt/SAM-BA%20Protocol/
    unsigned char buf[SAM_BA_FLASH_PAGE_SIZE];
    uint32_t checksum = 0;

    // try non-USB and then USB mode (0 and 1)
    for (int isUSB = 0; isUSB < 2; isUSB++)
    {
        serialPortSleep(p->port, 250);
        serialPortClose(p->port);
        if (!serialPortOpenRetry(p->port, p->port->port, SAM_BA_BAUDRATE, 1))
        {
            bootloader_perror(p, "Failed to open port.\n");
            serialPortClose(p->port);
            return 0;
        }

        // flush
        serialPortWrite(p->port, (void*)"#", 2);
        int count = serialPortReadTimeout(p->port, buf, sizeof(buf), 100);

        // non-interactive mode
        count = serialPortWriteAndWaitFor(p->port, (void*)"N#", 2, (void*)"\n\r", 2);
        if (!count)
        {
            bootloader_perror(p, "Failed to handshake with SAM-BA\n");
            serialPortClose(p->port);
            return 0;
        }

        // set flash mode register
        serialPortWrite(p->port, (const unsigned char*)"W400e0c00,04000600#", 19);

        FILE* file;

#ifdef _MSC_VER

        fopen_s(&file, p->fileName, "rb");

#else

        file = fopen(p->fileName, "rb");

#endif

        if (file == 0)
        {
            bootloader_perror(p, "Unable to load bootloader file\n");
            serialPortClose(p->port);
            return 0;
        }

        fseek(file, 0, SEEK_END);
        int size = ftell(file);
        fseek(file, 0, SEEK_SET);
        checksum = 0;

        if (size != SAM_BA_BOOTLOADER_SIZE)
        {
            bootloader_perror(p, "Invalid bootloader file\n");
            serialPortClose(p->port);
            return 0;
        }

        if (p->statusText)
            p->statusText(p->obj, "Writing bootloader...");

        uint32_t offset = 0;
        while (fread(buf, 1, SAM_BA_FLASH_PAGE_SIZE, file) == SAM_BA_FLASH_PAGE_SIZE)
        {
            if (!samBaFlashEraseWritePage(p->port, offset, buf, isUSB))
            {
                if (!isUSB)
                {
                    offset = 0;
                    break; // try USB mode
                }
                bootloader_perror(p, "Failed to upload page at offset %d\n", (int)offset);
                serialPortClose(p->port);
                return 0;
            }
            for (uint32_t* ptr = (uint32_t*)buf, *ptrEnd = (uint32_t*)(buf + sizeof(buf)); ptr < ptrEnd; ptr++)
            {
                checksum ^= *ptr;
            }
            offset += SAM_BA_FLASH_PAGE_SIZE;
            if (p->uploadProgress != 0)
            {
                p->uploadProgress(p->obj, (float)offset / (float)SAM_BA_BOOTLOADER_SIZE);
            }
        }
        fclose(file);
        if (offset != 0)
        {
            break; // success!
        }
    }

    if (p->verifyProgress != 0)
    {
        if (p->statusText)
            p->statusText(p->obj, "Verifying bootloader...");

        switch (samBaVerify(p->port, checksum, p->obj, p->verifyProgress))
        {
        case -1: bootloader_perror(p, "Flash read error\n"); return 0;
        case -2: bootloader_perror(p, "Flash checksum error\n"); return 0;
        }
    }

    if (!samBaSetBootFromFlash(p->port))
    {
        bootloader_perror(p, "Failed to set boot from flash GPNVM bit\n");
        serialPortClose(p->port);
        return 0;
    }

    if (!samBaSoftReset(p->port))
    {
        bootloader_perror(p, "Failed to reset device\n");
        serialPortClose(p->port);
        return 0;
    }

    serialPortClose(p->port);
    return 1;
}

int bootloadUpdateBootloaderEx(bootload_params_t* p)
{
    strncpy(p->bootloadEnableCmd, "NELB,!!SAM-BA!!\0", 16);
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);
    serialPortSleep(p->port, 250);
    serialPortClose(p->port);

    //Send command to enter SAM-BA mode across different baud rates
    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_3000000, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);
    serialPortSleep(p->port, 250);

    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_921600, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);
    serialPortSleep(p->port, 250);

    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_460800, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);
    serialPortSleep(p->port, 250);

    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_230400, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);
    serialPortSleep(p->port, 250);

    serialPortClose(p->port);
    if (!serialPortOpenRetry(p->port, p->port->port, BAUDRATE_115200, 1))
    {
        bootloader_perror(p, "Failed to open port.\n");
        serialPortClose(p->port);
        return 0;
    }
    serialPortWriteAscii(p->port, p->bootloadEnableCmd, 16);

    //Should be in SAM-BA mode, start sending data
    return bootloadUpdateBootloaderSendFile(p);
}

int enableBootloader(serial_port_t* port, int baudRate, char* error, int errorLength, const char* bootloadEnableCmd)
{
    int baudRates[] = { baudRate, IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

    // detect if device is already in bootloader mode
    bootload_params_t p;
    memset(&p, 0, sizeof(p));
    p.port = port;
	p.baudRate = bootloaderClosestBaudRate(baudRate);

    // attempt to handshake in case we are in bootloader mode
    //When we handshake with legacy bootloader baudrate it prevents us from entering the bootloader when the uINS is running its firmware.
    //Instead of checking for the bootloader first, we will send the command to enter the bootloader without checking.
    //After the command is sent, it will try to handsake again and will find the bootloader even if the bootloader is alreay running before sending the command to enter.
    //if (!bootloaderHandshake(&p))
    {
        // in case we are in program mode, try and send the commands to go into bootloader mode
        unsigned char c = 0;
        for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(baudRates); i++)
        {
            if (baudRates[i] == 0)
                continue;

            serialPortClose(port);
            if (serialPortOpenInternal(port, baudRates[i], error, errorLength) == 0)
            {
                serialPortClose(port);
                return 0;
            }
            for (size_t loop = 0; loop < 10; loop++)
            {
                serialPortWriteAscii(port, "STPB", 4);
                serialPortWriteAscii(port, bootloadEnableCmd, 4);
                c = 0;
                if (serialPortReadCharTimeout(port, &c, 13) == 1)
                {
                    if (c == '$')
                    {
                        // done, we got into bootloader mode
                        i = 9999;
                        break;
                    }
                }
                else
                {
                    serialPortFlush(port);
                }
            }
        }

        serialPortClose(port);
        SLEEP_MS(BOOTLOADER_REFRESH_DELAY);

        // if we can't handshake at this point, bootloader enable has failed
        if (!bootloaderHandshake(&p))
        {
            // failure
            serialPortClose(port);
            return 0;
        }
    }

    // ensure bootloader restarts in fresh state
    bootloaderRestart(port);

    // by this point the bootloader should be enabled
    serialPortClose(port);
    return 1;
}

static int disableBootloaderInternal(serial_port_t* port, char* error, int errorLength, int baud)
{
    // open the serial port
    if (serialPortOpenInternal(port, baud, error, errorLength) == 0)
    {
        return 0;
    }

    // send the "reboot to program mode" command and the device should start in program mode
    serialPortWrite(port, (unsigned char*)":020000040300F7", 15);
    serialPortSleep(port, 250);
    serialPortClose(port);
    return 1;
}


int disableBootloader(serial_port_t* port, char* error, int errorLength)
{
    int result = 0;
    result |= disableBootloaderInternal(port, error, errorLength, IS_BAUD_RATE_BOOTLOADER);
    result |= disableBootloaderInternal(port, error, errorLength, IS_BAUD_RATE_BOOTLOADER_RS232);
	result |= disableBootloaderInternal(port, error, errorLength, IS_BAUD_RATE_BOOTLOADER_SLOW);
	return result;
}
