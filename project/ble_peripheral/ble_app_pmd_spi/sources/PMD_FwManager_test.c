#include "PMD_FwManager.h"
#include "PMD_SpiFlash.h"
#include "PMD_FwManager_test.h"

#define BIN_1_ADDR ((uint32_t) 0x0)
#define BIN_2_ADDR ((uint32_t) 0x80000)
#define SIZE 260

void fwM_testFncWrite()
{
    union MemoryAddress addr;
    uint8_t data[SIZE];
    uint32_t dataLength = SIZE;
    uint16_t i;

    chipErase();

    for(i = 0; i < SIZE; i++)
    {
        data[i] = i % 256;
    }

    addr.address = BIN_1_ADDR;

    splitAndStoreData(data, dataLength, addr);
}

void fwM_testFncRead(uint32_t tmpAddr)
{
    union MemoryAddress addr;
    struct transmissionData recData;
    uint16_t i, j;

    addr.address = tmpAddr;

    for(i = 0; i < 5; i++)
    {
        addr.address = tmpAddr + i * BUFFER_SIZE;
        NRF_LOG_PRINTF("Reading halfpage: %c\r\n", i + 48);
        NRF_LOG_HEX(addr.address);
        NRF_LOG("\r\n");
        readData(addr, &recData);

        for(j = 0; j < recData.rx_length; j++)
        {
            NRF_LOG_HEX_CHAR(recData.rx_data[j]);
            NRF_LOG(" ");
        }

        NRF_LOG("\r\n\n\n");

        memset(recData.rx_data,0, BUFFER_SIZE);
        recData.rx_length = 0;
    }
}

void fwM_testFncInfo()
{
    struct binaryInfo info;
    getBinaryInfo(BIN_1_ADDR, &info);

    NRF_LOG_HEX_CHAR(info.status);
    NRF_LOG(" ");
    NRF_LOG_HEX(info.size);
    NRF_LOG(" ");
    NRF_LOG_HEX(info.version);
    NRF_LOG(" ");
    NRF_LOG_HEX(info.hash);
    NRF_LOG(" ");
    NRF_LOG_HEX(info.crc);
    NRF_LOG("\r\n");
}

void fwM_testFncBin()
{
    uint8_t data[600];
    uint32_t dataLength = 384;
    uint16_t i;

    chipErase();

    // Half Page 0
    data[0] = 0xFF;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x58;

    for(i = 0x05; i < 0x13; i++)
    {
        data[i] = i % 256;
    }

    for(i = 0x13; i < 0x80; i++)
    {
        data[i] = 0xAA;
    }

    // Half Page 1
    for(i = 0x80; i < 0x100; i++)
    {
        data[i] = i-0x80 % 256;
    }

    // Half Page 2
    for(i = 0x100; i < 0x180; i++)
    {
        data[i] = i-0x100 % 256;
    }

    // Half Page 3
    for(i = 0x180; i < 0x200; i++)
    {
        data[i] = i-0x180 % 256;
    }

    // Half Page 4
    for(i = 0x200; i < 0x258; i++)
    {
        data[i] = i-0x200 % 256;
    }

//     writeBinaryStatus(0x00, BIN_1_ADDR);

    fwM_testFncInfo();

    writeBinary(data, dataLength);
    dataLength = 216;
    writeBinary(&data[384], dataLength);
}

void fwM_testFnc()
{
//     fwM_testFncWrite();
    fwM_testFncBin();
    fwM_testFncInfo();
    fwM_testFncRead(0);
    fwM_testFncRead(0x80000);
}
