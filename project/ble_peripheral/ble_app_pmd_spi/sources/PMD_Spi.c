/* Copyright (C) 2015 Daniel Tatzel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "PMD_Spi.h"

#define DEBUG

#define WRITE_ENABLE ((uint8_t) 0x06)
#define WRITE_DISABLE ((uint8_t) 0x04)
#define READ_STATUS_REG ((uint8_t) 0x05)
#define WRITE_STATUS_REG ((uint8_t) 0x01)
#define READ_DATA ((uint8_t) 0x03)
#define FAST_READ ((uint8_t) 0x0B)
#define PAGE_PROGRAM ((uint8_t) 0x02)
#define SECTOR_ERASE ((uint8_t) 0x20)
#define BLOCK_ERASE ((uint8_t) 0xD8)
#define CHIP_ERASE ((uint8_t) 0xC7) /* Duration up to 7s seconds */
#define POWER_DOWN ((uint8_t) 0xB9)
#define RELEASE_POWER_DOWN ((uint8_t) 0xAB)   /* Also Device-ID when not in deep power down */
#define MANUFACTURER_DEVICE_ID ((uint8_t) 0x90)

#define MANUFACTURER ((uint8_t) 4)
#define DEVICE_ID ((uint8_t) 5)

#define MEMORY_SIZE ((uint32_t) 0x100000)

#define BINARY_1_ADDR ((uint32_t) 0x0)
#define BINARY_2_ADDR ((uint32_t) 0x80000)

#define BINARY_INFO_OFFEST ((uint32_t) 0x0)
#define MASTER_KEY_OFFEST ((uint32_t) 0x100)
#define SLAVE_KEY_OFFEST ((uint32_t) 0x180)
#define SIG_OFFEST ((uint32_t) 0x200)
#define BINARY_OFFEST ((uint32_t) 0x300)

#define DUMMY_BYTE ((uint8_t) 0x00)
#define PAGE_PROGRAMM_SIZE ((uint8_t) 0x80)

#define SPI_INSTANCE  0 /**< SPI instance index. */

static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool flash_wip_flag;
static volatile uint8_t curr_block_protection;
static volatile uint32_t binary_size;

enum AddressBytes {
    LOWER_BYTE,
    MIDDLE_BYTE,
    UPPER_BYTE
};

enum spiCommandLayout {
    COMMAND,
    UPPER_ADDRESS_BYTE,
    MIDDLE_ADDRESS_BYTE,
    LOWER_ADDRESS_BYTE,
    DUMMY
};

union MemoryAddress {
 uint32_t address;
 uint8_t addressBytes[3];
};

union StatusRegister {
 uint8_t status;
 uint8_t statusBits[8];
};

static void waitUntilEndOfTransmission();
static void writeStatusReg(uint8_t status);
static void eraseBinary(uint32_t binaryStartAddress);

static void writeEnable();
static void writeDisable();

static void readStatusReg();

static void readData(union MemoryAddress addr);
static void fastReadData(union MemoryAddress addr);

static void pageProgram(uint8_t* data, uint8_t data_length, union MemoryAddress addr);

static void sectorErase(union MemoryAddress addr);
static void blockErase(union MemoryAddress addr);
static void chipErase();

static void powerDown();
static void releasePowerDown();

void splitAndStoreData(uint8_t* data, uint32_t data_length, const uint32_t addr);

/**
 * @brief SPI user event handler.
 * @param event
 */
void pmd_spi_event_handler(nrf_drv_spi_evt_t const * p_event) // TODO: Array Zugriffe abstrahieren
{
    spi_xfer_done = true;
    int8_t index = 0;
    
    switch(p_event->data.done.p_tx_buffer[COMMAND])
    {
        case WRITE_ENABLE:
            /* Nothing to do */
        break;
        
        case WRITE_DISABLE:
            /* Nothing to do */
        break;
        
        case READ_STATUS_REG:
            flash_wip_flag = p_event->data.done.p_rx_buffer[1] & 0x01;
            curr_block_protection = ((p_event->data.done.p_rx_buffer[1] & 0x3C) & 0xF) >> 2;

#ifdef DEBUG
            NRF_LOG("Status register: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[1]);
            NRF_LOG("\r\n");
#endif
        break;
        
        case WRITE_STATUS_REG:
            /* Nothing to do */
        break;
        
        case READ_DATA:            
#ifdef DEBUG
            NRF_LOG("Read data (hex):");
            
            for(index = p_event->data.done.tx_length; index < p_event->data.done.rx_length; index++)
            {
                NRF_LOG(" ");
                NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[index]);
            }
            
            NRF_LOG("\r\n");
#endif
            
            
//             if(BINARY_INFO_OFFEST == p_event->data.done.p_tx_buffer[MIDDLE_ADDRESS_BYTE])
//             {
//                 p_event->data.done.p_rx_buffer[4];
//             }
        break;
        
        case FAST_READ:
#ifdef DEBUG
            NRF_LOG("Fast read data (hex):");
            
            for(index = p_event->data.done.tx_length; index < p_event->data.done.rx_length; index++)
            {
                NRF_LOG(" ");
                NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[index]);
            }
            
            NRF_LOG("\r\n");
#endif
        break;
        
        case PAGE_PROGRAM:
            /* Nothing to do */
        break;
        
        case SECTOR_ERASE:
            /* Nothing to do */
        break;
        
        case BLOCK_ERASE:
            /* Nothing to do */
        break;
        
        case CHIP_ERASE:
            /* Nothing to do */
        break;
        
        case POWER_DOWN:
            /* Nothing to do */
        break;

        case RELEASE_POWER_DOWN:
            // Device ID zurückgeben
#ifdef DEBUG
            if (NULL != p_event->data.done.p_rx_buffer || 0 == p_event->data.done.rx_length)
            {
                NRF_LOG("Device ID: 0x");
                NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[DEVICE_ID - 1]);
                NRF_LOG("\r\n");
            }
#endif
        break;
        
        case MANUFACTURER_DEVICE_ID:
#ifdef DEBUG
            NRF_LOG("Manufacturer: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[MANUFACTURER]);
            NRF_LOG(" Device ID: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[DEVICE_ID]);
            NRF_LOG("\r\n");
#endif
        break;
        
        default:
            /* Error handling */
        break;
    }
}

void pmd_spi_init()
{
    LEDS_CONFIGURE(BSP_LED_1_MASK);
    LEDS_OFF(BSP_LED_1_MASK);
    
    spi_xfer_done = true;
    flash_wip_flag = false;
    binary_size = 0;

#ifdef DEBUG
    NRF_LOG_PRINTF("SPI Init\r\n");
#endif
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi0, &spi_config, pmd_spi_event_handler));
}

void pmd_spi_transceiver()
{
//     getManufacturerAndDeviceId();
//     releasePowerDown();
//     sectorErase();
//     nrf_delay_ms(300);
//     uint8_t data[7] = {127,0,0,1,2,3,4};
// //     uint8_t data[7] = {4,3,2,1,0,0,127};
//     pageProgram(data, sizeof(data));
//     nrf_delay_ms(7);
//     readData();
    
    if(0)
    {
        union MemoryAddress bob;
        bob.address = 0x00000000;
        
        writeDisable();
        readData(bob);
        fastReadData(bob);
        pageProgram(NULL, 0, bob);
        sectorErase(bob);
        blockErase(bob);
        chipErase();
        powerDown();
        releasePowerDown();
        eraseBinary(BINARY_1_ADDR);
        splitAndStoreData(NULL, 0, bob.address);
    }
    
//     setMemWriteProtection(2);
//     NRF_LOG_HEX_CHAR(getMemWriteProtection());
//     NRF_LOG("\r\n");
    
    LEDS_INVERT(BSP_LED_1_MASK);
}

static void waitUntilEndOfTransmission()
{
    while (!spi_xfer_done)
    {
        __WFE();
    }
}

static void waitUntilChipIsIdle()
{
    do
    {
        readStatusReg();
    } while (flash_wip_flag);
}

void setMemWriteProtection(uint8_t blockProtection) /* See datasheet for details */
{
    writeStatusReg((blockProtection & 0x0F) << 2);
}

uint8_t getMemWriteProtection()
{
    readStatusReg();
    
    return curr_block_protection;
}

static void eraseBinary(uint32_t binaryStartAddress) // TODO: Test
{
    int index;
    union MemoryAddress tmpAddr;
    
    tmpAddr.address = binaryStartAddress;
    
    for(index = 0; index < 8; index++) // TODO: Magical Numbers ersetzen
    {
//         NRF_LOG_HEX(tmpAddr.address);
//         NRF_LOG("\r\n");
        blockErase(tmpAddr);
        tmpAddr.addressBytes[UPPER_BYTE]++;
    }
}

// uint32_t (uint32_t data_length, const uint32_t addr) // TODO: Name und auslesen etc
// {
//     const uint32_t currAddress = addr + BINARY_OFFEST + binary_size;
//     uint32_t addrLimit;
// 
//     if(BINARY_1_ADDR == addr) // ersetzen durch auslesen und angabe fixer adresse, über Address-Byte abgleich
//     {
//         addrLimit = BINARY_2_ADDR;
//     }
//     else if(BINARY_2_ADDR == addr)
//     {
//         addrLimit = MEMORY_SIZE;
//     }
//     else
//     {
//         return ERROR_WRONG_ADDRESS;
//     }
//     
//     if(currAddress >= addrLimit)
//     {
//         eraseBinary(addr);
//         binary_size = 0;
//         return ERROR_BINARY_SIZE;
//     }
//     
//     binary_size += data_length; // muss hier mitgezählt werden wegen addresse
// }

void splitAndStoreData(uint8_t* data, uint32_t data_length, const uint32_t addr) // TODO: Test
{
    uint32_t halfPages = data_length / PAGE_PROGRAMM_SIZE;
    uint32_t rest = data_length % PAGE_PROGRAMM_SIZE;
    uint32_t index;
    union MemoryAddress tmpAddr;
    
    tmpAddr.address = addr + BINARY_OFFEST + binary_size;
    
    for(index = 0; index < halfPages; index++)
    {
        pageProgram(&data[index * PAGE_PROGRAMM_SIZE], PAGE_PROGRAMM_SIZE, tmpAddr);
        tmpAddr.address += PAGE_PROGRAMM_SIZE;
    }
    
    if(0 != rest)
    {
        pageProgram(&data[halfPages * PAGE_PROGRAMM_SIZE], rest, tmpAddr);
    }
}

static void writeEnable()
{
    uint8_t m_tx_buf = WRITE_ENABLE;
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    spi_xfer_done = false;
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));
    
    waitUntilEndOfTransmission();
}

static void writeDisable()
{
    uint8_t m_tx_buf = WRITE_DISABLE;
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    waitUntilEndOfTransmission();

    spi_xfer_done = false;
        
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));
}

static void readStatusReg()
{
    uint8_t m_rx_buf[2] = {0};
    uint8_t m_rx_length = sizeof(m_rx_buf);
    
    uint8_t m_tx_buf = READ_STATUS_REG;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();
}

static void writeStatusReg(uint8_t status)
{
    uint8_t m_tx_buf[2] = {0};
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = WRITE_STATUS_REG;
    m_tx_buf[1] = status;

    writeEnable();

    spi_xfer_done = false;
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));
    
    waitUntilEndOfTransmission();
    
    waitUntilChipIsIdle();
}

static void readData(union MemoryAddress addr) // TODO: Umbau auf 128 Byte
{
    uint8_t m_rx_buf[11] = {0};
    uint8_t m_rx_length = sizeof(m_rx_buf);
    
    uint8_t m_tx_buf[4];    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = READ_DATA;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();
}

static void fastReadData(union MemoryAddress addr) // TODO: Umbau auf 128 Byte
{
    uint8_t m_rx_buf[12] = {0};
    uint8_t m_rx_length = sizeof(m_rx_buf);
    
    uint8_t m_tx_buf[5];    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = FAST_READ;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];
    m_tx_buf[DUMMY] = DUMMY_BYTE;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();
}

static void pageProgram(uint8_t* data, uint8_t data_length, union MemoryAddress addr)
{
    uint8_t m_tx_buf[PAGE_PROGRAMM_SIZE + 4] = {0};    
    uint8_t m_tx_length = 4 * sizeof(m_tx_buf[0]) + data_length;
    uint8_t index = 0;
    
    m_tx_buf[COMMAND] = PAGE_PROGRAM;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];
    
    for(index = 0; index < data_length; index++)
    {
        m_tx_buf[index+4] = data[index];
    }
    
    writeEnable();

    spi_xfer_done = false;
    
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
    
    waitUntilChipIsIdle();
}

static void sectorErase(union MemoryAddress addr)
{
    uint8_t m_tx_buf[4] = {0};    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = SECTOR_ERASE;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];
    
    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
    
    waitUntilChipIsIdle();
}

static void blockErase(union MemoryAddress addr)
{
    uint8_t m_tx_buf[4] = {0};    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = BLOCK_ERASE;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];
    
    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
    
    waitUntilChipIsIdle();
}

static void chipErase()
{
    uint8_t m_tx_buf = CHIP_ERASE;
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
    
    waitUntilChipIsIdle();
}

static void powerDown()
{
    uint8_t m_tx_buf = POWER_DOWN;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
}

static void releasePowerDown()
{
    uint8_t m_tx_buf[4];    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = RELEASE_POWER_DOWN;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
    
    nrf_delay_us(5);
}

void getDeviceID()
{
    uint8_t m_rx_buf[5] = {0};
    uint8_t m_rx_length = sizeof(m_rx_buf);
    
    uint8_t m_tx_buf[4];    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = RELEASE_POWER_DOWN;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();
}

void getManufacturerAndDeviceId()
{
    uint8_t m_rx_buf[6] = {0};
    uint8_t m_rx_length = sizeof(m_rx_buf);
    
    uint8_t m_tx_buf[4];    
    uint8_t m_tx_length = sizeof(m_tx_buf);
    
    m_tx_buf[COMMAND] = MANUFACTURER_DEVICE_ID;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();
}

