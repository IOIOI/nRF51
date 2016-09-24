/* Copyright (C) 2016 Daniel Tatzel
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

/*************************************************************************************************************************//**
* \file PMD_SpiFlash.c                                              <!-- Has to be changed according to the real file name -->
*
* \brief This module uses SPI to store / erase / read data grom the flash memory.                   <!-- Brief description -->
*
* This module is specific to the used flash memory chip.                                         <!-- Detailed description -->
* It uses SPI to communicate with the memory.
*
******************************************************************************************************************************
* <!-- Authors -->
******************************************************************************************************************************
*
* \author Daniel Tatzel                                                                      <!-- One line for each author -->
*
*****************************************************************************************************************************/

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include "PMD_SpiFlash.h"

/*===========================================================================================================================|
|  LOCAL CONSTANT / FUNCTION MACROS                                                                                          |
============================================================================================================================*/
#define WRITE_ENABLE ((uint8_t) 0x06)                              /**< Write enable command. */
#define WRITE_DISABLE ((uint8_t) 0x04)                             /**< Write disable command. */

#define READ_STATUS_REG ((uint8_t) 0x05)                           /**< Command for reading the status register of the flash chip. */
#define READ_STATUS_REG_CMD_SIZE ((uint8_t) 1)                     /**< Size of the read status register command. */
#define READ_STATUS_REG_BUFFER_SIZE ((uint8_t) 1)                  /**< Buffer size for the received status register content. */

#define WRITE_STATUS_REG ((uint8_t) 0x01)                          /**< Write status register command. */
#define WRITE_STATUS_REG_CMD_SIZE ((uint8_t) 2)                    /**< Size of the write status register command. */

#define READ_DATA ((uint8_t) 0x03)                                 /**< Read data command. */
#define READ_CMD_SIZE ((uint8_t) 4)                                /**< Size of the read data command. */

#define FAST_READ ((uint8_t) 0x0B)                                 /**< Fast read data command. */
#define FAST_READ_CMD_SIZE ((uint8_t) 0x05)                        /**< Size of the fast read data command. */

#define PAGE_PROGRAM ((uint8_t) 0x02)                              /**< Page program command (store data). */
#define PAGE_PROGRAM_CMD_SIZE ((uint8_t) 4)                        /**< Size of the page program command. */

#define SECTOR_ERASE ((uint8_t) 0x20)                              /**< Sector erase command. */
#define SECTOR_ERASE_CMD_SIZE ((uint8_t) 4)                        /**< Size of the sector erase command. */

#define BLOCK_ERASE ((uint8_t) 0xD8)                               /**< Block erase command. */
#define BLOCK_ERASE_CMD_SIZE ((uint8_t) 4)                         /**< Size of the block erase command. */

#define CHIP_ERASE ((uint8_t) 0xC7)                                /**< Chip erase command. */

#define POWER_DOWN ((uint8_t) 0xB9)                                /**< Power down command (sleep mode). */

#define RELEASE_POWER_DOWN ((uint8_t) 0xAB)                        /**< Relese power down command (wakeup). */
#define RELEASE_POWER_DOWN_CMD_SIZE ((uint8_t) 4)                  /**< Size of the release power down command. */

#define DEVICE_ID_CMD_SIZE ((uint8_t) RELEASE_POWER_DOWN_CMD_SIZE) /**< Device ID command. */
#define DEVICE_ID_BUFFER_SIZE ((uint8_t) 1)                        /**< Size of the device ID command. */

#define MANUFACTURER_DEVICE_ID ((uint8_t) 0x90)                    /**< Command for reading the manufacturer and device ID. */
#define MANUFACTURER_DEVICE_ID_CMD_SIZE ((uint8_t) 4)              /**< Size of the manufacturer and device ID command. */
#define MANUFACTURER_DEVICE_ID_BUFFER_SIZE ((uint8_t) 2)           /**< Buffer size for the response of manaufacturer and devie ID command. */

#define STATUS_REG_WIP_BIT ((uint8_t) 0x01)                        /**< Bit position of the Work / Write In Progress bit. */
#define STATUS_REG_WEL_BIT ((uint8_t) 0x02)                        /**< Bit position of the Write Enable Latch bit. */

#define MEM_PROTECTION_BIT_MASK ((uint8_t) 0x3C)                   /**< Bit mask for getting the memory protection value of the status Register. */

#define MANUFACTURER ((uint8_t) 4)                                 /**< Byte position of the manufacturer ID. */
#define DEVICE_ID ((uint8_t) 5)                                    /**< Byte position of the device ID. */

#define SPI_INSTANCE  0                                            /**< SPI instance index. */

/*===========================================================================================================================|
|  LOCAL DATA TYPES AND STRUCTURES                                                                                           |
============================================================================================================================*/
/**@brief Enum for accessing the bytes of the flash commands. */
enum spiCommandLayout {
    COMMAND,
    UPPER_ADDRESS_BYTE,
    MIDDLE_ADDRESS_BYTE,
    LOWER_ADDRESS_BYTE,
    DUMMY
};

/**@brief Union for the content of the status register. */
union StatusRegister {
 uint8_t status;
 uint8_t statusBits[BYTE_SIZE];
};


/*===========================================================================================================================|
|  LOCAL DATA PROTOTYPES                                                                                                     |
============================================================================================================================*/
static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;                                    /**< Flag used to indicate that SPI instance completed the transfer. */

/*===========================================================================================================================|
| LOCAL FUNCTION PROTOTYPES                                                                                                  |
|===========================================================================================================================*/
/**
 * @brief Function that blocks furhter communication until last transmission has finished.
 */
static void waitUntilEndOfTransmission();

/**
 * @brief Function that blocks furhter communication until the chip is idle again.
 */
static void waitUntilChipIsIdle();

/**
 * @brief Set the write enable latch bit of the spi flash chip.
 *
 * This function has to be called before any write or erase operation.
 */
static void writeEnable();

/**
 * @brief Reset the write enable latch bit of the spi flash chip.
 */
static void writeDisable();

/**
 * @brief Function for reading the status register of the spi flash.
 *
*  @retval Status Value of the status register.
 */
static uint8_t readStatusReg();

/**
 * @brief Reset the write enable latch bit of the spi flash chip.
 *
 * @param[in] status    Value which gets written into the status register..
 */
static void writeStatusReg(uint8_t status);

/*===========================================================================================================================|
|  LOCAL FUNCTIONS                                                                                                           |
|===========================================================================================================================*/
static void waitUntilEndOfTransmission()
{
    while (!spi_xfer_done)
    {
        __WFE();
    }
}

static void waitUntilChipIsIdle()
{
    bool isIdle;
    uint8_t status;

    do
    {
        status = readStatusReg();
        isIdle = (status & STATUS_REG_WEL_BIT) || (status & STATUS_REG_WIP_BIT);

        nrf_delay_us(50);
    }
    while (isIdle);
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

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
}

static uint8_t readStatusReg()
{
    uint8_t m_rx_buf[READ_STATUS_REG_CMD_SIZE + READ_STATUS_REG_BUFFER_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);

    uint8_t m_tx_buf = READ_STATUS_REG;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    memset(m_rx_buf, 0, m_rx_length);

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    return m_rx_buf[1];
}

static void writeStatusReg(uint8_t status)
{
    uint8_t m_tx_buf[WRITE_STATUS_REG_CMD_SIZE] = {0};
    uint8_t m_tx_length = sizeof(m_tx_buf);

    m_tx_buf[COMMAND] = WRITE_STATUS_REG;
    m_tx_buf[1] = status;

    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();

    waitUntilChipIsIdle();
}

/*===========================================================================================================================|
|  GLOBAL FUNCTIONS                                                                                                          |
|===========================================================================================================================*/
void spiEventHandler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;

#ifdef EBUG
    int8_t index = 0;
#endif /* DEBUG */

    switch(p_event->data.done.p_tx_buffer[COMMAND])
    {
        case WRITE_ENABLE:
            /* Nothing to do */
        break;

        case WRITE_DISABLE:
            /* Nothing to do */
        break;

        case READ_STATUS_REG:
#ifdef DEBUG
            NRF_LOG("Status register: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[1]);
            NRF_LOG("\r\n");
#endif /* DEBUG */
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
#endif /* DEBUG */
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
#endif /* DEBUG */
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
#endif /* DEBUG */
        break;

        case MANUFACTURER_DEVICE_ID:
#ifdef DEBUG
            NRF_LOG("Manufacturer: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[MANUFACTURER]);
            NRF_LOG(" Device ID: 0x");
            NRF_LOG_HEX_CHAR(p_event->data.done.p_rx_buffer[DEVICE_ID]);
            NRF_LOG("\r\n");
#endif /* DEBUG */
        break;

        default:
            /* Error handling */
        break;
    }
}

void initSPI()
{
    LEDS_CONFIGURE(BSP_LED_1_MASK);
    LEDS_OFF(BSP_LED_1_MASK);

    spi_xfer_done = true;

#ifdef DEBUG
    NRF_LOG_PRINTF("SPI Init\r\n");
#endif /* DEBUG */

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi0, &spi_config, spiEventHandler));
}

void spiTestFnc()
{
    if(0)
    {
        writeDisable();
    }

    LEDS_INVERT(BSP_LED_1_MASK);
}

void setMemWriteProtection(uint8_t blockProtection)
{
    writeStatusReg((blockProtection & 0x0F) << 2);
}

uint8_t getMemWriteProtection()
{
    uint8_t status;
    uint8_t curr_block_protection;

    status = readStatusReg();
    curr_block_protection = ((status & MEM_PROTECTION_BIT_MASK) >> 2) & 0xF;

    return curr_block_protection;
}

void splitAndStoreData(uint8_t* data, uint32_t dataLength, union MemoryAddress addr)
{
    const uint32_t halfPages = dataLength / BUFFER_SIZE;
    const uint32_t rest = dataLength % BUFFER_SIZE;
    uint32_t index;

    for(index = 0; halfPages > index; index++)
    {
        pageProgram(&data[index * BUFFER_SIZE], BUFFER_SIZE, addr);
        addr.address += BUFFER_SIZE;
    }

    if(0 != rest)
    {
        pageProgram(&data[halfPages * BUFFER_SIZE], rest, addr);
    }
}

#ifndef USE_FAST_READ
void readData(union MemoryAddress addr, struct transmissionData* retData)
{
    uint8_t m_rx_buf[READ_CMD_SIZE + BUFFER_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);

    uint8_t m_tx_buf[READ_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    memset(m_rx_buf, 0, m_rx_length);
    m_tx_buf[COMMAND] = READ_DATA;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    if(NULL != retData)
    {
        memcpy(retData->rx_data, &m_rx_buf[READ_CMD_SIZE], BUFFER_SIZE);
        retData->rx_length = BUFFER_SIZE;
        retData->tx_length = 0;
    }
}

#else

void fastReadData(union MemoryAddress addr, struct transmissionData* retData)
{
    uint8_t m_rx_buf[FAST_READ_CMD_SIZE + BUFFER_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);

    uint8_t m_tx_buf[FAST_READ_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    memset(m_rx_buf, 0, m_rx_length);
    m_tx_buf[COMMAND] = FAST_READ;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];
    m_tx_buf[DUMMY] = DUMMY_BYTE;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    if(NULL != retData)
    {
        memcpy(retData->rx_data, &m_rx_buf[FAST_READ_CMD_SIZE], BUFFER_SIZE);
        retData->rx_length = BUFFER_SIZE;
        retData->tx_length = 0;
    }
}
#endif /* USE_FAST_READ */

void pageProgram(uint8_t* data, uint8_t dataLength, union MemoryAddress addr)
{
    uint8_t m_tx_buf[PAGE_PROGRAM_CMD_SIZE + BUFFER_SIZE] = {0};
    uint8_t m_tx_length = PAGE_PROGRAM_CMD_SIZE * sizeof(m_tx_buf[0]) + dataLength;
    uint8_t index = 0;

    if(NULL == (data) || (0 == dataLength))
    {
        return;
    }

    m_tx_buf[COMMAND] = PAGE_PROGRAM;
    m_tx_buf[UPPER_ADDRESS_BYTE] = addr.addressBytes[UPPER_BYTE];
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = addr.addressBytes[MIDDLE_BYTE];
    m_tx_buf[LOWER_ADDRESS_BYTE] = addr.addressBytes[LOWER_BYTE];

    for(index = 0; index < dataLength; index++)
    {
        m_tx_buf[index + PAGE_PROGRAM_CMD_SIZE] = data[index];
    }

    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();

    waitUntilChipIsIdle();
}

void sectorErase(union MemoryAddress addr)
{
    uint8_t m_tx_buf[SECTOR_ERASE_CMD_SIZE] = {0};
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

void blockErase(union MemoryAddress addr)
{
    uint8_t m_tx_buf[BLOCK_ERASE_CMD_SIZE] = {0};
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

void chipErase()
{
    uint8_t m_tx_buf = CHIP_ERASE;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    writeEnable();

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();

    waitUntilChipIsIdle();
}

void powerDown()
{
    uint8_t m_tx_buf = POWER_DOWN;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();
}

void releasePowerDown()
{
    uint8_t m_tx_buf[RELEASE_POWER_DOWN_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    m_tx_buf[COMMAND] = RELEASE_POWER_DOWN;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = DUMMY_BYTE;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, NULL, 0));

    waitUntilEndOfTransmission();

    nrf_delay_us(5);
}

#ifdef DEBUG
uint8_t getDeviceID()
{
    uint8_t m_rx_buf[DEVICE_ID_CMD_SIZE + DEVICE_ID_BUFFER_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);

    uint8_t m_tx_buf[DEVICE_ID_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    memset(m_rx_buf, 0, m_rx_length);
    m_tx_buf[COMMAND] = RELEASE_POWER_DOWN;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = DUMMY_BYTE;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    return m_rx_buf[DEVICE_ID - 1];
}

uint16_t getManufacturerAndDeviceId()
{
    uint8_t m_rx_buf[MANUFACTURER_DEVICE_ID_CMD_SIZE + MANUFACTURER_DEVICE_ID_BUFFER_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);

    uint8_t m_tx_buf[MANUFACTURER_DEVICE_ID_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    uint16_t retVal = 0;

    memset(m_rx_buf, 0, m_rx_length);
    m_tx_buf[COMMAND] = MANUFACTURER_DEVICE_ID;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    retVal = m_rx_buf[MANUFACTURER];
    retVal = retVal << BYTE_SIZE;
    retVal = retVal | m_rx_buf[DEVICE_ID];

    return retVal;
}
#endif /* DEBUG */
