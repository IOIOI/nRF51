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

#include "PMD_Spi.h"

/* ########################## */
/* SPI COMMANDS               */
/* ########################## */
#define WRITE_ENABLE ((uint8_t) 0x06)
#define WRITE_DISABLE ((uint8_t) 0x04)

#define READ_STATUS_REG ((uint8_t) 0x05)
#define READ_STATUS_REG_CMD_SIZE ((uint8_t) 1)
#define READ_STATUS_REG_BUFFER_SIZE ((uint8_t) 1)
#define WRITE_STATUS_REG ((uint8_t) 01)

#define READ_DATA ((uint8_t) 0x03)
#define READ_CMD_SIZE ((uint8_t) 0x04)

#define FAST_READ ((uint8_t) 0x0B)
#define FAST_READ_CMD_SIZE ((uint8_t) 0x05)

#define PAGE_PROGRAM ((uint8_t) 0x02)

#define SECTOR_ERASE ((uint8_t) 0x20)

#define BLOCK_ERASE ((uint8_t) 0xD8)

#define CHIP_ERASE ((uint8_t) 0xC7)

#define POWER_DOWN ((uint8_t) 0xB9)

#define RELEASE_POWER_DOWN ((uint8_t) 0xAB)

#define MANUFACTURER_DEVICE_ID ((uint8_t) 0x90)
/* ########################## */
/* END OF SPI COMMANDS        */
/* ########################## */

#define MANUFACTURER ((uint8_t) 4)
#define DEVICE_ID ((uint8_t) 5)

#define DUMMY_BYTE ((uint8_t) 0x00)


#define SPI_INSTANCE  0 /**< SPI instance index. */

static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
// static volatile bool flash_wip_flag;

enum spiCommandLayout {
    COMMAND,
    UPPER_ADDRESS_BYTE,
    MIDDLE_ADDRESS_BYTE,
    LOWER_ADDRESS_BYTE,
    DUMMY
};

union StatusRegister {
 uint8_t status;
 uint8_t statusBits[8];
};

static void waitUntilEndOfTransmission();
static void waitUntilChipIsIdle();

static void writeEnable();
static void writeDisable();

static uint8_t readStatusReg();
static void writeStatusReg(uint8_t status);

/* ########################## */
/* BEGINN OF STATIC FUNCTIONS */
/* ########################## */
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
        isIdle = (status & 0x02) || (status & 0x01);
        nrf_delay_us(50);
    } while (isIdle);
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
    uint8_t m_rx_buf[READ_STATUS_REG_BUFFER_SIZE + READ_STATUS_REG_CMD_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);
    memset(m_rx_buf, 0, m_rx_length);

    uint8_t m_tx_buf = READ_STATUS_REG;
    uint8_t m_tx_length = sizeof(m_tx_buf);

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, &m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    return m_rx_buf[1];
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
/* ########################## */
/* END OF STATIC FUNCTIONS    */
/* ########################## */

/* ########################## */
/* BEGINN OF GLOBAL FUNCTIONS */
/* ########################## */
/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event) // TODO: Array Zugriffe abstrahieren
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

//             if(BIN_INFO_OFFEST == p_event->data.done.p_tx_buffer[MIDDLE_ADDRESS_BYTE])
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

/**
 * @brief SPI initialisation.
 */
void spi_init()
{
    LEDS_CONFIGURE(BSP_LED_1_MASK);
    LEDS_OFF(BSP_LED_1_MASK);

    spi_xfer_done = true;

#ifdef DEBUG
    NRF_LOG_PRINTF("SPI Init\r\n");
#endif /* DEBUG */
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi0, &spi_config, spi_event_handler));
}

/**
 * @brief SPI Test function.
 */
void spi_transceiver()
{
    if(0)
    {
        writeDisable();
    }

    LEDS_INVERT(BSP_LED_1_MASK);
}

/**
 * @brief Set the write protection for specific memory areas.
 * @param Protection area (see datasheet for details)
 */
void setMemWriteProtection(uint8_t blockProtection)
{
    writeStatusReg((blockProtection & 0x0F) << 2);
}

/**
 * @brief Get the current memory protection value
 */
uint8_t getMemWriteProtection()
{
    uint8_t status;
    uint8_t curr_block_protection;

    status = readStatusReg();
    curr_block_protection = ((status & 0x3C) >> 2) & 0xF;

    return curr_block_protection;
}

void splitAndStoreData(uint8_t* data, uint32_t data_length, union MemoryAddress addr)
{
    const uint32_t halfPages = data_length / BUFFER_SIZE;
    const uint32_t rest = data_length % BUFFER_SIZE;
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
    uint8_t m_rx_buf[BUFFER_SIZE + READ_CMD_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);
    memset(m_rx_buf, 0, m_rx_length);

    uint8_t m_tx_buf[READ_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

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
    uint8_t m_rx_buf[BUFFER_SIZE + FAST_READ_CMD_SIZE];
    uint8_t m_rx_length = sizeof(m_rx_buf);
    memset(m_rx_buf, 0, m_rx_length);

    uint8_t m_tx_buf[FAST_READ_CMD_SIZE];
    uint8_t m_tx_length = sizeof(m_tx_buf);

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

void pageProgram(uint8_t* data, uint8_t data_length, union MemoryAddress addr)
{
    uint8_t m_tx_buf[BUFFER_SIZE + 4] = {0};
    uint8_t m_tx_length = 4 * sizeof(m_tx_buf[0]) + data_length;
    uint8_t index = 0;

    if(NULL == (data) || (0 == data_length))
    {
        return;
    }

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

void sectorErase(union MemoryAddress addr)
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

void blockErase(union MemoryAddress addr)
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

#ifdef DEBUG
uint8_t getDeviceID()
{
    uint8_t m_rx_buf[5];
    uint8_t m_rx_length = sizeof(m_rx_buf);
    memset(m_rx_buf, 0, m_rx_length);

    uint8_t m_tx_buf[4];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    m_tx_buf[COMMAND] = RELEASE_POWER_DOWN;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    return m_rx_buf[4];
}

uint16_t getManufacturerAndDeviceId()
{
    uint16_t retVal = 0;
    uint8_t m_rx_buf[6];
    uint8_t m_rx_length = sizeof(m_rx_buf);
    memset(m_rx_buf, 0, m_rx_length);

    uint8_t m_tx_buf[4];
    uint8_t m_tx_length = sizeof(m_tx_buf);

    m_tx_buf[COMMAND] = MANUFACTURER_DEVICE_ID;
    m_tx_buf[UPPER_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[MIDDLE_ADDRESS_BYTE] = DUMMY_BYTE;
    m_tx_buf[LOWER_ADDRESS_BYTE] = 0x00;

    spi_xfer_done = false;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0, m_tx_buf, m_tx_length, m_rx_buf, m_rx_length));

    waitUntilEndOfTransmission();

    retVal = m_rx_buf[4];
    retVal = retVal << 8;
    retVal = retVal | m_rx_buf[5];

    return retVal;
}
#endif /* DEBUG */
/* ########################## */
/* END OF GLOBAL FUNCTIONS    */
/* ########################## */
