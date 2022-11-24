/**
 * @file il3829_core.c
 * @brief E-Paper IL3829 kernel driver implementation for Zephir OS
 *
 * @version 0.1
 * @copyright 2022 Nikita Bulaev (bnv@bulki.me)
 *
 * @note Several ideas are taken from suppliers examples at https://github.com/MHEtLive/MH-ET-LIVE-E-Papers
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define DT_DRV_COMPAT ilitek_il3829

#include "il3829_core.h"

#include <stdio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(IL3829_DISPLAY, CONFIG_DISPLAY_LOG_LEVEL);

/* Private typedef */
/** Display data struct */
typedef struct {
    bool newDataFlag;                      /*!< Flag to indicate new data is available */
    enum display_pixel_format pixelFormat; /*!< Pixel format */
    enum display_orientation orientation;  /*!< Display orientation */
} il3829_data_t;

/** Display configuration struct */
typedef struct {
    struct spi_dt_spec spi;    /*!< SPI device, display connected to */
    struct gpio_dt_spec dc;    /*!< Data/Command GPIO */
    struct gpio_dt_spec reset; /*!< Reset GPIO */
    struct gpio_dt_spec busy;  /*!< Busy GPIO */

    uint32_t width;  /*!< Display width */
    uint32_t height; /*!< Display height */
} il3829_config_t;

/* Private function prototypes */
/* Low level driver functions */
static void IL3829_HW_Reset(const struct device * dev);
static bool IL3829_IsBusy(const struct device * dev);
static int IL3829_Transmit(const struct device * dev, uint8_t cmd, const void * tx_data, size_t tx_len);
static inline int IL3829_SendCommand(const struct device * dev, uint8_t command);
static int IL3829_SetWindows(const struct device * dev, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
static int IL3829_SetCursor(const struct device * dev, uint16_t x, uint16_t y);
static int IL3829_TurnDisplayOn(const struct device * dev);

/* High level API functions */
static int il3829_init(const struct device * dev);
static int il3829_display_blanking_on(const struct device * dev);
static int il3829_display_blanking_off(const struct device * dev);
static int il3829_write(const struct device * dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor * desc,
                        const void * buf);
static int il3829_read(const struct device * dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor * desc,
                       void * buf);
static void * il3829_get_framebuffer(const struct device * dev);
static int il3829_set_brightness(const struct device * dev, const uint8_t brightness);
static int il3829_set_contrast(const struct device * dev, const uint8_t contrast);
static void il3829_get_capabilities(const struct device * dev, struct display_capabilities * capabilities);
static int il3829_set_pixel_format(const struct device * dev, const enum display_pixel_format pixel_format);
static int il3829_set_orientation(const struct device * dev, const enum display_orientation orientation);

/* Private variables */
/** Display data handler */
static il3829_data_t il3829_data = {
    .newDataFlag = false,
    .pixelFormat = PIXEL_FORMAT_MONO10,
    .orientation = DISPLAY_ORIENTATION_NORMAL   //
};

/** Display configuration handler */
static const il3829_config_t il3829_config = {
    .spi    = SPI_DT_SPEC_INST_GET(0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),
    .dc     = GPIO_DT_SPEC_INST_GET(0, dc_gpios),
    .busy   = GPIO_DT_SPEC_INST_GET(0, busy_gpios),
    .reset  = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
    .width  = DT_INST_PROP(0, width),
    .height = DT_INST_PROP(0, height)   //
};

/** Waveform Look Up Table for full display update */
const uint8_t lut_full_update[] = {0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22, 0x66, 0x69, 0x69, 0x59, 0x58, 0x99, 0x99,
                                   0x88, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xB4, 0x13, 0x51, 0x35, 0x51, 0x51, 0x19, 0x01, 0x00};

/** Waveform Look Up Table for partial display update */
const uint8_t lut_partial_update[] = {0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/** Driver API definition */
static const struct display_driver_api il3829_driver_api = {
    .blanking_on      = il3829_display_blanking_on,
    .blanking_off     = il3829_display_blanking_off,
    .write            = il3829_write,
    .read             = il3829_read,
    .get_framebuffer  = il3829_get_framebuffer,
    .set_brightness   = il3829_set_brightness,
    .set_contrast     = il3829_set_contrast,
    .get_capabilities = il3829_get_capabilities,
    .set_pixel_format = il3829_set_pixel_format,
    .set_orientation  = il3829_set_orientation,
};

/* Static Low level API function definitions */
/**
 * @brief Hardware Reset the display
 *
 * @param[in] dev Pointer to the display device structure handler
 */
static void IL3829_HW_Reset(const struct device * dev) {
    const il3829_config_t * config = dev->config;

    gpio_pin_set_dt(&config->reset, 1);
    k_msleep(10);
    gpio_pin_set_dt(&config->reset, 0);
    k_msleep(10);
    gpio_pin_set_dt(&config->reset, 1);
    k_msleep(200);
}

/**
 * @brief Check if the display is busy
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return true Display is busy
 * @return false Display is not busy
 */
static bool IL3829_IsBusy(const struct device * dev) {
    const il3829_config_t * config = dev->config;

    return gpio_pin_get_dt(&config->busy);
}

/**
 * @brief Transmit data to the display
 *
 * @param[in] dev     Pointer to the display device structure handler
 * @param[in] cmd     Command to send
 * @param[in] tx_data Pointer to the data to send
 * @param[in] tx_len  Length of the data to send
 *
 * @return int 0 if successful, negative error code otherwise
 */
static int IL3829_Transmit(const struct device * dev, uint8_t cmd, const void * tx_data, size_t tx_len) {
    const il3829_config_t * config = dev->config;

    int r;
    struct spi_buf tx_buf;
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1U};

    /* send command */
    tx_buf.buf = &cmd;
    tx_buf.len = 1U;

    gpio_pin_set_dt(&config->dc, IL3829_CMD);
    r = spi_write_dt(&config->spi, &tx_bufs);
    if (r < 0) {
        return r;
    }

    /* send data (if any) */
    if (tx_data != NULL) {
        tx_buf.buf = (void *)tx_data;
        tx_buf.len = tx_len;

        gpio_pin_set_dt(&config->dc, IL3829_DATA);
        r = spi_write_dt(&config->spi, &tx_bufs);
        if (r < 0) {
            return r;
        }
    }

    return 0;
}

/**
 * @brief Send command to the display
 *
 * @param[in] dev     Pointer to the display device structure handler
 * @param[in] command Command to send
 *
 * @return int 0 if successful, negative error code otherwise
 */
static inline int IL3829_SendCommand(const struct device * dev, uint8_t command) {
    return IL3829_Transmit(dev, command, NULL, 0);
}

/**
 * @brief Set LUT for the display
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return Nothing
 */
static inline void IL3829_SetLookUpTableBw(const struct device * dev) {
    IL3829_Transmit(dev, IL3829_WRITE_LUT_REGISTER, lut_partial_update, 30);
}

static int IL3829_SetWindows(const struct device * dev, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end) {
    uint8_t buf[4];
    int ret = 0;

    buf[0] = (x_start >> 3) & 0xFF;
    buf[1] = (x_end >> 3) & 0xFF;

    if ((ret = IL3829_Transmit(dev, IL3829_SET_RAM_X_ADDRESS_START_END_POSITION, buf, 2)) < 0) {
        return ret;
    }

    buf[0] = y_start & 0xFF;
    buf[1] = (y_start >> 8) & 0xFF;
    buf[2] = y_end & 0xFF;
    buf[3] = (y_end >> 8) & 0xFF;

    if ((ret = IL3829_Transmit(dev, IL3829_SET_RAM_Y_ADDRESS_START_END_POSITION, buf, 4)) < 0) {
        return ret;
    }

    return 0;
}

static int IL3829_SetCursor(const struct device * dev, uint16_t x, uint16_t y) {
    uint8_t buf[2];
    int ret = 0;

    buf[0] = (x >> 3) & 0xFF;

    if ((ret = IL3829_Transmit(dev, IL3829_SET_RAM_X_ADDRESS_COUNTER, buf, 1)) < 0) {
        return ret;
    }

    buf[0] = y & 0xFF;
    buf[1] = (y >> 8) & 0xFF;

    if ((ret = IL3829_Transmit(dev, IL3829_SET_RAM_Y_ADDRESS_COUNTER, buf, 2)) < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Turn the display on
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return int 0 if successful, negative error code otherwise
 */
static int IL3829_TurnDisplayOn(const struct device * dev) {
    uint8_t data = 0xD7;

    IL3829_Transmit(dev, IL3829_DISPLAY_UPDATE_CONTROL_2, &data, 1);
    IL3829_SendCommand(dev, IL3829_MASTER_ACTIVATION);
    IL3829_SendCommand(dev, IL3829_TERMINATE_FRAME_READ_WRITE);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    return 0;
}

/** Clear and reset the display */
int IL3829_ClearDisplay(const struct device * dev) {
    const il3829_config_t * config = dev->config;
    const il3829_data_t * data     = dev->data;

    int ret = 0;

    uint16_t width  = (config->width % 8 == 0) ? (config->width / 8) : (config->width / 8 + 1);
    uint16_t height = config->height;

    if ((ret = IL3829_SetWindows(dev, 0, 0, config->width, config->height)) < 0) {
        return ret;
    }

    uint8_t white;

    if (data->pixelFormat == PIXEL_FORMAT_MONO10) {
        white = 0xFF;   // White is white =)
    } else {
        white = 0x00;   // White is black =(
    }

    struct spi_buf tx_buf;
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1U};

    tx_buf.buf = (void *)&white;
    tx_buf.len = 1;

    for (uint16_t i = 0; i < height; i++) {
        if ((ret = IL3829_SetCursor(dev, 0, i)) < 0) {
            return ret;
        }

        if ((ret = IL3829_SendCommand(dev, IL3829_WRITE_RAM)) < 0) {
            return ret;
        }

        if ((ret = gpio_pin_set_dt(&config->dc, IL3829_DATA)) < 0) {
            return ret;
        }

        for (uint16_t j = 0; j < width; j++) {
            if ((ret = spi_write_dt(&config->spi, &tx_bufs)) < 0) {
                return ret;
            }
        }
    }

    return IL3829_TurnDisplayOn(dev);
}

/**
 * @brief API function to initialize the display
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return int 0 if successful, negative error code otherwise
 */
static int il3829_init(const struct device * dev) {
    LOG_DBG("Initializing IL3829 display driver");

    const il3829_config_t * config = dev->config;

    uint8_t dataSend[4] = {0};

    if (!device_is_ready(config->spi.bus)) {
        LOG_ERR("SPI device is not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->dc.port)) {
        LOG_ERR("DC GPIO device is not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->reset.port)) {
        LOG_ERR("Reset GPIO device is not ready");
        return -ENODEV;
    }

    if (!device_is_ready(config->busy.port)) {
        LOG_ERR("Busy GPIO device is not ready");
        return -ENODEV;
    }

    (void)gpio_pin_configure_dt(&config->dc, GPIO_OUTPUT);
    (void)gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_HIGH);
    (void)gpio_pin_configure_dt(&config->busy, GPIO_INPUT);

    IL3829_HW_Reset(dev);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Driver Output Control */
    dataSend[0] = (config->height - 1) & 0xFF;
    dataSend[1] = ((config->height - 1) >> 8) & 0xFF;
    dataSend[2] = 0x00; /* GD = 0; SM = 0; TB = 0 */

    IL3829_Transmit(dev, IL3829_DRIVER_OUTPUT_CONTROL, dataSend, 3);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Booster Soft Start Control */
    dataSend[0] = IL3829_BSSC_DURATION_20ms | IL3829_BSSC_DRIVING_3x | IL3829_BSSC_GDR_6_58us;   // Soft start setting for Phase1
    dataSend[1] = IL3829_BSSC_DURATION_10ms | IL3829_BSSC_DRIVING_3x | IL3829_BSSC_GDR_3_34us;   // Soft start setting for Phase2
    dataSend[2] = IL3829_BSSC_DURATION_10ms | IL3829_BSSC_DRIVING_4x | IL3829_BSSC_GDR_1_54us;   // Soft start setting for Phase3

    IL3829_Transmit(dev, IL3829_BOOSTER_SOFT_START_CONTROL, dataSend, 3);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Set Dummy Line Period */
    dataSend[0] = 0x16;   // Default POR: 22 dummy lines per gate

    IL3829_Transmit(dev, IL3829_SET_DUMMY_LINE_PERIOD, dataSend, 1);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Set Gate Time */
    dataSend[0] = 0x08;   // Default POR: 50 Hz

    IL3829_Transmit(dev, IL3829_SET_GATE_TIME, dataSend, 1);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Write VCOM Register */
    dataSend[0] = 0xA8;   // @FIXME: Why? Magic number, can't find it in the datasheet

    IL3829_Transmit(dev, IL3829_WRITE_VCOM_REGISTER, dataSend, 1);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    /* Data Entry Mode Setting */
    dataSend[0] = IL3829_DATA_ENTRY_MODE_X_INC_Y_INC | IL3829_DATA_ENTRY_MODE_AM_X_DIR;

    IL3829_Transmit(dev, IL3829_DATA_ENTRY_MODE_SETTING, dataSend, 1);

    IL3829_SetLookUpTableBw(dev);

    while (IL3829_IsBusy(dev)) {
        k_sleep(K_MSEC(1));
    }

    LOG_DBG("IL3829 display driver init: OK");

    return 0;
}

/**
 * @brief API function to turn the display off (sleep mode)
 *
 * @param[in] dev  Pointer to the display device structure handler
 *
 * @return int 0 if successful, negative errno code otherwise
 */
static int il3829_display_blanking_on(const struct device * dev) {
    uint8_t data = IL3829_DEEP_SLEEP_MODE_ON;

    return IL3829_Transmit(dev, IL3829_DEEP_SLEEP_MODE, &data, 1);
}

/**
 * @brief API function to turn the display on or update
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return int 0 if successful, negative errno code otherwise
 */
static int il3829_display_blanking_off(const struct device * dev) {
    il3829_data_t * data = dev->data;

    // If newData flag is set, we need to update the display
    if (data->newDataFlag) {
        data->newDataFlag = false;
        return IL3829_TurnDisplayOn(dev);
    }

    return 0;
}

/**
 * @brief API function to write a buffer to the display
 *
 * @param[in] dev  Pointer to the display device structure handler
 * @param[in] x    X position of the upper left corner of the area to write to
 * @param[in] y    Y position of the upper left corner of the area to write to
 * @param[in] desc Pointer to the display buffer descriptor
 * @param[in] buf  Pointer to the buffer to write
 *
 * @return int 0 if successful, negative errno code otherwise
 */
static int il3829_write(const struct device * dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor * desc,
                        const void * buf) {
    size_t buf_len;
    int ret = 0;

    il3829_data_t * data = dev->data;

    if (desc->pitch < desc->width) {
        LOG_ERR("Pitch is smaller then width");
        return -1;
    }

    buf_len = MIN(desc->buf_size, desc->height * desc->width / 8);

    if (buf == NULL || buf_len == 0U) {
        LOG_ERR("Display buffer is not available");
        return -1;
    }

    if (desc->pitch > desc->width) {
        LOG_ERR("Unsupported mode");
        return -1;
    }

    if ((ret = IL3829_SetWindows(dev, x, y, x + desc->width, y + desc->height)) < 0) {
        return ret;
    }

    const uint8_t * d = (uint8_t *)buf;

    for (uint16_t i = 0; i < desc->height; i++) {
        const uint8_t * p = d + i * (desc->width / 8);

        if ((ret = IL3829_SetCursor(dev, x, y + i)) < 0) {
            return ret;
        }

        if ((ret = IL3829_Transmit(dev, IL3829_WRITE_RAM, p, (desc->width / 8))) < 0) {
            return ret;
        }
    }

    data->newDataFlag = true;

    return 0;
}

static int il3829_read(const struct device * dev, const uint16_t x, const uint16_t y, const struct display_buffer_descriptor * desc,
                       void * buf) {
    LOG_ERR("Reading from display not supported");
    return -ENOTSUP;
}

static void * il3829_get_framebuffer(const struct device * dev) {
    LOG_ERR("Direct framebuffer access not supported");
    return NULL;
}

static int il3829_set_brightness(const struct device * dev, const uint8_t brightness) {
    LOG_ERR("Set brightness not implemented");
    return -ENOTSUP;
}

static int il3829_set_contrast(const struct device * dev, const uint8_t contrast) {
    LOG_ERR("Set contrast not supported");
    return -ENOTSUP;
}

/**
 * @brief API function to get IL3829 display capabilities
 *
 * @param[in]  dev          Pointer to the device structure for the driver instance
 * @param[out] capabilities Pointer to the display capabilities structure
 *
 * @return Nothing
 */
static void il3829_get_capabilities(const struct device * dev, struct display_capabilities * capabilities) {
    LOG_DBG("Getting capabilities");

    il3829_data_t * data = dev->data;

    capabilities->x_resolution            = il3829_config.width;
    capabilities->y_resolution            = il3829_config.height;
    capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO01;
    capabilities->current_pixel_format    = data->pixelFormat;
    capabilities->screen_info             = SCREEN_INFO_MONO_MSB_FIRST;
    capabilities->current_orientation     = data->orientation;
}

/**
 * @brief API function to set the pixel format of the display
 *
 * @param[in] dev          Pointer to the device structure for the driver instance
 * @param[in] pixel_format Pixel format to be set (supported: PIXEL_FORMAT_MONO01 and PIXEL_FORMAT_MONO10)
 *
 * @return int 0 if successful, negative errno code otherwise
 */
static int il3829_set_pixel_format(const struct device * dev, const enum display_pixel_format pixel_format) {
    if ((pixel_format & (PIXEL_FORMAT_MONO01 | PIXEL_FORMAT_MONO10)) == 0) {
        LOG_ERR("Pixel format not supported");
        return -ENOTSUP;
    }

    il3829_data_t * data = dev->data;
    data->pixelFormat    = pixel_format;

    return 0;
}

/**
 * @brief API function to set the display orientation
 *
 * @param[in] dev         Pointer to the display device
 * @param[in] orientation Orientation to set from enum display_orientation
 *
 * @return int 0 if successful, negative error code otherwise
 */
static int il3829_set_orientation(const struct device * dev, const enum display_orientation orientation) {
    il3829_data_t * data = dev->data;
    data->orientation    = orientation;

    return 0;
}

DEVICE_DT_INST_DEFINE(0, il3829_init, NULL, &il3829_data, &il3829_config, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &il3829_driver_api);
