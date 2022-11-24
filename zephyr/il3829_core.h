#ifndef IL3829_CORE_H
#define IL3829_CORE_H

#include <zephyr/device.h>

/** Command/data GPIO level for commands. */
#define IL3829_CMD 0U

/** Command/data GPIO level for data. */
#define IL3829_DATA 1U

#define IL3829_DRIVER_OUTPUT_CONTROL                0x01
#define IL3829_GATE_DRIVING_VOLTAGE_CONTROL         0x03
#define IL3829_BOOSTER_SOFT_START_CONTROL           0x0C
#define IL3829_GATE_SCAN_START_POSITION             0x0F
#define IL3829_DEEP_SLEEP_MODE                      0x10
#define IL3829_DATA_ENTRY_MODE_SETTING              0x11
#define IL3829_SW_RESET                             0x12
#define IL3829_TEMPERATURE_SENSOR_CONTROL           0x1A
#define IL3829_MASTER_ACTIVATION                    0x20
#define IL3829_DISPLAY_UPDATE_CONTROL_1             0x21
#define IL3829_DISPLAY_UPDATE_CONTROL_2             0x22
#define IL3829_WRITE_RAM                            0x24
#define IL3829_WRITE_VCOM_REGISTER                  0x2C
#define IL3829_WRITE_LUT_REGISTER                   0x32
#define IL3829_SET_DUMMY_LINE_PERIOD                0x3A
#define IL3829_SET_GATE_TIME                        0x3B
#define IL3829_BORDER_WAVEFORM_CONTROL              0x3C
#define IL3829_SET_RAM_X_ADDRESS_START_END_POSITION 0x44
#define IL3829_SET_RAM_Y_ADDRESS_START_END_POSITION 0x45
#define IL3829_SET_RAM_X_ADDRESS_COUNTER            0x4E
#define IL3829_SET_RAM_Y_ADDRESS_COUNTER            0x4F
#define IL3829_TERMINATE_FRAME_READ_WRITE           0xFF

/* Deep Sleep mode */
#define IL3829_DEEP_SLEEP_MODE_OFF 0x00 /*!< Normal Mode [POR] */
#define IL3829_DEEP_SLEEP_MODE_ON  0x01 /*!< Deep Sleep Mode */

/* Data Entry mode setting */
#define IL3829_DATA_ENTRY_MODE_X_DEC_Y_DEC 0x00 /*!< X decrement, Y decrement */
#define IL3829_DATA_ENTRY_MODE_X_INC_Y_DEC 0x01 /*!< X increment, Y decrement */
#define IL3829_DATA_ENTRY_MODE_X_DEC_Y_INC 0x02 /*!< X decrement, Y increment */
#define IL3829_DATA_ENTRY_MODE_X_INC_Y_INC 0x03 /*!< X increment, Y increment [POR] */

#define IL3829_DATA_ENTRY_MODE_AM_X_DIR 0x00 /*!< Address counter is updated in X direction [POR] */
#define IL3829_DATA_ENTRY_MODE_AM_Y_DIR 0x04 /*!< Address counter is updated in Y direction */

/* Booster Soft start Control bits */
#define IL3829_BSSC_DURATION_10ms (0x04 << 5) /*!< Duration of Phase 10ms */
#define IL3829_BSSC_DURATION_20ms (0x05 << 5) /*!< Duration of Phase 20ms */
#define IL3829_BSSC_DURATION_30ms (0x06 << 5) /*!< Duration of Phase 30ms */
#define IL3829_BSSC_DURATION_40ms (0x07 << 5) /*!< Duration of Phase 40ms */

#define IL3829_BSSC_DRIVING_1x 0x00        /*!< Driving Strength Selection: 1 */
#define IL3829_BSSC_DRIVING_2x (0x01 << 3) /*!< Driving Strength Selection: 2 */
#define IL3829_BSSC_DRIVING_3x (0x02 << 3) /*!< Driving Strength Selection: 3 */
#define IL3829_BSSC_DRIVING_4x (0x03 << 3) /*!< Driving Strength Selection: 4 */

#define IL3829_BSSC_GDR_0_27us 0x00 /*!< Min Off Time Setting of GDR: 0.27us */
#define IL3829_BSSC_GDR_0_34us 0x01 /*!< Min Off Time Setting of GDR: 0.34us */
#define IL3829_BSSC_GDR_0_4us  0x02 /*!< Min Off Time Setting of GDR: 0.4us */
#define IL3829_BSSC_GDR_0_54us 0x03 /*!< Min Off Time Setting of GDR: 0.54us */
#define IL3829_BSSC_GDR_0_8us  0x04 /*!< Min Off Time Setting of GDR: 0.8us */
#define IL3829_BSSC_GDR_1_54us 0x05 /*!< Min Off Time Setting of GDR: 1.54us */
#define IL3829_BSSC_GDR_3_34us 0x06 /*!< Min Off Time Setting of GDR: 3.34us */
#define IL3829_BSSC_GDR_6_58us 0x07 /*!< Min Off Time Setting of GDR: 6.58us */

/**
 * @brief Clear the display
 *
 * @param[in] dev Pointer to the display device structure handler
 *
 * @return int 0 if successful, negative error code otherwise
 */
int IL3829_ClearDisplay(const struct device * dev);

#endif /* IL3829_CORE_H */
