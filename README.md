 # e-paper: IL3829 kernel driver implementation for Zephir OS

 ## Installation

 1. Clone this repository
 2. Connect driver as Zephyr external module in CMakelists.txt, example:
    ```cmake
    set(CUSTOM_DRV_DIR $ENV{HOME}/workspace/zephyr)
    set(DRV_DISPLAY ${CUSTOM_DRV_DIR}/drivers/display/epaper_il3829)

    list(APPEND ZEPHYR_EXTRA_MODULES
    ${DRV_DISPLAY}
    )
    ```
3. Create or add to your device overlay, example:
    ```yaml
    / {
        chosen {
            zephyr,display = &e_display;
        };

        aliases {
            display0 = &e_display;
        };
    };

    &arduino_spi {
        e_display: il3829@0 {
            compatible = "ilitek,il3829";
            spi-max-frequency = <2000000>;
            status = "okay";
            reg = <0>;

            dc-gpios = <&arduino_header 10 GPIO_ACTIVE_HIGH>;     /* D4 */
            reset-gpios = <&arduino_header 14 GPIO_ACTIVE_HIGH>;  /* D8 */
            busy-gpios = <&arduino_header 13 GPIO_ACTIVE_HIGH>;   /* D7 */

            width = <200>;
            height = <200>;
        };
    };
    ```
4. Create DTS bindings file `dts/bindings/ilitek,il3829.yaml` in your project:
    ```yaml
    description: IL3829 e-paper display controller

    compatible: "ilitek,il3829"
    include: [spi-device.yaml]

    properties:
    dc-gpios:
        type: phandle-array
        required: true

    busy-gpios:
        type: phandle-array
        required: true

    reset-gpios:
        type: phandle-array
        required: true

    width:
        type: int
        required: true
        description: Width of the display in pixels

    height:
        type: int
        required: true
        description: Height of the display in pixels

    ```
5. Add parameters to project `prj.conf`:
    ```
    CONFIG_DISPLAY=y
    CONFIG_IL3829_DISPLAY=y
    ```
    For LVGL set also:
    ```
    CONFIG_LV_COLOR_DEPTH_32=n
    CONFIG_LV_COLOR_DEPTH_1=y
    ```
6. Compile, enjoy!

## Important note!
As e-paper is very slow, and LVGL fills the display partially, internally driver updates display only after `display_blanking_off()` execution. So, if you want to see something on display, you have to call `display_blanking_off()` after `lv_task_handler()`.

## Usage example
```c
#include <lvgl.h>
#include <zephyr/drivers/display.h>

void main(void) {
    const struct device * display_dev;

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display not ready, aborting test");
        return;
    }

    lv_obj_t * timeLabel;

    lv_task_handler();
    display_blanking_off(display_dev);

    timeLabel = lv_label_create(lv_scr_act());
    lv_label_set_text(timeLabel, "BNV");
    lv_obj_align(timeLabel, LV_ALIGN_CENTER, 0, 0);

    /* Infinite loop */
    for (;;) {
        lv_task_handler();
        display_blanking_off(display_dev);
        k_sleep(K_MSEC(10));
    }
}
```

## License

 This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

 * [Zephyr Project](https://www.zephyrproject.org/)
 * [LVGL](https://lvgl.io/)
 * [IL3829 datasheet](https://www.e-paper-display.com/IL3829d045.pdf)
