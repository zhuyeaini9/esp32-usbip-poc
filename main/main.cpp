#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/types.h>
#include <esp_vfs_fat.h>
#include "nvs_flash.h"
#include "usbip.hpp"
#include "driver/gpio.h"

#define DEV_VBUS_EN GPIO_NUM_12   // DEV_VBUS_EN: High level to enable DEV_VBUS power supply. (power the board from the device port by connecting to a computer)
#define BOOST_EN GPIO_NUM_13      // BOOST_EN: High level to enable Boost boost circuit. Low to enable USB_DEV as the power source
#define IDEV_LIMIT_EN GPIO_NUM_17 // LIMIT_EN: Enable current limiting IC, high level enable.
#define USB_SEL GPIO_NUM_18       // USB_SEL: Used to switch the USB interface. When high level, the USB_HOST interface is enabled. When low level, the USB_DEV interface is enabled.

#define GPIO_OUTPUT_PIN_SEL ((1ULL << DEV_VBUS_EN) | (1ULL << BOOST_EN) | (1ULL << IDEV_LIMIT_EN) | (1ULL << USB_SEL))

extern "C" void start_server();
USBhost *host;
static USBipDevice *device;
static bool is_ready = false;
static USBIP usbip;

void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    ESP_LOGW("", "usb_host_client_event_msg_t event: %d", event_msg->event);
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    {
        host->open(event_msg);

        info = host->getDeviceInfo();
        ESP_LOGI("USB_HOST_CLIENT_EVENT_NEW_DEV", "device speed: %s, device address: %d, max ep_ctrl size: %d, config: %d", info.speed ? "USB_SPEED_FULL" : "USB_SPEED_LOW", info.dev_addr, info.bMaxPacketSize0, info.bConfigurationValue);
        dev_desc = host->getDeviceDescriptor();

        device = new USBipDevice();
        device->init(host);

        is_ready = true;
    }
    else
    {
        // TODO: release all interfaces claimed in device.init
        is_ready = false;
        device->deinit();
        delete (device);
    }
}

void init_usbip()
{
    host = new USBhost();
    host->registerClientCb(client_event_callback);
    host->init();
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;        // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;              // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;   // bit mask of the pins to set,e.g.GPIO12/18
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // disable pull-up mode

    gpio_config(&io_conf);          // configure GPIO with the given settings
    gpio_set_level(DEV_VBUS_EN, 1); // Purpose: Powers USB_HOST (USB-A socket/receptacle) with power from the USB_DEV port (USB-A plug) as opposed to the battery [Therefore we should set the battery switch to OFF when using this mode]
    gpio_set_level(BOOST_EN, 0);
    gpio_set_level(IDEV_LIMIT_EN, 1);
    gpio_set_level(USB_SEL, 1);

    init_usbip();

    start_server();
}
