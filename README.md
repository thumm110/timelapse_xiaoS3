# Peach Tree Timelapse for XIAO ESP32S3 Sense

This project runs a daylight-only timelapse camera on a Seeed Studio XIAO ESP32S3 Sense. It captures JPEG photos to a microSD card, sleeps between captures to save power, and exposes a temporary Wi-Fi download interface when a button is pressed.

GitHub repo: `thumm110/timelapse_xiaoS3`

## Hardware

- Seeed Studio XIAO ESP32S3 Sense
- Camera module connected through the stock XIAO Sense camera connector
- microSD card
- Optional normally-open pushbutton from `A1/D1` to `GND` for download mode
- Optional 1-cell Li-ion/LiPo battery monitor on `A0` through a resistor divider

## What the sketch does

- Connects to Wi-Fi and syncs time with NTP
- Calculates sunrise and sunset from configured latitude and longitude
- Captures photos only during the daylight window, with an extra spillover margin
- Saves images to the SD card using filenames like `YYYYMMDD_HHMMSS.jpg`
- Uses deep sleep between wakeups
- Starts a temporary web server for browsing and downloading images when the download button is pressed

## Current default configuration

These values are hard-coded near the top of [`xiao_timelapse_peach_tree.ino`](/home/thumm/Arduino/xiao_timelapse_peach_tree/xiao_timelapse_peach_tree.ino):

- Wi-Fi SSID: `user_SSID`
- Wi-Fi password: `SSID_password`
- Time zone: `EST5EDT,M3.2.0/2,M11.1.0/2`
- Latitude: `33.01000`
- Longitude: `-80.00000`
- Capture interval: `60` minutes
- Daylight spillover: `30` minutes
- Download AP SSID: `gardencam`
- Download AP password: `gardentree`
- Download window: `30` minutes
- SD chip select pin: `21`
- Battery monitor: disabled by default

Update the Wi-Fi credentials and location before flashing.

## Wiring

### Download button

Wire a normally-open pushbutton between:

- `A1/D1`
- `GND`

Pressing the button wakes the board and starts download mode.

### Optional battery monitor

If you want battery-aware sleep behavior, wire battery positive through a resistor divider to `A0` so the ADC input stays at or below `3.3V`.

Example 1:1 divider:

- battery `+` -> `100k` -> `A0`
- `A0` -> `100k` -> `GND`
- battery `-` -> `GND`

Then enable battery monitoring in the sketch:

- Set `BATTERY_MONITOR_ENABLED` to `true`
- Leave `BATTERY_DIVIDER_RATIO` at `2.0` for the 100k/100k example

## Build and flash

Open the sketch in the Arduino IDE and use settings appropriate for the XIAO ESP32S3 Sense.

Recommended checklist:

- Install ESP32 board support
- Select `XIAO_ESP32S3`
- Make sure PSRAM is enabled if your environment exposes that option
- Install any required ESP32 camera support included with the board package
- Edit Wi-Fi credentials and location constants
- Flash the sketch

## Download mode

When the board enters download mode, it:

- starts a soft AP named `gardencam`
- uses password `gardentree`
- also attempts to join the configured Wi-Fi network
- serves an HTTP interface on port `80`

Available endpoints include:

- `/` browse the photo gallery
- `/list` list files as JSON
- `/latest` fetch the latest photo
- `/file?name=/filename.jpg` view a file
- `/download?name=/filename.jpg&download=1` download a file
- `/snapshot` capture a live JPEG snapshot
- `/stream` open a live MJPEG stream

There are also delete actions in the web UI for individual files and all files.

## Power behavior

- The board sleeps between capture windows
- If battery monitoring is enabled and voltage is low, it skips work and sleeps longer
- If Wi-Fi or time sync fails, it sleeps for 15 minutes before retrying

## Notes

- Photos are stored in the SD card root directory
- Image counter state is kept in RTC memory across deep sleep
- Capture resolution is `UXGA` when PSRAM is available, otherwise it falls back to `SVGA`
