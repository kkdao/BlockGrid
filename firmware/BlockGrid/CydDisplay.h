// ---------------------------------------------------------------------------
// Display backend for the ESP32-2432S028R ("Cheap Yellow Display"), used
// when DISPLAY_CYD is 1 in BlockGrid.ino.
//
// It stands in for MatrixPanel_I2S_DMA: the sketch draws its 96x48 picture
// into this class exactly as it does for the HUB75 panel (it is an
// Adafruit GFX canvas), and flipDMABuffer() shows it on the 320x240 TFT,
// every pixel drawn as a 3x3 block, 288x144 in the middle of the screen.
//
// Like the panel library's double buffering, a flip only copies the
// finished frame (9 KB) and returns; a background task sends it to the
// screen over SPI. That keeps the flip quick wherever it is called
// from, including the splash, which draws from an esp_timer callback.
// Memory: three 96x48 frames, ~27 KB, against ~55 KB for the panel's
// two DMA buffers.
//
// Pins, from the vendor schematic (ESP32-2432S028 V0.1): TFT on HSPI with
// SCK 14, MOSI 13, MISO 12, CS 15, DC 2, reset tied to the board's reset;
// backlight on GPIO21 through a transistor (PWM dims it). The screen on the
// board this was tested with takes ILI9341 commands but is laid out
// landscape and uses RGB colour order (see the panel config below).
// ---------------------------------------------------------------------------
#pragma once

#include <Adafruit_GFX.h>
#include <LovyanGFX.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>

#define CYD_TFT_SCK   14
#define CYD_TFT_MOSI  13
#define CYD_TFT_MISO  12
#define CYD_TFT_CS    15
#define CYD_TFT_DC     2
#define CYD_TFT_BL    21

#define CYD_SCALE 3

// This screen washes out colours that aren't at full strength: the bitcoin
// logo's orange (189, 109, 16) showed yellow next to the simulator. Each
// channel goes through this curve on its way out; 1.8 was picked on the
// board from 1.0, 1.4, 1.8 and 2.2.
#define CYD_GAMMA 1.8f

// The logo's orange reads yellow on this screen next to the block
// height's amber (255, 165, 30), so it is drawn in that amber instead:
// its main body (brightest channel 189) and brighter pixels come out
// exactly that amber, and the softer edge pixels proportionally dimmer,
// so the outline stays smooth.
inline uint16_t cydAmberLogo(uint16_t px) {
    uint16_t r = (px >> 11) << 3, g = ((px >> 5) & 63) << 2, b = (px & 31) << 3;
    uint16_t m = max(r, max(g, b)) * 255 / 189;
    if (m > 255) m = 255;
    uint16_t ag = 165 * m / 255, ab = 30 * m / 255;
    return ((m & 0xF8) << 8) | ((ag & 0xFC) << 3) | (ab >> 3);
}

class CydTft : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 panel;
    lgfx::Bus_SPI bus;
    lgfx::Light_PWM light;

public:
    CydTft() {
        auto b = bus.config();
        b.spi_host = HSPI_HOST;
        b.spi_mode = 0;
        b.freq_write = 40000000;
        b.freq_read = 16000000;
        b.pin_sclk = CYD_TFT_SCK;
        b.pin_mosi = CYD_TFT_MOSI;
        b.pin_miso = CYD_TFT_MISO;
        b.pin_dc = CYD_TFT_DC;
        bus.config(b);
        panel.setBus(&bus);

        auto p = panel.config();
        p.pin_cs = CYD_TFT_CS;
        p.pin_rst = -1;
        p.pin_busy = -1;
        // This screen's controller is laid out landscape, 320 wide, unlike a
        // stock ILI9341 (240 wide); orientation 6 below was picked on the
        // board by showing all eight.
        p.panel_width = 320;
        p.panel_height = 240;
        p.memory_width = 320;
        p.memory_height = 240;
        p.readable = true;
        p.invert = false;
        p.rgb_order = true;   // RGB, not the usual ILI9341 BGR: else orange shows blue
        p.bus_shared = false;
        panel.config(p);

        auto l = light.config();
        l.pin_bl = CYD_TFT_BL;
        l.invert = false;
        l.freq = 12000;
        l.pwm_channel = 7;
        light.config(l);
        panel.setLight(&light);

        setPanel(&panel);
    }
};

class CydDisplay : public GFXcanvas16 {
public:
    CydDisplay(uint16_t w, uint16_t h) : GFXcanvas16(w, h) {}

    bool begin() {
        frameBytes = (size_t)width() * height() * sizeof(uint16_t);
        front = (uint16_t *)malloc(frameBytes);
        shown = (uint16_t *)malloc(frameBytes);
        lock = xSemaphoreCreateMutex();
        if (!getBuffer() || !front || !shown || !lock) return false;
        memset(front, 0, frameBytes);
        for (int i = 0; i < 32; i++) lut5[i] = (uint8_t)lroundf(31.0f * powf(i / 31.0f, CYD_GAMMA));
        for (int i = 0; i < 64; i++) lut6[i] = (uint8_t)lroundf(63.0f * powf(i / 63.0f, CYD_GAMMA));

        tft.init();
        tft.setRotation(6);
        tft.fillScreen(0);    // the border around the picture stays black
        originX = (tft.width() - width() * CYD_SCALE) / 2;
        originY = (tft.height() - height() * CYD_SCALE) / 2;

        return xTaskCreatePinnedToCore(pushTask, "cydPush", 4096, this, 1, &task, 0) == pdPASS;
    }

    uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

    void clearScreen() { fillScreen(0); }

    // Hands the finished frame to the push task; the canvas is free to draw
    // the next one as soon as this returns.
    void flipDMABuffer() {
        if (!task) return;
        xSemaphoreTake(lock, portMAX_DELAY);
        memcpy(front, getBuffer(), frameBytes);
        xSemaphoreGive(lock);
        xTaskNotifyGive(task);
    }

    // 0 turns the backlight off; 1-255 is its PWM duty.
    void setBrightness8(uint8_t b) { tft.setBrightness(b); }

private:
    CydTft tft;
    uint16_t *front = nullptr;   // the latest flipped frame
    uint16_t *shown = nullptr;   // the push task's own copy, so a flip never waits on SPI
    size_t frameBytes = 0;
    SemaphoreHandle_t lock = nullptr;
    TaskHandle_t task = nullptr;
    int32_t originX = 0, originY = 0;
    uint8_t lut5[32], lut6[64];  // CYD_GAMMA for 5-bit red/blue and 6-bit green

    uint16_t applyGamma(uint16_t c) const {
        return (lut5[c >> 11] << 11) | (lut6[(c >> 5) & 63] << 5) | lut5[c & 31];
    }

    static void pushTask(void *arg) {
        CydDisplay *d = static_cast<CydDisplay *>(arg);
        for (;;) {
            // Flips that land while a frame is being sent collapse into one:
            // only the newest frame is sent next.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            xSemaphoreTake(d->lock, portMAX_DELAY);
            memcpy(d->shown, d->front, d->frameBytes);
            xSemaphoreGive(d->lock);
            d->push();
        }
    }

    // Each row goes out as runs of same-coloured pixels, every run one
    // filled rectangle 3 pixels tall; most of the picture is black, so a
    // frame is a few hundred rectangles.
    void push() {
        const int w = width(), h = height();
        tft.startWrite();
        for (int y = 0; y < h; y++) {
            const uint16_t *src = shown + y * w;
            int x = 0;
            while (x < w) {
                const uint16_t c = src[x];
                int run = 1;
                while (x + run < w && src[x + run] == c) run++;
                tft.fillRect(originX + x * CYD_SCALE, originY + y * CYD_SCALE,
                             run * CYD_SCALE, CYD_SCALE, applyGamma(c));
                x += run;
            }
        }
        tft.endWrite();
    }
};
