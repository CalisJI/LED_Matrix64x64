// #include </ESP32-HUB75-MatrixPanel-DMA-master/src/ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <FastLED.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <spacecraft.h>
// const char* ssid = "VSMI-Guest";
// const char* password = "h3ll0vsmi";
WebSocketsServer webSocket = WebSocketsServer(81); // Sử dụng cổng 81
std::vector<uint8_t> frame_data;                   // Dùng để lưu trữ dữ liệu nhận được
const char *ssid = "VIETTEL_txXZQ27z";
const char *password = "CCF5A3K9";

// Set your Static IP address
// IPAddress local_IP(192, 168, 1, 243);
// IPAddress gateway(192, 168, 1, 1);
// IPAddress subnet(255, 255, 255, 0);

IPAddress local_IP(192, 168, 2, 243);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   //optional
// IPAddress secondaryDNS(8, 8, 4, 4); //optional
#define R1_PIN 4
#define G1_PIN 5
#define B1_PIN 6
#define R2_PIN 7
#define G2_PIN 15
#define B2_PIN 16
#define A_PIN 18
#define B_PIN 8 // Changed from library default
#define C_PIN 3
#define D_PIN 42
#define E_PIN 39 // required for 1/32 scan panels, like 64x64px. Any available pin would do, i.e. IO32
#define LAT_PIN 40
#define OE_PIN 2
#define CLK_PIN 41

// Configure for your panel(s) as appropriate!
#define PANEL_WIDTH 64
#define PANEL_HEIGHT 64
#define PANELS_NUMBER 4

#define PANE_WIDTH PANEL_WIDTH *PANELS_NUMBER
#define PANE_HEIGHT PANEL_HEIGHT

#define MAX_UDP_SIZE 1468
CRGB currentColor;
CRGBPalette16 palettes[] = {HeatColors_p, LavaColors_p, RainbowColors_p, RainbowStripeColors_p, CloudColors_p};
CRGBPalette16 currentPalette = palettes[0];
CRGB ColorFromCurrentPalette(uint8_t index = 0, uint8_t brightness = 255, TBlendType blendType = LINEARBLEND)
{
    return ColorFromPalette(currentPalette, index, brightness, blendType);
}
uint16_t time_counter = 0, cycles = 0, fps = 0;
unsigned long fps_timer;
WiFiUDP udp;
unsigned int localUdpPort = 5000;
const int bufferSize = PANE_WIDTH * PANE_HEIGHT * 2; // Adjust based on your data size
char incomingPacket[bufferSize];                     // Buffer for incoming packets

char *incomingPacket_psram = (char *)ps_malloc(bufferSize * sizeof(char));

#define NUM_PIXELS PANE_WIDTH *PANE_HEIGHT

// Biến trạng thái
int dataOffset = 0;
bool dataComplete = false;
MatrixPanel_I2S_DMA *dma_display = nullptr;
// Another way of creating config structure
// Custom pin mapping for all pins
HUB75_I2S_CFG::i2s_pins _pins = {R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
HUB75_I2S_CFG mxconfig(
    PANEL_WIDTH,           // width
    PANEL_HEIGHT,          // height
    PANELS_NUMBER,         // chain length
    _pins,                 // pin mapping
    HUB75_I2S_CFG::FM6126A // driver chip
);

// Pixel pixels[NUM_PIXELS];
uint8_t tempBuffer[NUM_PIXELS * 2];
bool recieve_done = false;
// Use PSRAM
// uint8_t *tempBuffer = (uint8_t *)ps_malloc(sizeof(uint8_t *));

int receivedBytes = 0; // Số byte đã nhận
bool newFrame = true;
uint32_t rgb565_to_rgb888(uint16_t rgb565)
{
    // Tách các kênh màu từ giá trị RGB565
    uint8_t r = (rgb565 >> 11) & 0x1F; // 5 bit cho đỏ
    uint8_t g = (rgb565 >> 5) & 0x3F;  // 6 bit cho xanh lục
    uint8_t b = rgb565 & 0x1F;         // 5 bit cho xanh dương

    // Chuyển đổi các kênh màu từ 5/6 bit sang 8 bit
    r = (r * 255 + 15) / 31; // Chuyển đổi từ 5 bit sang 8 bit
    g = (g * 255 + 31) / 63; // Chuyển đổi từ 6 bit sang 8 bit
    b = (b * 255 + 15) / 31; // Chuyển đổi từ 5 bit sang 8 bit

    // Ghép các giá trị RGB888 thành một giá trị 32 bit
    return (r << 16) | (g << 8) | b;
}
void receivePixels(const std::vector<uint8_t> &data)
{
    // Chuyển đổi dữ liệu từ RGB565
    for (int y = 0; y < PANE_HEIGHT; y++)
    {
        for (int x = 0; x < PANE_WIDTH; x++)
        {
            if (x <= 128)
            {
                int index = y * 128 + x;
                uint16_t rgb565 = (data[index * 2] << 8) | data[index * 2 + 1];
                dma_display->drawPixel(x, y, rgb565);
            }
            else
            {
                int x1 = x - 128;
                int index = ((128 * 64) + (y * 128 + x1));
                uint16_t rgb565 = (data[index * 2] << 8) | data[index * 2 + 1];
                dma_display->drawPixel(x, y, rgb565);
            }
        }
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_BIN:
        // Nhận dữ liệu và thêm vào frame_data
        frame_data.insert(frame_data.end(), payload, payload + length);
        // Kiểm tra nếu dữ liệu đã đầy đủ
        if (frame_data.size() == 16384 * 2)
        { // 16,384 bytes
            // Xử lý dữ liệu frame_data
            receivePixels(frame_data);
            // Serial.println(frame_data.size());
            //  Ví dụ: chuyển đổi và hiển thị lên ma trận LED
            frame_data.clear(); // Xóa dữ liệu sau khi xử lý
        }
        break;
    default:
        frame_data.clear();
        break;
    }
}

void receivePixels(char *packet, int length)
{
    //   if (newFrame) {
    //       newFrame = false;
    //   }

    if (receivedBytes + length > sizeof(tempBuffer))
    {
        length = sizeof(tempBuffer) - receivedBytes; // Giới hạn độ dài
    }

    memcpy(tempBuffer + receivedBytes, packet, length);
    receivedBytes += length;
    // Kiểm tra xem toàn bộ dữ liệu có được nhận đầy đủ chưa
    if (receivedBytes == NUM_PIXELS * 2)
    {
        // dma_display->clearScreen();
        for (int y = 0; y < PANE_HEIGHT; y++)
        {
            for (int x = 0; x < PANE_WIDTH; x++)
            {

                if (PANE_WIDTH <= 128)
                {
                    int index = (y * PANE_WIDTH + x);
                    uint16_t color = (tempBuffer[index * 2] << 8) | tempBuffer[index * 2 + 1];
                    uint32_t rgb888 = rgb565_to_rgb888(color);
                    uint8_t r = (rgb888 >> 16) & 0xFF; // Kênh đỏ
                    uint8_t g = (rgb888 >> 8) & 0xFF;  // Kênh xanh lục
                    uint8_t b = rgb888 & 0xFF;         // Kênh xanh dương
                    // pixels[index].color = color;
                    // dma_display->drawPixel(x, y, color);
                    //  if(y<=31)dma_display->drawPixelRGB888(x,y,r,g,b);
                    //  else  dma_display->drawPixelRGB888(x,y,r,b,g);
                    dma_display->drawPixelRGB888(x, y, r, g, b);
                }
                else
                {
                    if (x <= 128)
                    {
                        int index = (y * 128 + x);
                        uint16_t color = (tempBuffer[index * 2] << 8) | tempBuffer[index * 2 + 1];
                        uint32_t rgb888 = rgb565_to_rgb888(color);
                        uint8_t r = (rgb888 >> 16) & 0xFF; // Kênh đỏ
                        uint8_t g = (rgb888 >> 8) & 0xFF;  // Kênh xanh lục
                        uint8_t b = rgb888 & 0xFF;         // Kênh xanh dương
                        dma_display->drawPixelRGB888(x, y, r, g, b);
                    }
                    else
                    {

                        int x1 = x - 128;
                        int index = ((128 * 64) + (y * 128 + x1));
                        uint16_t color = (tempBuffer[index * 2] << 8) | tempBuffer[index * 2 + 1];
                        uint32_t rgb888 = rgb565_to_rgb888(color);
                        uint8_t r = (rgb888 >> 16) & 0xFF; // Kênh đỏ
                        uint8_t g = (rgb888 >> 8) & 0xFF;  // Kênh xanh lục
                        uint8_t b = rgb888 & 0xFF;         // Kênh xanh dương
                        dma_display->drawPixelRGB888(x, y, r, g, b);
                    }
                }
            }
        }
        recieve_done = false;
        // Đặt lại bộ nhớ tạm và biến nhận bytes
        memset(tempBuffer, 0, sizeof(tempBuffer));
        receivedBytes = 0;
        newFrame = true;
    }
}
void plasma_palette()
{

    for (int x = 0; x < PANE_WIDTH; x++)
    {
        if (x <= 128)
        {
            for (int y = 0; y < PANE_HEIGHT; y++)
            {
                int16_t v = 128;
                uint8_t wibble = sin8(time_counter);
                v += sin16(x * wibble * 3 + time_counter);
                v += cos16(y * (128 - wibble) + time_counter);
                v += sin16(y * x * cos8(-time_counter) / 8);

                currentColor = ColorFromPalette(currentPalette, (v >> 8)); //, brightness, currentBlendType);
                // if(y<=31)dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
                // else dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.b, currentColor.g);
                dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
            }
        }
        else if (x > 128 && x <= 256)
        {

            for (int y = 0; y < PANE_HEIGHT; y++)
            {
                int16_t v = 128;
                uint8_t wibble = sin8(time_counter);
                int x1 = x - 128;
                int y1 = y + 64;
                v += sin16(x1 * wibble * 3 + time_counter);
                v += cos16(y1 * (128 - wibble) + time_counter);
                v += sin16(y1 * x1 * cos8(-time_counter) / 8);

                currentColor = ColorFromPalette(currentPalette, (v >> 8)); //, brightness, currentBlendType);
                // if(y<=31)dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
                // else dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.b, currentColor.g);
                dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
            }
        }
        else
        {
            for (int y = 0; y < PANE_HEIGHT; y++)
            {
                int16_t v = 128;
                uint8_t wibble = sin8(time_counter);
                int x1 = x - 256;
                int y1 = y + 64;
                v += sin16(x1 * wibble * 3 + time_counter);
                v += cos16(y1 * (128 - wibble) + time_counter);
                v += sin16(y1 * x1 * cos8(-time_counter) / 8);

                currentColor = ColorFromPalette(currentPalette, (v >> 8)); //, brightness, currentBlendType);
                // if(y<=31)dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
                // else dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.b, currentColor.g);
                dma_display->drawPixelRGB888(x, y, currentColor.r, currentColor.g, currentColor.b);
            }
        }
    }

    ++time_counter;
    ++cycles;
    ++fps;

    if (cycles >= 1024)
    {
        time_counter = 0;
        cycles = 0;
        currentPalette = palettes[random(0, sizeof(palettes) / sizeof(palettes[0]))];
    }

    // print FPS rate every 5 seconds
    // Note: this is NOT a matrix refresh rate, it's the number of data frames being drawn to the DMA buffer per second
    if (fps_timer + 5000 < millis())
    {
        // Serial.printf_P(PSTR("Effect fps: %d\n"), fps/5);
        fps_timer = millis();
        fps = 0;
    }
}

void draw_spacecraft(int x, int y)
{
    for (int i = 0; i < 20; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            uint16_t color = aircraft[j * 20 + i];
            int _x = x + i;
            int _y = y + i;
            if (_x <= 128)
            {
                if (_y <= 64)
                {
                    dma_display->drawPixel(_x, _y, color);
                }
                else
                {
                    _y = _y - 64;
                    _x = _x + 128;

                    dma_display->drawPixel(_x, _y, color);
                }
            }
            else
            {
            }

            dma_display->drawPixel(_X, _y, color);
        }
    }
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    if (!WiFi.config(local_IP, gateway, subnet))
    {
        Serial.println("STA Failed to configure");
    }
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.print("Connected to WiFi.");
    Serial.println(WiFi.localIP());
    udp.begin(localUdpPort);

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin();            // setup the LED matrix
    dma_display->setBrightness8(50); // 0-255
    dma_display->clearScreen();
    // dma_display->fillScreenRGB888(0, 255, 0);
    currentPalette = RainbowColors_p;
}
void loop()
{
    webSocket.loop();
}
