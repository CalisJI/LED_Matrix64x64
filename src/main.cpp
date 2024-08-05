//#include </ESP32-HUB75-MatrixPanel-DMA-master/src/ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
const char* ssid = "VSMI-Guest";
const char* password = "h3ll0vsmi";

#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 12
#define B2_PIN 13
#define A_PIN 23
#define B_PIN 22 // Changed from library default
#define C_PIN 5
#define D_PIN 17
#define E_PIN 32 // required for 1/32 scan panels, like 64x64px. Any available pin would do, i.e. IO32
#define LAT_PIN 4
#define OE_PIN 15
#define CLK_PIN 16


// Configure for your panel(s) as appropriate!
#define PANEL_WIDTH 64
#define PANEL_HEIGHT 64  
#define PANELS_NUMBER 1

#define PANE_WIDTH PANEL_WIDTH * PANELS_NUMBER
#define PANE_HEIGHT PANEL_HEIGHT

#define MAX_UDP_SIZE 1460
WiFiUDP udp;
unsigned int localUdpPort = 5000;
const int bufferSize = PANE_WIDTH*PANE_HEIGHT*2;  // Adjust based on your data size
char incomingPacket[bufferSize];  // Buffer for incoming packets
uint16_t pixelData[PANE_WIDTH*PANE_HEIGHT];  // Assuming 2 bytes per RGB565 pixel

// Cấu trúc Pixel
struct Pixel {
    uint16_t color;
};
#define PIXEL_ENTRY_SIZE 5
#define NUM_PIXELS PANE_WIDTH*PANE_HEIGHT
// Biến trạng thái
int dataOffset = 0;
bool dataComplete = false;
uint16_t arr[PANE_HEIGHT*PANE_WIDTH] PROGMEM;
MatrixPanel_I2S_DMA *dma_display = nullptr;
//Another way of creating config structure
//Custom pin mapping for all pins
HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
HUB75_I2S_CFG mxconfig(
            PANEL_WIDTH,   // width
            PANEL_HEIGHT,   // height
            PANELS_NUMBER,   // chain length
           _pins,   // pin mapping
  HUB75_I2S_CFG::FM6126A      // driver chip
);

Pixel pixels[PANE_WIDTH * PANE_HEIGHT];

void rgb565_to_rgb888(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = (color >> 8) & 0xF8;
  g = (color >> 3) & 0xFC;
  b = (color << 3) & 0xF8;
}

uint8_t tempBuffer[PANE_WIDTH * PANE_HEIGHT * 2];
int receivedBytes = 0;  // Số byte đã nhận
bool newFrame = true;

void resetPixels() {
    memset(pixels, 0, sizeof(pixels));
    memset(tempBuffer, 0, sizeof(tempBuffer));
    receivedBytes = 0;
    dataComplete = false;
}

void receivePixels(char* packet, int length) {
  if (newFrame) {
      newFrame = false;
      //dma_display->clearScreen();
  }

  if (receivedBytes + length > sizeof(tempBuffer)) {
        length = sizeof(tempBuffer) - receivedBytes;  // Giới hạn độ dài
  }
  memcpy(tempBuffer + receivedBytes, packet, length);
  receivedBytes += length;
  // Kiểm tra xem toàn bộ dữ liệu có được nhận đầy đủ chưa
    if (receivedBytes == PANE_WIDTH * PANE_HEIGHT * 2) {
        //dma_display->clearScreen();
        dataComplete = true;

        // Xử lý dữ liệu khi đã nhận đầy đủ
        int index = 0;
        for (int i = 0; i < receivedBytes; i += 2) {
            if (index < PANE_WIDTH * PANE_HEIGHT) {
                uint16_t color = (tempBuffer[i] << 8) | tempBuffer[i + 1];
                pixels[index].color = color;
                index++;
            }
        }

      

        // Đặt lại bộ nhớ tạm và biến nhận bytes
        memset(tempBuffer, 0, sizeof(tempBuffer));
        receivedBytes = 0;
        newFrame = true;
    }
}


void displayPixels() {
    for (int y = 0; y < PANE_HEIGHT; y++) {
        for (int x = 0; x < PANE_WIDTH; x++) {
            int index = (y * PANE_WIDTH + x);
            uint16_t color = pixels[index].color;
            if (color != 0) {
                dma_display->drawPixel(x, y, color);
            }
            else{
              dma_display->drawPixel(x, y, 0);
            }
            pixels[index].color = 0;
        }
    }
}
// void resetPixels() {
//     for (int i = 0; i < PANE_WIDTH*PANE_HEIGHT; i++) {
//         pixels[i].color = 0;  // Đặt màu về 0 hoặc màu nền khác
//     }
// }
void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.print("Connected to WiFi.");
    Serial.println(WiFi.localIP());
    udp.begin(localUdpPort);
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    dma_display->begin(); // setup the LED matrix
    dma_display->setBrightness8(15); //0-255
    dma_display->clearScreen();  
}

void loop() {
   int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(incomingPacket, MAX_UDP_SIZE);
        if (len > 0) {
            //Serial.print("Received packet of size ");
            //Serial.println(len);
            // appendData(incomingPacket, len);
            receivePixels(incomingPacket, len);
            // Nếu dữ liệu đầy đủ, hiển thị
            if (dataComplete) {
                displayPixels();
                //Serial.print("complete");
                dataComplete = false;  // Reset trạng thái
                
                // resetPixels();
                //memset(pixelData, 0, sizeof(pixelData));
            }
        }
    } else {
        //Serial.println("No packet received");
    }

    // delay(200);
}

