# Project stream frame via websocket using ESP32 to LED Panel 128x128 with PlatformIO extension

### Configuration:
![](/img/wire.png)

**Connection Type:** `128x128` and `128x64`


### Pinout
![](/img/connector.jpg)


### Example code to send image/gif.video... to ESP32

```python
from PIL import Image, ImageSequence
import socket
import struct
import time
import cv2
import serial
from flask import Flask, request, redirect, make_response
import os
import asyncio
import websockets
import threading
### Flask configuration
app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
RESIZED_FOLDER = 'resized'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
os.makedirs(RESIZED_FOLDER, exist_ok=True)

### Socket configuration
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_ip = '192.168.2.243'  # IP của ESP32
udp_port = 5000 
uri = "ws://192.168.2.243:81"
media_path = '/home/qthi/Documents/quyth/Image/128x128/gif2.gif'

# To manage the WebSocket thread
websocket_thread = None
websocket_stop_event = threading.Event()

### Page
@app.route('/')
def upload_form():
    return '''
    <html><body>
    <h1>Upload Image</h1>
    <form action="/upload" method="post" enctype="multipart/form-data">
      <input type="file" name="file">
      <input type="submit" value="Upload">
    </form>
    </body></html>
    '''
frames = []
@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return redirect(request.url)
    
    file = request.files['file']
    if file.filename == '':
        return redirect(request.url)
    
    if file:
        filepath = os.path.join(UPLOAD_FOLDER, file.filename)
        file.save(filepath)
        frames = resize_gif_and_get_colors(filepath, (128, 128))
        threading.Thread(target=start_websocket_thread, args=(frames,)).start()
        # return send_file(resized_filepath, as_attachment=True)
        return make_response('OK',200)

def resize_gif_and_get_colors(input_path, new_size):
    """
    Thay đổi kích thước từng khung hình của ảnh GIF và trả về mảng màu RGB565 cho từng khung hình.
    
    :param input_path: Đường dẫn đến ảnh GIF gốc
    :param new_size: Kích thước mới (width, height)
    :return: Danh sách các mảng màu của từng khung hình
    """
    frames_colors = []
    
    with Image.open(input_path) as img:
        for frame in ImageSequence.Iterator(img):
            frame = frame.convert('RGB').resize(new_size, Image.ANTIALIAS)
            width, height = frame.size
            frame_data = bytearray()
            for y in range(height):
                for x in range(width):
                    r, g, b = frame.getpixel((x, y))
                    rgb565 = rgb888_to_rgb565(r, g, b)
                    frame_data.extend(struct.pack('!' + 'H', rgb565))

            frames_colors.append((frame_data,width,height))

    return frames_colors

def rgb888_to_rgb565(r, g, b):
    """Chuyển đổi từ RGB888 sang RGB565"""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
def load_image_to_rgb565(file_path):
    """Tải tệp ảnh và chuyển đổi từng pixel sang định dạng RGB565"""
    img = Image.open(file_path)
    img = img.convert('RGB')  # Đảm bảo ảnh ở định dạng RGB

    width, height = img.size
    frame_data = []

    for y in range(height):
        for x in range(width):
            r, g, b = img.getpixel((x, y))
            rgb565 = rgb888_to_rgb565(r, g, b)
            frame_data.append(rgb565)
    print(frame_data)
    return frame_data
def load_gif_to_rgb565(file_path):
    """Tải tệp GIF và chuyển đổi từng khung hình sang định dạng RGB565"""
    gif = Image.open(file_path)
    frames = []

    for frame in ImageSequence.Iterator(gif):
        frame = frame.convert('RGB')
        width, height = frame.size
        frame_data = []
        for y in range(height):
            for x in range(width):
                r, g, b = frame.getpixel((x, y))
                rgb565 = rgb888_to_rgb565(r, g, b)
                frame_data.append(rgb565)
        frames.append((frame_data, width, height))

    return frames
def load_video_to_rgb565(file_path):
    """Tải video và chuyển đổi từng khung hình sang định dạng RGB565"""
    cap = cv2.VideoCapture(file_path)
    frames = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = frame.shape[:2]
        frame_data = []
        for y in range(height):
            for x in range(width):
                r, g, b = frame[y, x]
                rgb565 = rgb888_to_rgb565(r, g, b)
                frame_data.append(rgb565)
        frames.append((frame_data, width, height))
    print(frames)
    cap.release()
    return frames
def send_frame(frame_data, udp_ip, udp_port):
    """Gửi khung hình qua UDP"""
    
    packet_size = 1460 # Kích thước gói tối đa để tránh tràn UDP
    num_packets = (len(frame_data) * 2 + packet_size - 1) // packet_size
    for i in range(num_packets):
        start_index = i * packet_size // 2
        end_index = min((i + 1) * packet_size // 2, len(frame_data))
        packet = struct.pack('!' + 'H' * (end_index - start_index), *frame_data[start_index:end_index])
        sock.sendto(packet, (udp_ip, udp_port))
        # time.sleep(0.1)

def send_data_via_serial(port, baudrate, data):
    with serial.Serial(port, baudrate, timeout=1) as ser:
        # Gửi mảng dữ liệu
        for value in data:
            # Gửi giá trị RGB565 (2 byte)
            ser.write(struct.pack('<H', value))

def start_websocket_thread(frames):
    global websocket_thread, websocket_stop_event

    if websocket_thread and websocket_thread.is_alive():
        websocket_stop_event.set()
        websocket_thread.join()
    
    websocket_stop_event.clear()
    websocket_thread = threading.Thread(target=lambda:asyncio.run(send_image_data(frames,websocket_stop_event)))
    websocket_thread.start()
    # asyncio.run(send_image_data(frames))

async def send_image_data(frames, stop_event):
      # Thay <ESP32_IP> bằng địa chỉ IP của ESP32
    # uri = "ws://localhost:8765"  # Thay <ESP32_IP> bằng địa chỉ IP của ESP32
    #frames = load_gif_to_rgb565(file_path)
    async with websockets.connect(uri) as websocket:
        while not stop_event.is_set():
            #frame_data = load_image_to_rgb565(file_path)
            # frame_data = create_test_pattern()
            for frame_data, _, _ in frames:
                # Chia nhỏ dữ liệu nếu cần (tùy thuộc vào kích thước của dữ liệu và giới hạn của WebSocket)
                chunk_size = 12288  # Kích thước mỗi phần dữ liệu
                for i in range(0, len(frame_data), chunk_size):
                    chunk = frame_data[i:i + chunk_size]
                    await websocket.send(chunk)
                await asyncio.sleep(1/10)  # Giữ tốc độ gửi dữ liệu

def main():
    app.run(debug=True)
if __name__ == "__main__":
    main()

```

### Select Image Page
![](/img/webpage.jpg)

### Hardware
- ESP32-S3 N8R2 Dual Type-C
![](/img/esp32s3.jpg)
- Pinout
![](/img/esp32-s3_devkitc-1_pinlayout_v1.1.jpg)
