import sensor, image, time
import network

# ----------------------
# CONFIG
# ----------------------
WIFI_SSID = 
WIFI_PASSWORD =
FRAME_INTERVAL = 5  # capture every N frames
JPEG_QUALITY = 30

# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120
sensor.skip_frames(time=2000)

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

print("Connecting to Wi-Fi...")
while not wlan.isconnected():
    time.sleep_ms(500)
print("Connected! IP:", wlan.ifconfig()[0])

# Setup HTTP server
import usocket as socket

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print("HTTP server listening on port 80")

frame_count = 0
latest_img = None

while True:
    # Capture frame every FRAME_INTERVAL
    frame_count += 1
    if frame_count % FRAME_INTERVAL == 0:
        latest_img = sensor.snapshot().compress(quality=JPEG_QUALITY)

    # Accept client connection
    cl, addr = s.accept()
    print('Client connected from', addr)
    request = cl.recv(1024)  # read request (ignored)
    
    if latest_img:
        cl.send(b"HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\n\r\n")
        cl.send(latest_img)
    cl.close()
