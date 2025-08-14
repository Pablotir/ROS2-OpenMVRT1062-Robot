import requests
import cv2
import numpy as np
import os

# ----------------------
# CONFIG
# ----------------------
OPENMV_IP = "192.168.1.123"  # replace with OpenMV IP from Wi-Fi
SAVE_FOLDER = "openmv_wifi_images"
DISPLAY_IMAGES = True

os.makedirs(SAVE_FOLDER, exist_ok=True)
frame_index = 0

try:
    while True:
        try:
            url = f"http://{OPENMV_IP}/"
            resp = requests.get(url, timeout=1)
            img_bytes = resp.content

            nparr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is not None:
                frame_index += 1

                # Save image
                filename = os.path.join(SAVE_FOLDER, f"frame_{frame_index:04d}.jpg")
                cv2.imwrite(filename, img)

                # Display
                if DISPLAY_IMAGES:
                    cv2.imshow("OpenMV Wi-Fi", img)
                    cv2.waitKey(1)

        except requests.exceptions.RequestException as e:
            print("Error fetching frame:", e)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    cv2.destroyAllWindows()
