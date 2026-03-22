import cv2
import numpy as np
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# CONFIGURATION
# We generate a fake image instead of using a camera
width = 320
height = 240

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            print("Client connected. Streaming fake images...")
            
            try:
                counter = 0
                while True:
                    # 1. Create a blank image (Blue background)
                    frame = np.zeros((height, width, 3), np.uint8)
                    
                    # 2. Add some changing color (Simulate movement)
                    # Cycle color Red channel 0-255
                    color_val = int((np.sin(counter * 0.05) + 1) * 127)
                    frame[:] = (255, color_val, 0) # BGR: Blue, Green, Red
                    
                    # 3. Add Text overlay
                    text = f"Fake ESP32-CAM {counter}"
                    cv2.putText(frame, text, (20, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # 4. Encode as JPEG
                    ret, jpg = cv2.imencode('.jpg', frame)
                    frame_bytes = jpg.tobytes()
                    
                    # 5. Send MJPEG frame
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    
                    counter += 1
                    time.sleep(0.1) # 10 FPS
                    
            except Exception as e:
                print(f"Connection closed: {e}")
        else:
            self.send_error(404)

if __name__ == '__main__':
    # 0.0.0.0 makes it accessible from localhost
    server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
    print("Fake ESP32-CAM running on http://localhost:8080/stream")
    print("Open this URL in your browser to verify, or launch the ROS node.")
    server.serve_forever()