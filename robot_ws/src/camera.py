import cv2
from flask import Flask, Response

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # 0 for first USB camera

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            continue
        # encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        # yield frame in HTTP multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # run server on all interfaces
    app.run(host='0.0.0.0', port=5000, debug=False)

