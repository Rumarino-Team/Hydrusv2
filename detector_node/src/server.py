#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse
import cv2
from starlette.middleware.cors import CORSMiddleware

app = FastAPI()
bridge = CvBridge()
annotated_image = None

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def annotated_image_callback(msg):
    global annotated_image
    try:
        annotated_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

@app.get("/")
def index():
    return {"message": "YOLO Video Streaming Server"}

def generate():
    global annotated_image
    while True:
        if annotated_image is not None:
            ret, jpeg = cv2.imencode('.jpg', annotated_image)
            if not ret:
                continue
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.get("/video_feed")
def video_feed():
    return StreamingResponse(generate(), media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node('video_stream_with_yolo_fastapi_node', anonymous=True)
    rospy.Subscriber('/annotated_image', Image, annotated_image_callback)
    import uvicorn
    uvicorn.run(app, host='0.0.0.0', port=8000)
