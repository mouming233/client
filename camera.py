import cv2
import rospy
import socket
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped
import numpy as np
import struct
import threading


subscribed=False
bridge = CvBridge()



s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('10.236.112.215', 7777))
m=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
m.connect(('10.236.112.215', 8888))





def depth_image_callback(msg):
    
    global depth_image,subscribed
    subscribed=True
    print(subscribed)
    depth_image = bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
    cv2.waitKey(5)
    img_data = cv2.imencode('.jpg', depth_image)[1].tobytes()
    
    s.sendall(len(img_data).to_bytes(4, byteorder='big'))
    s.sendall(img_data)
    
def send_thread():
    global response
    while True:
        response = m.recv(1024)
        print(response.decode())
        if response.decode()=='hello1':    
            client='message'
            flag=1
            m.send(struct.pack("!7sI",client.encode('utf-8'),int(flag)))
        if subscribed==True:
            client='xiaoche1'
            flag=1
            m.send(struct.pack("!7sI",client.encode('utf-8'),int(flag)))

            
if __name__ == '__main__':
    send_thread = threading.Thread(target=send_thread)
    send_thread.start()
    rospy.init_node("target_site")


    rospy.Subscriber("/qingzhou/camera_link/image_raw",Image,depth_image_callback)

    
