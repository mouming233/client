#!/usr/bin/env python
import rospy
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import socket
import threading
import struct

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('10.236.112.215', 6666))
m=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
m.connect(('10.236.112.215', 8888))


def map_callback(data):
    print(123)
    global subscribed
    subscribed=True
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution

    grid = list(data.data)
    grid = [grid[x:x+width] for x in range(0, len(grid), width)]
    

    img = np.zeros((height, width, 3), np.uint8)

    for i in range(height):
        for j in range(width):
            if grid[i][j] == 100:
                img[i,j] = (0, 0, 0)
            elif grid[i][j] == 0:
                img[i,j] = (255, 255, 255) 
    img_data = cv2.imencode('.jpg', img)[1].tobytes()
    s.sendall(len(img_data).to_bytes(4, byteorder='big'))
    s.sendall(img_data)
    cv2.imshow("Map", img)
    cv2.waitKey(1)

def send_thread():
    global response
    while True:
        response = m.recv(1024)
        print(response.decode())
        if response.decode()=='hello4':    
            client='message'
            flag=4
            m.send(struct.pack("!7sI",client.encode('utf-8'),int(flag)))


if __name__ == '__main__':
    send_thread = threading.Thread(target=send_thread)
    send_thread.start()
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        cv2.waitKey(1)
