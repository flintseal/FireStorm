import cv2
import urllib.request
import numpy as np
from pupil_apriltags import Detector  # TODO: Replace with Aruco Marker Library
import threading
from networktables import NetworkTables
import time

cond = threading.Condition()
notified = [False]


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()


NetworkTables.initialize(server='10.87.78.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
table = NetworkTables.getTable("RobotData")

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()


stream = urllib.request.urlopen('http://roborio-8778-frc.local:1181/stream.mjpg')
bytes = b''

at_detector = Detector(
   families="tag16h5",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

while True:
    bytes += stream.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        npyarray = np.fromstring(jpg, dtype=np.uint8)
        i = cv2.cvtColor(cv2.imdecode(npyarray, cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
        detectorReadouts = at_detector.detect(i)
        intList = []
        for object in detectorReadouts:
            parsedList = str(object.center).replace("[", "").replace("]", "").replace("[ ", "").replace(" ]", "").split(" ")
            for item in parsedList:
                if item == "":
                    parsedList.remove(item)
                else:
                    intList.append(float(item))
            print(intList)
            if len(intList) == 2:
                table.putString("AprilTagLocationX", int(intList[0]))
                table.putString("AprilTagLocationY", int(intList[1]))
                table.putString("AprilTagLocationTimestamp", time.time())
                # i = cv2.drawMarker(i, (int(intList[0]), int(intList[1])), (1, 1, 1))
        cv2.imshow('i', i)
        if cv2.waitKey(1) == 27:
            exit(0)
