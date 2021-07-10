import sys
import cv2
import os

videodir = ".\\0\\"
videolist = []
framenum = sys.maxsize
for i in range(12):
    videoi = cv2.VideoCapture(videodir + "camera" + str(i+1) + ".mp4")
    framenum = min(framenum, int(videoi.get(7)))
    videolist.append(videoi)

datasetdir = ".\\dataset0\\"
for i in range(framenum):
    print(i)
    tempdir = datasetdir + "frame_" + str(i) + "\\"
    os.system("mkdir " + tempdir)
    os.system("mkdir " + tempdir + "images")
    os.system("mkdir " + tempdir + "depths")
    for j in range(12):
        success, frame = videolist[j].read()
        if success:
            cv2.imwrite(tempdir + "images\\" + str(j) + ".png", frame)

