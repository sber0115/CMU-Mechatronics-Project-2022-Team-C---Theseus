#!/usr/bin/env python3

# node: video_stream_node
# topic: video_stream

import rospy
from sensor_msgs.msg import Image
from imutils.video import VideoStream
import imutils
import time
import cv2


def videoStreamTalker():

    print("***Starting video stream***")
    video = VideoStream(src=0).start() # change src parameter if there are multiple cameras
    time.sleep(1.0)
    print("***Press 'q' to end video stream***")
    
    # create a 'video_stream_node' node
    rospy.init_node('video_stream_node', anonymous=False)

    # ceate a "video_stream" topic 
    pub_video_stream = rospy.Publisher('/video_stream', Image, queue_size=1)

    # loop over frames from the video stream
    while True:
    	# grab one frame from the video stream
        frame = video.read()
        if frame is None:
            print("Failed to capture video stream!")
            print("Check 'VideoStream' camera source!")
            break
        # resize the frame with a maximum width of 1000 pixels
        frame = imutils.resize(frame, width=1000)
                
    	# publish the video_stream message (frame)
        pub_video_stream.publish(frame)
        cv2.imshow("Frame", frame)

        # if 'q' is pressed, end the loop
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    video.stop()


if __name__ == '__main__':
    try: 
        videoStreamTalker()
    except rospy.ROSInterruptException: 
        pass
