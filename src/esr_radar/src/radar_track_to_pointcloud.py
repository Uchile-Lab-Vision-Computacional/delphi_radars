#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from radar_msgs.msg import RadarTrack, RadarTracks

class track_transformer():
    def __init__(self):
        self.sub = rospy.Subscriber("/esr_1/radar_tracks", RadarTracks, self.callback)
        self.pub = rospy.Publisher("/esr_1/radar_pointcloud", PointCloud, queue_size=10)
        self.header = Header()
        self.header.frame_id = 'esr_1'

    def callback(self,msg):
        points = []
        out_msg = PointCloud()
        self.header.stamp = rospy.Time.now()
        out_msg.header = self.header

        for track in msg.tracks:
            point = track.position
            #print(point)
            points.append(point)
        
        out_msg.points = points    
        self.pub.publish(out_msg) 

def main():
    print('initializing node')
    rospy.init_node("track_transformer", anonymous=True)
    print('initializing transformer object')
    transformer = track_transformer()
    print('success in initialization')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down polygon_transformer node")

if __name__ == "__main__":
    main()
