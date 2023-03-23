#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from radar_msgs.msg import RadarTrack, RadarTrackArray

class polygon_transformer():
    def __init__(self):
        self.sub = rospy.Subscriber("/as_tx/radar_tracks", RadarTrackArray, self.callback)
        self.pub = rospy.Publisher("/processed/polygon_stamped", PointCloud, queue_size=10)
        self.header = Header()
        self.header.frame_id = 'esr_1'

    def callback(self,msg):
        points = []
        out_msg = PointCloud()
        self.header.stamp = rospy.Time.now()
        out_msg.header = self.header

        for track in msg.tracks:
            polygon = track.track_shape 
            track_points = polygon.points
            for point in track_points:
                print('point: ' + str(point))
                points.append(point)
        
        out_msg.points = points    
        self.pub.publish(out_msg) 

def main():
    print('initializing node')
    rospy.init_node("polygon_transformer", anonymous=True)
    print('initializing detector object')
    transformer = polygon_transformer()
    print('success in initialization')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down polygon_transformer node")

if __name__ == "__main__":
    main()

