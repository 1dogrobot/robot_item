#!/usr/bin/env python3     

from data_utils import *
from publish_utils import *
from cv_bridge import CvBridge
import os

DATA_PATH = "/home/yh/kitti/raw_data/2011_09_26/2011_09_26_drive_0005_sync"

if __name__ == '__main__':
    frame = 0
    # '''node init'''
    rospy.init_node('kitti_node', anonymous=True)
    
    # '''Publisher data'''
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)  
    '''launch car message'''
    ego_pub = rospy.Publisher('kitti_ego_car', MarkerArray, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)

    # model_pub =rospy.Publisher('kitti_car_model',Marker,queue_size=10)
    bridge = CvBridge()
    
    rate =rospy.Rate(10)
    df_tracking = read_tracking('/home/yh/kitti/training/label_02/0000.txt')
    # calib = Calibration('/home/yh/kitti/RawData/2011_09_26/',from_video=True)
    while not rospy.is_shutdown():
        df_tracking_frame = df_tracking[df_tracking.frame==frame]
        boxes_2d = np.array(df_tracking_frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types = np.array(df_tracking_frame['type'])
        track_ids = np.array(df_tracking_frame['track_id'])
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        publish_camera(cam_pub, bridge, image)
       
        ''' pointcloud data '''
        point_cloud = np.fromfile(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame), dtype=np.float32).reshape(-1,4)
        
        '''point cloud param'''
        publish_point_cloud(pcl_pub, point_cloud)
        publish_ego_car(ego_pub)
        imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
        publish_imu(imu_pub, imu_data)
        publish_gps(gps_pub, imu_data)
        # publish_car_model(model_pub)  
        rospy.loginfo("published")
        rate.sleep()     
        frame += 1
        frame %= 154