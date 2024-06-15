# Libs.
from Lib.PlannarDetect import RGBDNormal
from Lib.UdpReceiver import UdpRigidBodies
from Lib.UdpReceiver import DataProcessor
from Lib import savemat
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import numpy as np
import cv2
import time



def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

if __name__ == "__main__":
    # UDP
    if False:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        DP = DataProcessor(UDP.num_bodies, UDP.get_sample_rate())
        rigid_body_index = 1

    # saver
    # saver = savemat.DataSaver(
    #             'Abs_time',
    #             'x','y','z',
    #             'qx','qy','qz','qw',
    #             'cam_px','cam_py','cam_pz',
    #             'cam_inc_x','cam_inc_y','cam_inc_z'
    #             )

    # Cam setup
    if True:

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)

        # Variables to store feature points and tracking status
        initial_features = None
        previous_gray = None
        mask = None

        # align
        if True:
            align_to = rs.stream.color
            align = rs.align(align_to)

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        # FO = FeatureOdom(depth_intrin)

        t_start = time.time()

    if True:
        fx, fy, cx, cy = 605.2021484375, 605.3436279296875, 323.37109375, 246.51541137695312
        normal_estimator = RGBDNormal(fx, fy, cx, cy)


    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        
        if aligned_depth_frame is not None and aligned_color_frame is not None:
            abs_time = time.time() - t_start
            # data_raw = UDP.get_data()
            # data, save_list_data = DP.process_data(data_raw)
            
            # rotm = R.from_quat([data[rigid_body_index]['qx'], data[rigid_body_index]['qy'],
            #                                 data[rigid_body_index]['qz'], data[rigid_body_index]['qw'], ])
            
            # cv2.imshow('rgb', color_image)
            # cv2.imshow('depth', depth_cm)
            # rotm = np.eye(3)
            # position, increment, img_show = FO.run(aligned_color_frame, aligned_depth_frame, rotm.as_matrix)
            # print(position)
            im = np.asanyarray(aligned_color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            result_image = normal_estimator.process_and_visualize(im, depth_image)

            cv2.imshow('rgb', result_image)
            if cv2.waitKey(1) == ord('q'):
                break
            # saver.add_elements(abs_time,
            #                 data[rigid_body_index]['x'],data[rigid_body_index]['y'],data[rigid_body_index]['z'],
            #                 data[rigid_body_index]['qx'],data[rigid_body_index]['qy'],data[rigid_body_index]['qz'],data[rigid_body_index]['qw'],
            #                 position[0],position[1],position[2], increment[0],increment[1],increment[2])
    # saver.save2mat('DataExchange/')
    cv2.destroyAllWindows()
    # UDP.stop_thread()