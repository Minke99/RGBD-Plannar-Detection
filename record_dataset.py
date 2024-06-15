# Libs.
from Lib.FeatureOdom import FeatureOdom
from Lib.UdpReceiver import UdpRigidBodies
from Lib.UdpReceiver import DataProcessor
from Lib import savemat
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class RealTimeSleeper:
    def __init__(self, sample_time):
        self._sample_time = sample_time
        self.loop_start_time = time.time()
        self.loop_flag = 0

    def init(self):
        self.loop_start_time = time.time()

    def sleep(self):
        self.loop_flag = self.loop_flag + 1
        current_time = time.time()

        loop_end_time = (self.loop_start_time + self._sample_time)
        sleep_time = loop_end_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print('warning: loop frequency lower than expected!')
        self.loop_start_time = time.time()


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

# UDP
if True:
    UDP = UdpRigidBodies()
    UDP.start_thread()
    DP = DataProcessor(UDP.num_bodies, UDP.get_sample_rate())
    rigid_body_index = 1

# Cam setup

if True:

    pipe = rs.pipeline()
    cfg  = rs.config()

    cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # gyroscope


    profile = pipe.start(cfg)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)


# Recording
if True:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    time_temp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
    output_file_rgb = time_temp + '_rbg.mp4'
    output_file_depth = time_temp + '_depth.mp4'
    fps_r = 30
    RTS = RealTimeSleeper(1/fps_r)
    frame_size_r = (640,480)
    video_writer_rgb = cv2.VideoWriter(output_file_rgb, fourcc, fps_r, frame_size_r)
    video_writer_depth = cv2.VideoWriter(output_file_depth, fourcc, fps_r, frame_size_r)

# saver
saver = savemat.DataSaver(
            'Abs_time',
            'x','y','z',
            'qx','qy','qz','qw',
            'accx','accy','accz',
            'gyrox','gyroy','gyroz',
            )
# Track
if True:
    FO = FeatureOdom()



t_start = time.time()
while True:

    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    # accel = accel_data(frame[0].as_motion_frame().get_motion_data())
    # gyro = gyro_data(frame[1].as_motion_frame().get_motion_data())

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                     alpha = 0.5), cv2.COLORMAP_JET)
    
    if depth_frame is not None and color_image is not None:
        abs_time = time.time() - t_start
        data_raw = UDP.get_data()
        data, save_list_data = DP.process_data(data_raw)
        

        accel_frame = frame.first(rs.stream.accel).as_motion_frame().get_motion_data()
        acc = [accel_frame.x,accel_frame.y,accel_frame.z]

        gyro_frame =  frame.first(rs.stream.gyro).as_motion_frame().get_motion_data()
        gyro = [gyro_frame.x,gyro_frame.y,gyro_frame.z]

        print('acc:',acc)
        print('gyro:',gyro)



        rotm = R.from_quat([data[rigid_body_index]['qx'], data[rigid_body_index]['qy'],
                                          data[rigid_body_index]['qz'], data[rigid_body_index]['qw'], ])
    
        if cv2.waitKey(1) == ord('q'):
            break
        saver.add_elements(abs_time,
                           data[rigid_body_index]['x'],data[rigid_body_index]['y'],data[rigid_body_index]['z'],
                           data[rigid_body_index]['qx'],data[rigid_body_index]['qy'],data[rigid_body_index]['qz'],data[rigid_body_index]['qw'],
                           acc[0],acc[1],acc[2],
                           gyro[0],gyro[1],gyro[2],
                           )
        
        cv2.imshow('rgb', color_image)

        # Recording
        video_writer_rgb.write(color_image)
        depth_image_3c = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        video_writer_depth.write(depth_image_3c)
    RTS.sleep()
    # time.sleep(0.1)
video_writer_rgb.release()
video_writer_depth.release()
saver.save2mat_recording('DataExchange/',time_temp)
cv2.destroyAllWindows()
UDP.stop_thread()