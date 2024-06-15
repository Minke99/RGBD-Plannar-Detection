# Libs.
from Lib.PlannarDetect import FeatureOdom
from Lib.UdpReceiver import UdpRigidBodies
from Lib.UdpReceiver import DataProcessor
from Lib import savemat
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import numpy as np
import cv2
import time

import logging
import cflib.crtp  # noqa

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Commander
import os



# CF logging
roll_CF = 0 
pitch_CF = 0
yaw_CF = 0
wx_CF = 0
wy_CF = 0
accx_CF = 0
accy_CF = 0
accz_CF = 0

# Recording Datas
# ============= Logging ================
class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        # Inertia Info Logging
        self._lg_stab = LogConfig(name='InerialInfo', period_in_ms=10)
        self._lg_stab.add_variable('stateEstimate.roll', 'float')
        self._lg_stab.add_variable('stateEstimate.pitch', 'float')
        # self._lg_stab.add_variable('stateEstimate.yaw', 'float')
        self._lg_stab.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        self._lg_stab.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        self._lg_stab.add_variable('acc.y', 'float')
        self._lg_stab.add_variable('acc.z', 'float')
        self._lg_stab.add_variable('acc.x', 'float')
   
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)

            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
    
            # Start the logging
            self._lg_stab.start()
      
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # t = Timer(5, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        global roll_CF,pitch_CF,yaw_CF,wx_CF,wy_CF,accx_CF,accy_CF,accz_CF
        roll_CF = data['stateEstimate.roll']
        pitch_CF = data['stateEstimate.pitch']
        yaw_CF = 0
        wx_CF = float(data['stateEstimateZ.rateRoll'])/1000
        wy_CF = float(data['stateEstimateZ.ratePitch'])/1000
        accy_CF = float(data['acc.y'])
        accz_CF = float(data['acc.z'])
        accx_CF = float(data['acc.x'])



    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])

def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

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

fps_r = 30
RTS = RealTimeSleeper(1/fps_r)

# CF
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
URI = 'radio://0/13/2M'


# control flags
is_frames = True # recording frames

is_odemetry = True

is_CF = False

# installation calibration   from imu_cam to mocap_B
# r_C2B = R.from_matrix([[1, 0, 0],
#                    [0, 1, 0],
#                    [0, 0, 1]])

r_C2B = R.from_matrix([[0, 0, 1],
                   [-1, 0, 0],
                   [0, -1, 0]])

# Recording
if is_frames:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    time_temp = time.strftime('%Y%m%d_%H%M%S', time.localtime(time.time()))
    output_file_rgb = 'Sets/' + time_temp + '_rbg.mp4' 
    output_file_depth = 'Sets/' + time_temp + '_depth.mp4' 
    fps_r = 30
    RTS = RealTimeSleeper(1/fps_r)
    frame_size_r = (640,480)
    video_writer_rgb = cv2.VideoWriter(output_file_rgb, fourcc, fps_r, frame_size_r)
    video_writer_depth = cv2.VideoWriter(output_file_depth, fourcc, fps_r, frame_size_r)



if __name__ == "__main__":

    # UDP
    if True:
        UDP = UdpRigidBodies()
        UDP.start_thread()
        DP = DataProcessor(UDP.num_bodies, UDP.get_sample_rate())
        rigid_body_index = 1

    # saver
    saver = savemat.DataSaver(
                'Abs_time',
                'x','y','z',
                'qx','qy','qz','qw',
                'cam_px','cam_py','cam_pz',
                'cam_inc_x','cam_inc_y','cam_inc_z',
                'accx_Cam','accy_Cam','accz_Cam',
                'gyrox_Cam','gyroy_Cam','gyroz_Cam',
                'fused_x', 'fused_y', 'fused_z', 'fused_vx', 'fused_vy', 'fused_vz',
                'acc_Wx', 'acc_Wy', 'acc_Wz',
                )

    # Cam setup
    if True:
        
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # gyroscope
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
        
        FO = FeatureOdom(depth_intrin)

        t_start = time.time()

    # Fusion = ComplementaryFilter()
    # Fusion = KalmanFilter()

    
    #-------------------------------------------------------------------
    # connect to crazyflie
    print('===========================')
    print('connecting crazyflie.......')
    print('===========================\n')

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    # print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = LoggingExample(URI)
        print('===========================')
        print('crazyflie connected........')
        print('===========================\n')
        is_CF = True
    else:
        print('No Crazyflies found, cannot run example')
    #-------------------------------------------------------------------
    
    if is_CF:
        saver = savemat.DataSaver(
            'Abs_time',
            'x','y','z',
            'qx','qy','qz','qw',
            'cam_px','cam_py','cam_pz',
            'cam_inc_x','cam_inc_y','cam_inc_z',
            'roll_CF','pitch_CF',
            'wx_CF','wy_CF',
            'accx_CF','accy_CF','accz_CF',
            'accx_Cam','accy_Cam','accz_Cam',
            'gyrox_Cam','gyroy_Cam','gyroz_Cam',
            'fused_x', 'fused_y', 'fused_z', 'fused_vx', 'fused_vy', 'fused_vz',
            'acc_Wx', 'acc_Wy', 'acc_Wz',
            )
    last_time = time.time()
    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        accel_frame = frames.first(rs.stream.accel).as_motion_frame().get_motion_data()
        acc = [accel_frame.x,accel_frame.y,accel_frame.z]

        gyro_frame =  frames.first(rs.stream.gyro).as_motion_frame().get_motion_data()
        gyro = [gyro_frame.x,gyro_frame.y,gyro_frame.z]


        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                        alpha = 0.5), cv2.COLORMAP_JET)
        
        if aligned_depth_frame is not None and aligned_color_frame is not None:
            abs_time = time.time() - t_start
            data_raw = UDP.get_data()
            data, save_list_data = DP.process_data(data_raw)
            
            rotm_B2W = R.from_quat([data[rigid_body_index]['qx'], data[rigid_body_index]['qy'],
                                            data[rigid_body_index]['qz'], data[rigid_body_index]['qw'], ])
            
            rotm_C2W = np.dot(rotm_B2W.as_matrix(),r_C2B.as_matrix())

            position, increment, img_show = FO.run(aligned_color_frame, aligned_depth_frame, rotm_C2W)
            acc_W = np.dot(rotm_C2W, acc)
            est = Fusion.fuse(position, increment, acc_W, dt = (time.time()-last_time))

            
            last_time = time.time()
            cv2.imshow('rgb', img_show)
            if cv2.waitKey(1) == ord('q'):
                break
            if is_CF:
                saver.add_elements(abs_time,
                                data[rigid_body_index]['x'],data[rigid_body_index]['y'],data[rigid_body_index]['z'],
                                data[rigid_body_index]['qx'],data[rigid_body_index]['qy'],data[rigid_body_index]['qz'],data[rigid_body_index]['qw'],
                                position[0],position[1],position[2], 
                                increment[0],increment[1],increment[2],
                                roll_CF,pitch_CF,
                                wx_CF,wy_CF,
                                accx_CF,accy_CF,accz_CF,
                                acc[0],acc[1],acc[2],
                                gyro[0],gyro[1],gyro[2],
                                est[0], est[1], est[2], est[3], est[4], est[5], 
                                acc_W[0], acc_W[1], acc_W[2],)

            else:
                saver.add_elements(abs_time,
                                data[rigid_body_index]['x'],data[rigid_body_index]['y'],data[rigid_body_index]['z'],
                                data[rigid_body_index]['qx'],data[rigid_body_index]['qy'],data[rigid_body_index]['qz'],data[rigid_body_index]['qw'],
                                position[0],position[1],position[2], increment[0],increment[1],increment[2],
                                acc[0],acc[1],acc[2],
                                gyro[0],gyro[1],gyro[2],
                                est[0], est[1], est[2], est[3], est[4], est[5], 
                                acc_W[0], acc_W[1], acc_W[2],)
            
            if is_frames:
                # Recording
                video_writer_rgb.write(color_image)
                depth_image_3c = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
                video_writer_depth.write(depth_image_3c)
                
        # time.sleep(0.1)
        RTS.sleep()

    saver.save2mat_recording('Sets/',time_temp)
    video_writer_rgb.release()
    video_writer_depth.release()
    if is_CF:
        le._cf.close_link()
    cv2.destroyAllWindows()
    UDP.stop_thread()