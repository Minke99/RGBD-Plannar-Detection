import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
from collections import deque

class FeatureOdom():
    def __init__(self, K):
        self.K = K
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.shi_tomasi_params = dict(maxCorners=100, qualityLevel=0.5, minDistance=7, blockSize=7)
        self.fast = cv2.FastFeatureDetector_create()
        self.fast.setThreshold(50)
        self.kp = []
        self.previous_gray = []
        self.tracks = deque(maxlen=50)  # Stores the points and their timestamps
        self.mask = []

        self.Q1 = []
        self.Q1_pose = [0, 0, 0]
        self.Q1_rotm = []

        self.last_position = [0,0,0]

    def feature_detection(self, gray_image):
        return cv2.goodFeaturesToTrack(gray_image, mask=None, **self.shi_tomasi_params)
    
    # ORB features
    # def feature_detection(self, gray_image):
    #     # Initialize ORB detector
    #     orb = cv2.ORB_create(fastThreshold=50)
    #     # Detect ORB keypoints
    #     keypoints = orb.detect(gray_image, None)
    #     # Convert keypoints to numpy array in the format (N, 1, 2)
    #     kp_array = np.array([[kp.pt] for kp in keypoints], dtype=np.float32)
    #     return kp_array

    def is_duplicate(self, pt1, pts, threshold=5):
        for pt2 in pts:
            if np.linalg.norm(pt1 - pt2) < threshold:
                return True
        return False
    
    # For off-line testing
    # def get_3d_camera_coordinate(self, depth_pixel, aligned_depth_frame, depth_intrin):
    #     f_x = self.K[0, 0]  # x focal length
    #     f_y = self.K[1, 1]  # y focal length
    #     c_x = self.K[0, 2]  # x misalignment of the principal point with the center of the image
    #     c_y = self.K[1, 2]  # y misalignment of the principal point with the center of the image
    #     x, y = depth_pixel
    #     Z = aligned_depth_frame[int(y), int(x)]  # dep1 is a 2D array
    #     X = (x - c_x) * Z / f_x
    #     Y = (y - c_y) * Z / f_y
    #     camera_coordinate = [X, Y, Z]
    #     return camera_coordinate
    
    def get_3d_camera_coordinate(self, depth_pixel,aligned_depth_frame,depth_intrin):
        x = depth_pixel[0]
        y = depth_pixel[1]

        dis = aligned_depth_frame.get_distance(x,y)

        camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin,depth_pixel,dis)

        return camera_coordinate
    
    def filter_points(self, Q1, kp, min_z=0.1, max_z=4):
        valid_indices = np.where((Q1[:, 0, 2] >= min_z) & (Q1[:, 0, 2] <= max_z))[0]
        Q1_filtered = Q1[valid_indices, :]
        kp_filtered = kp[valid_indices, :]
        
        return Q1_filtered, kp_filtered

    def run(self, aligned_color_frame, aligned_depth_frame, rotm):
        try:
            im = np.asanyarray(aligned_color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
        except:
            im = aligned_color_frame
            depth_image = aligned_depth_frame
        gray_image = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        current_time = time.time()

        # Detect features
        if len(self.kp) == 0:
            print("init poinst")
            new_kp = self.feature_detection(gray_image)
            if new_kp is not None:
                # Filter out duplicate points
                new_kp = [kp for kp in new_kp if not self.is_duplicate(kp, self.kp)]
                if new_kp:
                    new_kp = np.array(new_kp)
                    self.kp = np.concatenate((self.kp, new_kp), axis=0) if len(self.kp) > 0 else new_kp
                self.previous_gray = gray_image.copy()
                self.mask = np.zeros_like(im)

            self.Q1 = np.array([self.get_3d_camera_coordinate([pt[0][0], pt[0][1]], aligned_depth_frame, self.K) for pt in self.kp])

            self.Q1 = self.Q1.reshape((self.Q1.shape[0], 1, 3))
            # Filter close and far points
            Q1_filtered, kp_filtered = self.filter_points(self.Q1, self.kp)
            self.Q1 = Q1_filtered
            self.kp = kp_filtered
            # self.Q1_pose = position
            self.Q1_rotm = rotm

            if len(self.kp) == 0:
                return self.last_position, [0,0,0], im

        # Track self.kp features in the current given im
        if len(self.kp) > 0:
            # Calculate optical flow
            new_kp, status, error = cv2.calcOpticalFlowPyrLK(self.previous_gray, gray_image, self.kp, None, **self.lk_params)

            # Select good points and filter out untrackable points
            valid_ind = (new_kp[:, 0, 0] >= 0) & (new_kp[:, 0, 0] < gray_image.shape[1]) & (new_kp[:, 0, 1] >= 0) & (new_kp[:, 0, 1] < gray_image.shape[0])
            status = status.flatten()
            status = status & valid_ind
            good_new = new_kp[status == 1]
            good_old = self.kp[status == 1]

            # New 3D points
            Q2 = np.array([self.get_3d_camera_coordinate([pt[0][0], pt[0][1]], aligned_depth_frame, self.K) for pt in good_new])
            Q2 = Q2.reshape((Q2.shape[0], 1, 3))
            self.Q1 = self.Q1[status == 1]

            # Update tracks with new points for drawing
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                self.tracks.append((a, b, c, d, current_time))

            self.previous_gray = gray_image.copy()
            self.kp = good_new.reshape(-1, 1, 2)

            # If all points are filtered out
            if len(self.kp) == 0:
                return self.last_position, [0,0,0], im
            
            position, offset = self.compute_trans(self.Q1, Q2, rotm)

            # Check if the number of tracked points is less than the threshold
            if len(self.kp) < 30:
                print("refresh")
                new_kp = self.feature_detection(gray_image)
                if new_kp is not None:
                    # Filter out duplicate points
                    new_kp = [kp for kp in new_kp if not self.is_duplicate(kp, self.kp)]
                    if new_kp:
                        new_kp = np.array(new_kp)
                        self.kp = np.concatenate((self.kp, new_kp), axis=0)

                # Update Q1
                Q1 = np.array([self.get_3d_camera_coordinate([pt[0][0], pt[0][1]], aligned_depth_frame, self.K) for pt in self.kp])
                self.Q1 = Q1.reshape((Q1.shape[0], 1, 3))
                # Filter close and far points
                Q1_filtered, kp_filtered = self.filter_points(self.Q1, self.kp)
                self.Q1 = Q1_filtered
                self.kp = kp_filtered
                self.Q1_pose = position
                self.Q1_rotm = rotm

        # Draw the tracks
        if True:
            im = cv2.add(im, self.mask)
            self.mask = np.zeros_like(im)
            for a, b, c, d, t in self.tracks:
                if current_time - t < 3:  # Only draw lines within n secs
                    self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 255), 2)  # Yellow lines
                    im = cv2.circle(im, (int(a), int(b)), 5, (0, 255, 0), -1)  # Green points

        return position, offset, im
    
    def compute_trans(self, Q1, Q2, rotm):
        deltaR = self.Q1_rotm.T @ rotm
        # deltaR = np.eye(3)
        P1 = Q1.reshape(-1, 3)
        P2 = Q2.reshape(-1, 3)

        RP2 = deltaR.dot(P2.T)
        RP2 = RP2.T
        
        position = np.median(P1 - RP2, axis=0)
        position = np.dot(self.Q1_rotm, position)
        # if np.linalg.norm(position) < 0.3:
        position += self.Q1_pose
        
        increment = position - self.last_position
        self.last_position = position
        return position, increment



if __name__ == "__main__":
    # Initialize the RealSense pipeline
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
    FO = FeatureOdom(depth_intrin)

    while True:
        # Wait for a coherent pair of frames: depth and color
        if True:
            frames = pipeline.wait_for_frames()


            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()
            color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()


            if not depth_frame or not color_frame:
                continue

            position, increment, img_show= FO.run(aligned_color_frame, aligned_depth_frame, np.eye(3))


            cv2.imshow('Feature Tracking', img_show)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()