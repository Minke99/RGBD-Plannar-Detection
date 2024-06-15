# Libs.
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import time
import open3d as o3d



class RGBDNormal:
    def __init__(self, fx, fy, cx, cy, grid_size=5):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.grid_size = grid_size

    def create_point_cloud(self, depth_image):
        h, w = depth_image.shape
        x, y = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_image.astype(np.float32) / 1000.0  # 假设深度以毫米为单位，转换为米
        x = (x - self.cx) * z / self.fx
        y = (y - self.cy) * z / self.fy
        points = np.stack((x, y, z), axis=-1)
        return points

    def compute_normals(self, points):
        h, w, _ = points.shape
        normals = np.zeros_like(points)

        for i in range(0, h, self.grid_size):
            for j in range(0, w, self.grid_size):
                window_points = points[i:i+self.grid_size, j:j+self.grid_size].reshape(-1, 3)
                valid_points = window_points[window_points[:, 2] > 0]  # 只考虑深度大于0的点

                if len(valid_points) < 3:
                    continue

                centroid = np.mean(valid_points, axis=0)
                centered_points = valid_points - centroid
                cov_matrix = np.dot(centered_points.T, centered_points)
                _, _, vh = np.linalg.svd(cov_matrix)
                normal = vh[2, :]
                
                normals[i:i+self.grid_size, j:j+self.grid_size] = normal

        return normals

    def visualize(self, color_image, normals):
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        h, w, _ = color_image.shape
        visual_image = color_image.copy()

        for i in range(0, h, self.grid_size):
            for j in range(0, w, self.grid_size):
                normal = normals[i, j]
                if np.linalg.norm(normal) == 0:
                    continue

                color = (normal + 1) / 2 * 255
                color = tuple(map(int, color))
                cv2.rectangle(visual_image, (j, i), (j + self.grid_size, i + self.grid_size), color, -1)

        return visual_image

    def process_and_visualize(self, aligned_color_frame, aligned_depth_frame):
        points = self.create_point_cloud(aligned_depth_frame)
        normals = self.compute_normals(points)
        visual_image = self.visualize(aligned_color_frame, normals)
        return visual_image





    

# if __name__ == '__main__':

#     cf = ComplementaryFilter()

#     p = np.asarray([1,0,0])

#     pd = np.asarray([1,0,0])

#     a = np.asarray([1,0,0])

#     dt = 0.1

#     cf.fuse(p,pd,a,dt)