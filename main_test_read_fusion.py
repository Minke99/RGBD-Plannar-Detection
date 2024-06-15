import scipy.io
import matplotlib.pyplot as plt
from Lib.ComplementaryFilter import ComplementaryFilter
from Lib.EKF import KalmanFilter
import numpy as np

def diff_extend(x, t, step):
    # 计算中间部分的差分
    temp_vx = (x[step:] - x[:-step]) / (t[step:] - t[:-step])
    
    # 计算两端的补充部分
    temp1 = np.ones(step // 2) * temp_vx[0]
    temp2 = np.ones(step // 2) * temp_vx[-1]
    
    # 拼接结果
    vx = np.concatenate((temp1, temp_vx, temp2))
    
    return vx

# 读取.mat文件
mat = scipy.io.loadmat('Sets/20240613_152821.mat')

# 计算5%到95%的索引范围
n = len(mat['Abs_time'].flatten())
start_index = int(0.01 * n)
end_index = int(0.9 * n)

# 提取并截取数据
acc_Wx = mat['acc_Wx'].flatten()[start_index:end_index]
acc_Wy = mat['acc_Wy'].flatten()[start_index:end_index]
acc_Wz = mat['acc_Wz'].flatten()[start_index:end_index]
cam_px = mat['cam_px'].flatten()[start_index:end_index]
cam_py = mat['cam_py'].flatten()[start_index:end_index]
cam_pz = mat['cam_pz'].flatten()[start_index:end_index]
cam_inc_x = mat['cam_inc_x'].flatten()[start_index:end_index]
cam_inc_y = mat['cam_inc_y'].flatten()[start_index:end_index]
cam_inc_z = mat['cam_inc_z'].flatten()[start_index:end_index]
x = mat['x'].flatten()[start_index:end_index]
y = mat['y'].flatten()[start_index:end_index]
z = mat['z'].flatten()[start_index:end_index]
t = mat['Abs_time'].flatten()[start_index:end_index]

vx = diff_extend(x, t,6)
vy = diff_extend(y, t,6)
vz = diff_extend(z, t,6)


vx_cam = cam_inc_x/(1/30)
vy_cam = cam_inc_y/(1/30)
vz_cam = cam_inc_z/(1/30)


Fusion = KalmanFilter()
# Fusion = ComplementaryFilter()


# 按时间序列运行提取的变量
time_series_length = len(acc_Wx)  # 假设所有变量长度相同
index = 0

# 初始化列表以存储est的结果
est_results = []

while index < time_series_length:
    # 在这里处理每个时间点的数据
    position = [cam_px[index], cam_py[index], cam_pz[index]]
    increment = [cam_inc_x[index], cam_inc_y[index], cam_inc_z[index]]
    acc_W = [acc_Wx[index], acc_Wy[index], acc_Wz[index]]

    est = Fusion.fuse(position, increment, acc_W, dt=1/30)
    
    # 记录结果
    est_results.append(est)
    
    index += 1

# 将 est_results 转换为 numpy 数组以便于后续处理
est_results = np.array(est_results)

# 绘制图像
fig, axs = plt.subplots(3, 2, figsize=(15, 15))

# 绘制左列的三个图
axs[0, 0].plot(vx)
axs[0, 0].plot(vx_cam)
axs[0, 0].plot(est_results[:, 3], color='black')
axs[0, 0].set_title('vx')
axs[0, 0].set_xlabel('Time Step')
axs[0, 0].set_ylabel('Value')

axs[1, 0].plot(vy)
axs[1, 0].plot(vy_cam)
axs[1, 0].plot(est_results[:, 4], color='black')
axs[1, 0].set_title('vy')
axs[1, 0].set_xlabel('Time Step')
axs[1, 0].set_ylabel('Value')

axs[2, 0].plot(vz)
axs[2, 0].plot(vz_cam)
axs[2, 0].plot(est_results[:, 5], color='black')
axs[2, 0].set_title('vz')
axs[2, 0].set_xlabel('Time Step')
axs[2, 0].set_ylabel('Value')

# 绘制右列的三个图
axs[0, 1].plot(x - x[0])
axs[0, 1].plot(est_results[:, 0], color='black')
axs[0, 1].set_title('x')
axs[0, 1].set_xlabel('Time Step')
axs[0, 1].set_ylabel('Value')

axs[1, 1].plot(y - y[0])
axs[1, 1].plot(est_results[:, 1], color='black')
axs[1, 1].set_title('y')
axs[1, 1].set_xlabel('Time Step')
axs[1, 1].set_ylabel('Value')

axs[2, 1].plot(z - z[0])
axs[2, 1].plot(est_results[:, 2], color='black')
axs[2, 1].set_title('z')
axs[2, 1].set_xlabel('Time Step')
axs[2, 1].set_ylabel('Value')

plt.tight_layout()
plt.show()


msex = np.mean((vx - est_results[:, 3])**2)
msey = np.mean((vy - est_results[:, 4])**2)
msez = np.mean((vz - est_results[:, 5])**2)
print(msex,msey,msez)