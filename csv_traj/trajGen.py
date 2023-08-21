import numpy as np
import math
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# 生成匀速8字形轨迹
def lemniscate_traj_create(p_bias, x_len, dt, v):
    if x_len <= 0 or dt <= 0 or v <= 0:
        print("Infeasible Parameters. Generate Nothing.")
        return
    
    l = v * dt      # 步长：每个周期的长度
    a = x_len/math.sqrt(2)  # 方程的参数，x轴方向的半长为sqrt(2)*a
    timestamps = np.array([0])
    p_traj = np.array([[p_bias[0]], [p_bias[1]], [p_bias[2]]])
    v_traj = np.array([[0], [v], [0]])
    a_traj = np.array([[0], [0], [0]])
    # x in [x_bias-x_len, x_bias+x_len]
    l_tem = 0
    x_old = 0
    y_old = 0
    _dd = 2*x_len / 100000
    for i in range(1, 100000):  # x4
        x = _dd * i
        p = np.array([1, 0, 2*((x-x_len)**2) + 2*(a**2), 0, ((x-x_len)**4) - 2*(a**2)*((x-x_len)**2)])
        roots = np.roots(p)
        y_list = np.real(roots[abs(np.imag(roots)) < 0.0001])    # 只取实数根,升序
        y_list.sort()
        if x < x_len:
            y = y_list[-1]
        else:
            y = y_list[0]
        l_tem += np.linalg.norm(np.array([x-x_old, y-y_old]))
        x_old = x
        y_old = y
        if l_tem >= l:
            p_traj = np.hstack([p_traj, np.array([[x+p_bias[0]], [y+p_bias[1]], [p_bias[2]]])])
            l_tem = l_tem - l
    # x in [x_bias+x_len, x_bias-x_len]
    for i in range(100000):    # x4
        x = 2*x_len - _dd * i
        p = np.array([1, 0, 2*((x-x_len)**2) + 2*(a**2), 0, ((x-x_len)**4) - 2*(a**2)*((x-x_len)**2)])
        roots = np.roots(p)
        y_list = np.real(roots[abs(np.imag(roots)) < 0.0001])  # 只取实数根,升序
        y_list.sort()
        if x > x_len:
            y = y_list[-1]
        else:
            y = y_list[0]
        l_tem += np.linalg.norm(np.array([x-x_old, y-y_old]))
        x_old = x
        y_old = y
        if l_tem >= l:
            p_traj = np.hstack([p_traj, np.array([[x+p_bias[0]], [y+p_bias[1]], [p_bias[2]]])])
            l_tem = l_tem - l
    # 求速度、加速度
    for i in range(1, len(p_traj[0, :])):
        timestamps = np.append(timestamps, [i*dt])
        g_v = p_traj[:, i:i+1] - p_traj[:, i-1:i]
        g_v = v * g_v/np.linalg.norm(g_v)
        v_traj = np.hstack([v_traj, g_v])
        g_a = g_v - v_traj[:, i-1:i]
        g_a = g_a/dt
        a_traj = np.hstack([a_traj, g_a])

    return timestamps, p_traj, v_traj, a_traj


ts, p_traj, v_traj, a_traj = lemniscate_traj_create([2,4,3], 2, 0.02, 2)

# p_traj, v_traj, a_traj = traj_8_create()
print("head:", p_traj[:,0], "    ", "tail:", p_traj[:,-1])

ax = plt.figure().add_subplot(projection='3d')
ax.plot(p_traj[0,:], p_traj[1,:], p_traj[2,:], label='parametric curve')
ax.legend()
plt.show()

with open('csv_traj/lemniscate.csv', 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile)
    spamwriter.writerow(["ts", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az"])
    for i in range(len(p_traj[0, :])):
        spamwriter.writerow([ts[i], 
                             p_traj[0,i], p_traj[1,i], p_traj[2,i], 
                             v_traj[0,i], v_traj[1,i], v_traj[2,i], 
                             a_traj[0,i], a_traj[1,i], a_traj[2,i]])
        