import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import matplotlib
import threading
import serial
import time
import sys

# 基础配置：解决中文显示
matplotlib.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
matplotlib.use('TkAgg')  # 确保GUI后端兼容

# 全局变量定义
radar_data = deque(maxlen=700)  # 雷达数据缓存（限制最大长度，避免内存溢出）
axis_angles = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}  # 小车三轴角度
data_buffer = ""  # 串口断帧数据拼接缓冲区

# 串口配置（需与硬件一致）
COM_PORT = "COM3"  # 目标串口（设备管理器中确认）
BAUDRATE = 115200  # 波特率（需与硬件一致）


def parse_buffer_data():
    """解析缓冲区数据：优先处理角度，再处理雷达数据"""
    global data_buffer, axis_angles, radar_data
    parsed_flag = False

    # 1. 解析三轴角度数据（格式：roll:12.34, pitch:-5.67, yaw:90.0）
    angle_matches = re.findall(r'(roll|pitch|yaw):([-+]?\d*\.\d+)', data_buffer)
    if angle_matches:
        temp_angles = axis_angles.copy()
        for axis_name, value_str in angle_matches:
            try:
                temp_angles[axis_name] = float(value_str)
                print(f"[角度解析] {axis_name}: {temp_angles[axis_name]:.2f}°")
                parsed_flag = True
            except Exception as e:
                print(f"[角度错误] {axis_name}:{value_str} | 错误：{e}")
        axis_angles.update(temp_angles)
        # 从缓冲区移除已解析的角度数据（避免重复解析）
        data_buffer = re.sub(r'(roll|pitch|yaw):[-+]?\d*\.\d+', "", data_buffer)
    window_size = 3
    max_diff_threshold = 1
    std_factor = 1.0
    # 2. 解析雷达数据（格式：A:123.45, D:0.678m）
    parsed_flag = False
    radar_matches = re.findall(r'A:([-+]?\d*\.\d+),\s*D:([-+]?\d*\.\d+)m', data_buffer)

    if radar_matches:
        new_points = []
        # 解析新提取的点
        for angle_str, dist_str in radar_matches:
            try:
                angle_deg = float(angle_str)
                distance_m = float(dist_str)
                new_points.append((np.radians(angle_deg), distance_m))
                print(f"[原始解析] 角度：{angle_deg:.1f}° | 距离：{distance_m:.3f}m")
                parsed_flag = True
            except Exception as e:
                print(f"[解析错误] {angle_str},{dist_str} | 错误：{e}")

        # 合并历史数据与新数据（按角度排序）
        combined = sorted(deque(radar_data) + deque(new_points), key=lambda x: x[0])

        # 仅进行异常值检测（不做平滑）
        filtered = []
        for i, (angle, dist) in enumerate(combined):
            # 查找相邻点（窗口内的前后点）
            neighbors = []
            for j in range(max(0, i - window_size), i):
                neighbors.append(combined[j][1])
            for j in range(i + 1, min(len(combined), i + 1 + window_size)):
                neighbors.append(combined[j][1])

            # 判断是否为异常点（条件更宽松）
            is_abnormal = False
            if len(neighbors) >= 2:  # 至少2个邻居才判断，避免初始数据误判
                neighbor_mean = np.mean(neighbors)
                neighbor_std = np.std(neighbors)

                # 异常判断：与平均值偏差超过3倍标准差，且与所有邻居差值都超过阈值
                diff_from_mean = abs(dist - neighbor_mean)
                all_neighbors_exceed = all(abs(dist - n) > max_diff_threshold for n in neighbors)

                if diff_from_mean > std_factor * neighbor_std and all_neighbors_exceed:
                    is_abnormal = True
                    print(f"[异常过滤] 角度：{np.degrees(angle):.1f}° | 距离：{dist:.3f}m "
                          f"(邻居平均：{neighbor_mean:.3f}m | 偏差：{diff_from_mean:.3f}m)")

            # 非异常点直接保留原始值（不做平滑）
            if not is_abnormal:
                filtered.append((angle, dist))
            # 异常点直接丢弃，不做任何替代处理

        # 更新雷达数据（去重，保留最新值）
        angle_dict = {angle: dist for angle, dist in filtered}
        radar_data.clear()
        radar_data.extend(sorted(angle_dict.items(), key=lambda x: x[0]))

        # 清理缓冲区
        data_buffer = re.sub(r'A:[-+]?\d*\.\d+,\s*D:[-+]?\d*\.\d+m', "", data_buffer)
    # 缓冲区过长时清理
    if not parsed_flag and len(data_buffer) > 600:
        print(f"[缓冲区清理] 丢弃前500字符 | 剩余：{data_buffer[300:350]}...")
        data_buffer = data_buffer[-300:]

    return data_buffer, radar_data


def serial_read_thread():
    """串口读取线程：持续从COM14读取数据并解析"""
    global data_buffer
    ser = None
    try:
        # 打开串口
        ser = serial.Serial(
            port=COM_PORT,
            baudrate=BAUDRATE,
            timeout=0.1,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        print(f"✅ 成功打开串口：{COM_PORT}（波特率：{BAUDRATE}）")

        # 持续读取直到图表窗口关闭
        while plt.get_fignums():
            if ser.in_waiting > 0:
                # 读取一行数据（按换行符分割）
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    data_buffer += line
                    print(f"[串口数据] 收到：{line[:50]}")
                    parse_buffer_data()  # 解析数据
            time.sleep(0.01)  # 降低CPU占用

    except serial.SerialException as e:
        print(f"❌ 串口异常：{e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 数据读取异常：{e}")
    finally:
        # 确保串口关闭
        if ser and ser.is_open:
            ser.close()
            print("📌 串口已安全关闭")


def update_plot(frame, ax, radar_line, angle_text, manual_zoom_flag):
    """更新极坐标图和角度文本"""
    # 1. 雷达数据更新
    if radar_data:
        angles_rad, distances_m = zip(*radar_data)
        radar_line.set_data(angles_rad, distances_m)

        # 仅当未手动缩放时，自动适配距离范围
        if not manual_zoom_flag[0]:
            max_dist = max(distances_m) if max(distances_m) > 0 else 10
            ax.set_ylim(0, max_dist * 1.1)  # 预留10%余量

    # 2. 角度文本更新
    angle_text.set_text(
        "小车三轴角度（实时更新）\n"
        f"横滚角(roll)：{axis_angles['roll']:.2f}°\n"
        f"俯仰角(pitch)：{axis_angles['pitch']:.2f}°\n"
        f"偏航角(yaw)：{axis_angles['yaw']:.2f}°"
    )

    return radar_line, angle_text


def main():
    # 1. 串口预检查（提前验证串口可用性）
    try:
        test_ser = serial.Serial(COM_PORT, BAUDRATE, timeout=0.5)
        test_ser.close()
    except serial.SerialException as e:
        print(f"❌ 串口{COM_PORT}打开失败：{e}")
        print("请检查：① 串口是否被占用；② 波特率是否与硬件一致；③ 设备是否连接。")
        return

    # 2. 创建极坐标图表
    fig = plt.figure(figsize=(12, 8))
    ax = plt.subplot(111, polar=True)

    # 图表基础配置
    ax.set_theta_zero_location('N')  # 0°方向朝北（上）
    ax.set_theta_direction(-1)  # 角度顺时针递增
    ax.set_ylim(0, 10)  # 初始距离范围（米）
    ax.set_ylabel('距离 (米)', labelpad=20, fontsize=10)
    ax.set_title(
        "小车雷达实时扫描（50ms/帧 | 鼠标滚轮缩放）",
        pad=20, fontsize=12, fontweight="bold"
    )

    # 3. 初始化绘图元素
    radar_line, = ax.plot([], [], 'bo', markersize=3)  # 雷达扫描点（蓝色）
    angle_text = plt.text(
        1.25, 0.5, "等待数据...",
        transform=ax.transAxes,
        fontsize=11,
        bbox=dict(facecolor="lightblue", alpha=0.9, boxstyle="round,pad=0.5")
    )

    # ------------------- 鼠标滚轮缩放功能 -------------------
    manual_zoom_flag = [False]  # 标记是否手动缩放（列表实现可变传递）
    ZOOM_RATIO = 0.8  # 缩放比例（向上滚=放大，向下滚=缩小）
    MIN_DIST = 0.1  # 最小显示距离（米）
    MAX_DIST = 50.0  # 最大显示距离（米）

    def on_scroll(event):
        if event.inaxes != ax:
            return
        current_max = ax.get_ylim()[1]
        # 根据滚轮方向计算新范围
        new_max = current_max * ZOOM_RATIO if event.step > 0 else current_max / ZOOM_RATIO
        new_max = max(MIN_DIST, min(new_max, MAX_DIST))  # 限制范围
        ax.set_ylim(0, new_max)
        ax.figure.canvas.draw()
        manual_zoom_flag[0] = True  # 标记为手动缩放
        print(f"[缩放] 距离范围：0 ~ {new_max:.2f}米")

    fig.canvas.mpl_connect('scroll_event', on_scroll)
    # ---------------------------------------------------------

    # 4. 启动动画（传递手动缩放标志）
    ani = FuncAnimation(
        fig,
        update_plot,
        fargs=(ax, radar_line, angle_text, manual_zoom_flag),
        interval=5,  # 每50ms更新一帧
        blit=True,  # 硬件加速（只更新变化区域）
        cache_frame_data=False  # 禁用帧缓存，确保实时性
    )

    # 5. 启动串口读取线程
    serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
    serial_thread.start()

    # 6. 显示图表（阻塞直到窗口关闭）
    plt.tight_layout()
    plt.show()

    print("\n📌 程序正常退出")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n📌 程序被手动终止（Ctrl+C）")
    except Exception as e:
        print(f"\n❌ 程序异常：{e}")