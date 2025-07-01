'''
修复版的MapVisualizer类，解决matplotlib兼容性问题
'''

import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
import numpy as np

# 这有助于在Raspberry Pi上运行
import matplotlib
matplotlib.use('TkAgg')

class Visualizer(object):

    # 机器人显示参数
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M  = 0.3

    def __init__(self, map_size_pixels, map_size_meters, title, show_trajectory=False, zero_angle=0):

        # 将原点放在中心
        self._init(map_size_pixels, map_size_meters, title, -map_size_pixels / 2, show_trajectory, zero_angle)

    def display(self, x_m, y_m, theta_deg):

        self._setPose(x_m, y_m, theta_deg)

        return self._refresh()

    def _init(self, map_size_pixels, map_size_meters, title, shift, show_trajectory=False, zero_angle=0):

        # 存储常量以便更新
        map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels
        self.map_scale_meters_per_pixel = map_size_meters / float(map_size_pixels)

        # 创建一个字节数组来显示带有颜色覆盖的地图
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)
        
        # 创建一个漂亮的大图形(10"x10")
        fig = plt.figure(figsize=(10,10))

        # 存储Python ID以检测窗口关闭
        self.figid = id(fig)

        # 兼容不同版本的matplotlib
        try:
            fig.canvas.set_window_title('SLAM')
        except:
            try:
                fig.canvas.manager.set_window_title('SLAM')
            except:
                pass
        plt.title(title)

        # 使用"artist"加速地图绘制
        self.img_artist = None

        # 尚无车辆显示
        self.vehicle = None

        # 创建坐标轴
        self.ax = fig.gca()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)

        # 因此我们必须重新标记坐标轴刻度以显示毫米
        ticks = np.arange(shift,self.map_size_pixels+shift+100,100)
        labels = [str(self.map_scale_meters_per_pixel * tick) for tick in ticks]
        self.ax.set_xticklabels(labels)
        self.ax.set_yticklabels(labels)

        # 存储以前的位置用于轨迹
        self.prevpos = None
        self.showtraj = show_trajectory

        # 我们基于像素设置坐标轴，以支持显示地图
        self.ax.set_xlim([shift, self.map_size_pixels+shift])
        self.ax.set_ylim([shift, self.map_size_pixels+shift])

        # 设置默认偏移以在原点居中
        shift = -self.map_size_pixels / 2

        self.zero_angle = zero_angle
        self.start_angle = None
        self.rotate_angle = 0

    def _setPose(self, x_m, y_m, theta_deg):
        '''
        设置车辆姿态:
        X:      左/右   (m)
        Y:      前/后 (m)
        theta:  旋转 (度)
        '''

        # 如果指示了零角度，抓取第一个角度来计算旋转
        if self.start_angle is None and self.zero_angle != 0: 
            self.start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # 按计算角度旋转，如果没有指示零角度，则为零
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m,y_m = x_m*c-y_m*s, y_m*c+x_m*s

        # 第一次迭代后擦除以前的车辆图像
        if not self.vehicle is None:
            self.vehicle.remove()

        # 使用非常短的箭头轴来定位箭头头部
        theta_rad = np.radians(theta_deg+d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        l = 0.1
        dx = l * c
        dy = l * s
 
        s = self.map_scale_meters_per_pixel

        self.vehicle=self.ax.arrow(x_m/s, y_m/s, 
                dx, dy, head_width=Visualizer.ROBOT_WIDTH_M/s, 
                head_length=Visualizer.ROBOT_HEIGHT_M/s, fc='r', ec='r')

        # 如果指示，显示轨迹
        currpos = self._m2pix(x_m,y_m)
        if self.showtraj and not self.prevpos is None:
            self.ax.add_line(mlines.Line2D((self.prevpos[0],currpos[0]), (self.prevpos[1],currpos[1])))
        self.prevpos = currpos

    def _refresh(self):                   

        # 如果我们有一个新的图形，说明出了问题(关闭图形失败)
        if self.figid != id(plt.gcf()):
            return False

        # 不阻塞地重绘当前对象
        plt.draw()

        # 刷新显示，在窗口关闭或键盘中断时设置标志
        try:
            plt.pause(.01) # 任意暂停以强制重绘
            return True
        except:
            return False

        return True

    def _m2pix(self, x_m, y_m):

        s = self.map_scale_meters_per_pixel

        return x_m/s, y_m/s
    
class MapVisualizer(Visualizer):
    
    def __init__(self, map_size_pixels, map_size_meters, title='MapVisualizer', show_trajectory=False):

        # 将原点放在左下角；不允许零角度设置
        Visualizer._init(self, map_size_pixels, map_size_meters, title, 0, show_trajectory, 0)

    def display(self, x_m, y_m, theta_deg, mapbytes):

        self._setPose(x_m, y_m, theta_deg)

        mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        # 暂停以允许显示刷新
        plt.pause(.001)

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray)

        else:

            self.img_artist.set_data(mapimg)

        return self._refresh() 