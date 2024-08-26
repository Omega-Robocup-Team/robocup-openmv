import pyb
import ustruct
import time
import sensor
import image
import math

debug_mode = True
robot_num = 1

ball_threshold = [(40, 59, 5, 30, 10, 39)]
yellow_threshold = [(40, 70, -20, 10, 20, 40)]
blue_threshold = [(30, 50, -15, 15, -25, -10)]

center = (358, 245) if robot_num else (354, 211)
radius = 340 if robot_num else 260


class TrackedObject:
    def __init__(self, obj_type: bool, threshold: list, search_margin: int, min_pixels: int, max_pixels: int) -> None:
        self.type = obj_type  # 0:ball | 1:goal
        self.threshold = threshold
        self.margin = search_margin
        self.min_pixels = min_pixels
        self.max_pixels = max_pixels
        self.roi = None
        self.new_value = 0
        self.best_match = None
        self.angle = 0
        self.dist = 0
        self.blobs = []
        self.refresh_time = 5
        self.time_buf = time.time()

    def get_roi(self, margin) -> tuple:
        if time.time() - self.time_buf > self.refresh_time:
            self.best_match = None
            self.time_buf = time.time()
        if self.best_match is not None:
            self.roi = []
            self.roi.append(
                max(0, center[0] - radius, self.best_match.x() - margin))
            self.roi.append(
                max(0, center[1] - radius, self.best_match.y() - margin))
            self.roi.append(
                min(self.best_match.x() + self.best_match.w() + margin, center[0] + radius, sensor.width()) - self.roi[0])
            self.roi.append(
                min(self.best_match.y() + self.best_match.h() + margin, center[1] + radius, sensor.height()) - self.roi[1])
        else:
            self.roi = []
            self.roi.append(max(0, center[0] - radius))
            self.roi.append(max(0, center[1] - radius))
            self.roi.append(min(center[0] + radius, sensor.width()) - self.roi[0])
            self.roi.append(min(center[1] + radius, sensor.height()) - self.roi[1])
        self.roi = list(map(int, self.roi))
        return self.roi

    def filter_blob(self, blob) -> bool:
        res = 1
        res *= self.min_pixels <= blob.pixels() <= self.max_pixels
        res *= math.sqrt((blob.cx() - center[0]) **
                         2 + (blob.cy() - center[1]) ** 2) <= radius
        return res

    def evaluate_blob(self, blob) -> float:
        res = blob.pixels()
        res *= blob.roundness() if self.type == 0 else 1
        return res

    def get_vector(self, blob) -> None:
        self.angle = int(math.atan2(
            blob.y() - center[1], blob.x() - center[0]) / math.pi * 180)
        self.dist = int(math.sqrt(
            (blob.cx() - center[0]) ** 2 + (blob.cy() - center[1]) ** 2) / radius * 100)

    def find(self, img) -> None:
        self.get_roi(self.margin)
        self.blobs = img.find_blobs(
            self.threshold,
            threshold_cb=self.filter_blob,
            roi=self.roi,
            merge=self.type,
            margin=25 if self.type else 0,
            x_stride=10 if self.type else 5,
            y_stride=10 if self.type else 5
        )
        if len(self.blobs) > 0:
            self.best_match = max(self.blobs, key=self.evaluate_blob)
            self.get_vector(self.best_match)
            self.new_value = 1
        else:
            self.best_match = None

    def draw_roi(self, img) -> None:
        if self.roi is not None:
            img.draw_rectangle(self.roi, color=(255, 0, 0), thickness=1)

    def draw_vector(self, img) -> None:
        if self.best_match is not None:
            img.draw_line((self.best_match.cx(), self.best_match.cy(
            ), center[0], center[1]), color=(255, 255, 255), thickness=1)

    def draw_blob(self, img, blob, color) -> None:
        if self.type == 0:
            img.draw_ellipse(blob.enclosed_ellipse(), color=color, thickness=1)
        else:
            img.draw_rectangle(blob.rect(), color=color, thickness=1)

    def draw(self, img) -> None:
        self.draw_roi(img)
        self.draw_vector(img)
        for blob in self.blobs:
            self.draw_blob(img, blob, (0, 255, 0)
                           if blob is self.best_match else (255, 255, 255))

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.SVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)

ball = TrackedObject(0, ball_threshold, 50, 100, 3000)
yellow_goal = TrackedObject(1, yellow_threshold, 50, 500, 20000)
blue_goal = TrackedObject(1, blue_threshold, 50, 5000, 20000)

data = None


def generate_data():
    global data
    data = ustruct.pack('<bbhhbhhbhh', 85, ball.new_value, ball.angle, ball.dist, yellow_goal.new_value,
                        yellow_goal.angle, yellow_goal.dist, blue_goal.new_value, blue_goal.angle, blue_goal.dist)


def nss_callback(line):
    global spi, data, ball, yellow_goal, blue_goal
    try:
        spi.send(data, timeout=1000)
        ball.new_value = 0
        yellow_goal.new_value = 0
        blue_goal.new_value = 0
    except OSError as err:
        pass


generate_data()
pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING,
           pyb.Pin.PULL_UP, nss_callback)

while True:
    clock.tick()
    img = sensor.snapshot()

    ball.find(img)
#    yellow_goal.find(img)
#    blue_goal.find(img)

    generate_data()

    if debug_mode:
#        yellow_goal.draw(img)
#        blue_goal.draw(img)
        ball.draw(img)
        img.draw_cross(center)
        img.draw_circle(center[0], center[1], radius)

    print(ball.new_value, ball.angle, ball.dist, yellow_goal.new_value, yellow_goal.angle,
          yellow_goal.dist, blue_goal.new_value, blue_goal.angle, blue_goal.dist, 'fps:', clock.fps())
