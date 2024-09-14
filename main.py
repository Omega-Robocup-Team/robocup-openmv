import pyb
import ustruct
import time
import sensor
import image
import math

debug_mode = 1
robot_num = 0

ball_threshold = [(15, 48, 26, 66, -5, 54)]
#ball_threshold = [(20, 45, 0, 30, 4, 34)]
yellow_threshold = [(30, 42, 1, 18, 7, 47)]
#yellow_threshold = [(35, 39, -10, 7, 12, 34)]
blue_threshold = [(20, 30, -18, 1, -37, -6)]
#blue_threshold = [(25, 31, -14, 5, -17, -1)]

line_threshold = [(45, 60, -32, -11, -11, 20)]
#line_threshold = [(54, 72, -20, -6, -5, 20)]

center = (383, 264) if robot_num else (454, 276)
radius = 320 if robot_num else 340
inner_radius = 75

fps = 0


class TrackedObject:
    def __init__(self, obj_type: bool, threshold: list, search_margin: int, min_pixels: int, max_pixels: int) -> None:
        self.type = obj_type  # 0:ball | 1:goal
        self.threshold = threshold
        self.margin = search_margin
        self.min_pixels = min_pixels
        self.max_pixels = max_pixels
        self.roi = None
        self.new_value = 0
        self.value_id = 0
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
        res *= math.sqrt((blob.cx() - center[0]) ** 2 + (blob.cy() - center[1]) ** 2) <= radius
        res *= math.sqrt((blob.cx() - center[0]) ** 2 + (blob.cy() - center[1]) ** 2) >= inner_radius

        return res

    def evaluate_blob(self, blob) -> float:
        res = blob.pixels()
        res *= blob.roundness() if self.type == 0 else 1
        return res

    def get_vector(self, blob) -> None:
        self.angle = int(math.atan2(blob.cx() - center[0], blob.cy() - center[1]) / math.pi * 180 - 90 + 360) % 360
        self.dist = int(math.sqrt((blob.cx() - center[0]) ** 2 + (blob.cy() - center[1]) ** 2) / radius * 100)

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
            self.value_id = self.value_id % 100 + 1
            self.new_value = self.value_id
        else:
            self.best_match = None
            self.new_value = 0

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

class Line:
    def __init__(self, threshold) -> None:
        self.threshold = threshold
        self.arr = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.segment_size = 15
        self.segment_dist = 100
        self.segment_n = 16

        self.visible = False
        self.angle = 0
        self.dist = 0

    def find(self, img) -> None:
        self.arr = []
        self.visible = 0
        for i in range(self.segment_n):
            self.arr.append(self.find_segment(img, i))

    def find_segment(self, img, i) -> None:
        blobs = img.find_blobs(
            self.threshold,
            roi=self.get_roi(i),
            merge=True,
            margin=self.segment_size * 2,
            area_threshold=200,
        )
        return len(blobs) > 0

    def get_roi(self, i) -> None:
        angle, x, y = self.get_coord(i)
        return (x - self.segment_size, y - self.segment_size, self.segment_size * 2, self.segment_size * 2)

    def draw(self, img) -> None:
        for i in range(len(self.arr)):
            angle, x, y = self.get_coord(i)
            img.draw_rectangle(self.get_roi(i), color=(255, 0, 0) if self.arr[i] else (0, 255, 0), thickness=1)
            img.draw_string(x, y, str(i), (255, 255, 255))

    def get_coord(self, i):
        angle = i * (360 / self.segment_n) / 180 * math.pi
        x = center[0] + int(self.segment_dist * math.cos(angle))
        y = center[1] - int(self.segment_dist * math.sin(angle))
        return angle, x, y

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.SVGA)
sensor.skip_frames(time=2000)
auto_gain_db = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db = auto_gain_db * 0.7)
sensor.set_auto_whitebal(False)
sensor.set_contrast(-3)

#sensor.set_brightness(3)

clock = time.clock()
spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)

ball = TrackedObject(0, ball_threshold, 50, 20, 8000)
yellow_goal = TrackedObject(1, yellow_threshold, 50, 0, 50000)
blue_goal = TrackedObject(1, blue_threshold, 50, 0, 50000)
line = Line(line_threshold)

data = None


def generate_data():
    global data
    data = ustruct.pack(f'<bbhhbhhbhh{'b'*16}b', 85, ball.new_value, ball.angle, ball.dist, yellow_goal.new_value,
                        yellow_goal.angle, yellow_goal.dist, blue_goal.new_value, blue_goal.angle, blue_goal.dist, *list(line.arr[i] for i in range(16)), int(fps))


def nss_callback(line):
    global spi, data, ball, yellow_goal, blue_goal
    try:
        spi.send(data, timeout=1000)
    except OSError as err:
        pass

generate_data()
pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING,
           pyb.Pin.PULL_UP, nss_callback)

while True:
    clock.tick()
    img = sensor.snapshot()

    ball.find(img)
    yellow_goal.find(img)
    blue_goal.find(img)
    line.find(img)

    generate_data()

    if debug_mode:
        yellow_goal.draw(img)
        blue_goal.draw(img)
        ball.draw(img)
        img.draw_cross(center)
        img.draw_circle(center[0], center[1], radius)
        img.draw_circle(center[0], center[1], inner_radius)
        line.draw(img)

    fps = clock.fps()
    print(ball.dist)
#    print(ball.new_value, ball.angle, ball.dist, yellow_goal.new_value, yellow_goal.angle,
#          yellow_goal.dist, blue_goal.new_value, blue_goal.angle, blue_goal.dist, 'fps:', fps)
