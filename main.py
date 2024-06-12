import pyb
import ustruct
import time
import sensor
import math
import image

debug_mode = True

ball_threshold = [(26, 75, 5, 37, 21, 46), (30, 50, 5, 36, 15, 35)]
yellow_threshold = [(58, 76, -27, -2, 25, 56)]
blue_threshold = [(30, 50, -15, 15, -25, -10)]

center = (354, 211)
#center = (284, 194)
radius = 260

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

text = '0 000 000 0 000 000 0 000 000'
data = ustruct.pack("<bi%ds" % len(text), 85, len(text), text)
spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)

def nss_callback(line):
    global spi, data
    try:
        spi.send(data, timeout=1000)
    except OSError as err:
        pass

pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, nss_callback)

class Object:
    def __init__(self, blob, obj_type) -> None:
        self.blob = blob
        self.angle = 0
        self.dist = 0
        self.value = 0
        self.obj_type = obj_type

    def is_in_mirror(self):
        return math.sqrt((self.blob.cx() - center[0]) ** 2 + (self.blob.cy() - center[1]) ** 2) < radius

    def evaluate(self):
        self.value = 1
        if self.obj_type == 0:
            self.value *= self.blob.pixels()
            self.value *= self.blob.roundness()
            self.value *= self.blob.pixels() < 2000
            self.value *= self.is_in_mirror()
        else:
            self.value *= self.blob.pixels()
            self.value *= self.is_in_mirror()
        return self.value

    def calc_vector(self):
        self.angle = int(math.atan2(self.blob.cx() - center[0], self.blob.cy() - center[1]) / math.pi * 180 - 90 + 360) % 360
        self.dist = int(math.sqrt((self.blob.cx() - center[0]) ** 2 + (self.blob.cy() - center[1]) ** 2) / radius * 100)

    def draw(self, img, i, color):
        if self.obj_type == 0:
            img.draw_ellipse(self.blob.enclosed_ellipse(), color=color)
            img.draw_string(self.blob.cx(), self.blob.cy(), str(i), color=color, scale=2)
        else:
            img.draw_rectangle(self.blob.rect(), color=color)
            img.draw_string(self.blob.cx(), self.blob.cy(), str(i), color=color, scale=2)

    def draw_vector(self, img):
        img.draw_line(center[0], center[1], self.blob.cx(), self.blob.cy(), color=(0, 255, 0))



class ObjectArray:
    def __init__(self, threshold, obj_type) -> None:
        self.threshold = threshold
        self.obj_type = obj_type # 0 - ball, 1 - yellow goal, 2 - blue goal
        self.candidates = []
        self.match = None

    def generate(self, blob_list):
        self.candidates = []
        for blob in blob_list:
            self.candidates.append(Object(blob, self.obj_type))
            self.candidates[-1].evaluate()

    def find(self, img):
        if (self.obj_type == 0):
            self.generate(img.find_blobs(ball_threshold, pixels_threshold=20, area_threshold=20))
        else:
            self.generate(img.find_blobs(self.threshold, pixels_threshold=100, area_threshold=100, merge=True, margin=20))

        self.candidates = sorted(self.candidates, key=lambda obj: obj.value, reverse=True)

        self.match = None
        if len(self.candidates):
            if self.candidates[0].value > 0:
                self.match = self.candidates[0]

    def draw(self, img):
        for i in range(len(self.candidates)):
            if self.candidates[i].value > 0:
                if i == 0:
                    self.candidates[i].draw(img, i, color=(0, 255, 0))
                    self.candidates[i].draw_vector(img)
                else:
                    self.candidates[i].draw(img, i, color=(255, 255, 255))
            else:
                self.candidates[i].draw(img, i, color=(255, 0, 0))

    def trim(self, text):
        return (3 - len(str(text))) * '0' + str(text)

    def text(self):
        if self.match != None:
            self.match.calc_vector()
            return '1 ' + self.trim(self.match.angle) + ' ' + self.trim(self.match.dist)
        else:
            return '0 000 000'

ball_array = ObjectArray(ball_threshold, 0)
yellow_array = ObjectArray(yellow_threshold, 1)
blue_array = ObjectArray(blue_threshold, 2)

while True:
    clock.tick()
    img = sensor.snapshot()

    ball_array.find(img)
    yellow_array.find(img)
    blue_array.find(img)

    text = ball_array.text() + ' ' + yellow_array.text() + ' ' + blue_array.text()
    data = ustruct.pack("<bi%ds" % len(text), 85, len(text), text)

    if debug_mode:
        ball_array.draw(img)
        yellow_array.draw(img)
        blue_array.draw(img)
        img.draw_cross(center)
        img.draw_circle(center[0], center[1], radius)

    print(text, 'fps:', clock.fps())
