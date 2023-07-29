import time
import cv2

class FPS:
    def __init__(self):
        self.last_time = 0
        self.current_time = 0
        self.fps = 0.0
        self.confidence = 0.1

    def update(self):
        self.last_time = self.current_time
        self.current_time = time.time()
        new_fps = 1.0 / (self.current_time - self.last_time)
        if self.fps == 0.0:
            self.fps = new_fps if self.last_time != 0 else 0.0
        else:
            self.fps = new_fps * self.confidence + self.fps * (1.0 - self.confidence)
        return float(self.fps)

    def show_fps(self, img):
        font = cv2.FONT_HERSHEY_PLAIN
        line = cv2.LINE_AA
        fps_text = 'FPS: {:.2f}'.format(self.fps)
        cv2.putText(img, fps_text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
        cv2.putText(img, fps_text, (10, 20), font, 1.0, (240, 240, 240), 1, line)
        return img

