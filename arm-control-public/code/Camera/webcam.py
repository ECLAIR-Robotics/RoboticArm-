import time
import cv2 as cv


class Webcam:
    CLOSE_WINDOW_KEY = 'q'

    def __init__(self, webcam_id=0):
        self.id = webcam_id
        self._capture = cv.VideoCapture(self.id)

        if not self._capture.isOpened():  # if failed to open
            raise Exception('Failed to open webcam, do you have the right id?')

    def capture(self):  # returns (H, W, C) ndarray where color is BGR
        success, frame = self._capture.read()
        if not success:
            raise Exception('Failed to capture image.')

        return frame

    def videoFeed(self):
        while True:
            image = self.capture()
            cv.imshow(f'Press {Webcam.CLOSE_WINDOW_KEY} to close', image)
            if cv.waitKey(1) == ord(Webcam.CLOSE_WINDOW_KEY):
                break

    @staticmethod
    def display(image):
        while True:
            cv.imshow(f'Press {Webcam.CLOSE_WINDOW_KEY} to close', image)
            if cv.waitKey(1) == ord(Webcam.CLOSE_WINDOW_KEY):
                break
