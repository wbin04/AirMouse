import cv2
import numpy as np
import HandTrackingModule as htm
import time
import pyautogui
from pynput.mouse import Controller

class VirtualMouse:
    def __init__(self, wCam=640, hCam=480, frameR=100, smoothening=7):
        self.wCam, self.hCam = wCam, hCam
        self.frameR = frameR
        self.smoothening = smoothening
        
        self.pTime = 0
        self.plocX, self.plocY = 0, 0
        self.clocX, self.clocY = 0, 0
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(3, self.wCam)
        self.cap.set(4, self.hCam)
        
        self.detector = htm.handDetector(maxHands=1)
        self.wScr, self.hScr = pyautogui.size()
        self.mouse = Controller()

    def process_frame(self):
        self.cap.grab()
        success, img = self.cap.retrieve()
        if not success:
            return None
        img = self.detector.findHands(img)
        lmList, bbox = self.detector.findPosition(img)
        return img, lmList

    def move_mouse(self, x1, y1):
        x3 = np.interp(x1, (self.frameR, self.wCam - self.frameR), (0, self.wScr))
        y3 = np.interp(y1, (self.frameR, self.hCam - self.frameR), (0, self.hScr))
        self.clocX = self.plocX + (x3 - self.plocX) / self.smoothening
        self.clocY = self.plocY + (y3 - self.plocY) / self.smoothening
        pyautogui.moveTo(self.wScr - self.clocX, self.clocY, duration=0.1)
        self.plocX, self.plocY = self.clocX, self.clocY

    def click_mouse(self, lineInfo):
        pyautogui.click()
        time.sleep(0.3)

    def run(self):
        while True:
            frame_data = self.process_frame()
            if frame_data is None:
                continue
            img, lmList = frame_data

            if len(lmList) != 0:
                x1, y1 = lmList[8][1:], lmList[12][1:]
                fingers = self.detector.fingersUp() if len(lmList) != 0 else [0, 0, 0, 0, 0]
                
                cv2.rectangle(img, (self.frameR, self.frameR), (self.wCam - self.frameR, self.hCam - self.frameR), (255, 0, 255), 2)
                
                if len(fingers) >= 3 and fingers[1] == 1 and fingers[2] == 0:
                    self.move_mouse(x1[0], y1[0])
                    cv2.circle(img, (x1[0], y1[0]), 15, (255, 0, 255), cv2.FILLED)
                
                if fingers[1] == 1 and fingers[2] == 1:
                    length, img, lineInfo = self.detector.findDistance(8, 12, img)
                    if length < 40:
                        cv2.circle(img, (lineInfo[4], lineInfo[5]), 15, (0, 255, 0), cv2.FILLED)
                        self.click_mouse(lineInfo)

            cTime = time.time()
            fps = 1 / (cTime - self.pTime)
            self.pTime = cTime
            cv2.putText(img, str(int(fps)), (20, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)
            cv2.imshow("Image", img)
            cv2.waitKey(1)