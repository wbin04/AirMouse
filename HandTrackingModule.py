import cv2
import numpy as np
import pyautogui
from HandDectionModule import HandDetector

class AirMouseControl:
    def __init__(self, detection_confidence=0.8):
        self.cap = cv2.VideoCapture(0)
        self.detector = HandDetector(detection_confidence)
        self.prev_x, self.prev_y = 0, 0

    def detect_hands(self):
        success, img = self.cap.read()
        if not success:
            return None, None
        
        hands, img = self.detector.find_hands(img)
        return hands, img
    
    def define_hands(self, bbox, hand_type, img):
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (255, 0, 255), 2)
        cv2.putText(img, hand_type, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

    def mouse_move(self, hand, img):
        index_x, index_y = hand["lmList"][8]  
        screen_x = np.interp(index_x, (100, img.shape[1] - 100), (self.detector.screen_width, 0))
        screen_y = np.interp(index_y, (100, img.shape[0] - 100), (0, self.detector.screen_height))

        cur_x = self.prev_x + (screen_x - self.prev_x) / self.detector.smooth_factor
        cur_y = self.prev_y + (screen_y - self.prev_y) / self.detector.smooth_factor
        pyautogui.moveTo(cur_x, cur_y)

        return cur_x, cur_y

    def mouse_click(self, type_click):
        if type_click == "LEFT":
            pyautogui.click()  
        elif type_click == "RIGHT":
            pyautogui.click(button='right')
        
    def mouse_scroll(self, type_scroll):
        if type_scroll == "DOWN":
            pyautogui.scroll(-20)
        elif type_scroll == "UP":
            pyautogui.scroll(20)
                
    def process_hands(self, hands, img):
        if hands:
            hand = hands[0]  
            fingers = self.detector.fingers_up(hand)
            bbox = hand["bbox"]
            hand_type = hand["type"]
            # print(fingers)

            if len(hand["lmList"]) < 13:
                return img

            self.define_hands(bbox, hand_type, img)    
            
            if fingers == [0, 0, 0, 0, 0]:
                self.mouse_scroll("DOWN")
            elif fingers == [1, 0, 0, 0, 0]:
                self.mouse_scroll("UP")

            elif fingers[1] == 1 and fingers[2] == 1 and fingers[3] == 1:        
                self.prev_x, self.prev_y = self.mouse_move(hand, img)

            elif fingers[1] == 0 and fingers[2] == 1:
                self.mouse_click("LEFT")
            elif fingers[1] == 0 and fingers[2] == 0:
                self.mouse_click("RIGHT")

            cv2.putText(img, f"Mouse: ({int(self.prev_x)}, {int(self.prev_y)})", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        
        return img

    def run(self):
        while True:
            hands, img = self.detect_hands()
            if img is not None:
                img = self.process_hands(hands, img)
                cv2.imshow("Air Mouse Control", img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()