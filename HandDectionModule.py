import cv2
import mediapipe as mp
import pyautogui

class HandDetector:
    def __init__(self, detection_confidence=0.8, max_hands=1):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=detection_confidence, max_num_hands=max_hands)
        self.mp_draw = mp.solutions.drawing_utils
        self.screen_width, self.screen_height = pyautogui.size()
        self.smooth_factor = 5  

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        hands_data = []
        
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                hand_type = "Right" if handedness.classification[0].label == "Left" else "Left"
                hand_info = {
                    "lmList": [(int(lm.x * img.shape[1]), int(lm.y * img.shape[0])) for lm in hand_landmarks.landmark],
                    "bbox": self._calculate_bounding_box(hand_landmarks, img),
                    "center": self._calculate_center(hand_landmarks, img),
                    "type": hand_type
                }
                hands_data.append(hand_info)
                
                if draw:
                    self.mp_draw.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        
        return hands_data, img

    def _calculate_bounding_box(self, hand_landmarks, img):
        x_list = [int(lm.x * img.shape[1]) for lm in hand_landmarks.landmark]
        y_list = [int(lm.y * img.shape[0]) for lm in hand_landmarks.landmark]
        return min(x_list), min(y_list), max(x_list) - min(x_list), max(y_list) - min(y_list)
    
    def _calculate_center(self, hand_landmarks, img):
        x_list = [int(lm.x * img.shape[1]) for lm in hand_landmarks.landmark]
        y_list = [int(lm.y * img.shape[0]) for lm in hand_landmarks.landmark]
        return sum(x_list) // len(x_list), sum(y_list) // len(y_list)

    def fingers_up(self, hand):
        lmList = hand["lmList"]
        fingers = []

        if lmList[4][0] > lmList[3][0]:  
            fingers.append(1)
        else:
            fingers.append(0)

        tips = [8, 12, 16, 20]
        for tip in tips:
            fingers.append(1 if lmList[tip][1] < lmList[tip - 2][1] else 0)

        return fingers