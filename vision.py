import time

import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
import threading

NUMBER = -1
TRACKER = False

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883

def on_connect(client, userdata, flags, rc):
    # print("Connected with result code " + str(rc))
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)


def on_message(client, userdata, message):
    global NUMBER, TRACKER

    msg = message.payload.decode()
    print('Ho ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic):
        if msg == '-1':
            # Richiesta di numero
            TRACKER = True


class handTracker:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5,modelComplexity=1,trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self, image, draw=True):
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def handsFinder(self,image,draw=True):
        imageRGB = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image

    def positionFinder(self, image, handNo=0):
        lmlist = []
        if self.results.multi_hand_landmarks:
            Hand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(Hand.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])

        return lmlist

def VisionTask():
    global NUMBER, TRACKER

    cap = cv2.VideoCapture(0)
    tracker = handTracker()
    tipIds = [4, 8, 12, 16, 20]
    consecutive_same_count = 0
    target_value = None

    while True:
        success, image = cap.read()
        if TRACKER:
            image = tracker.handsFinder(image)
            lmList = tracker.positionFinder(image)
            if len(lmList) != 0:
                fingers = []

                if lmList[tipIds[0]][1] < 320:  # Thumb dx
                    if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
                else:   # Thumb sx
                    if lmList[tipIds[0]][1] < lmList[tipIds[0] - 1][1]:
                        fingers.append(1)
                    else:
                        fingers.append(0)

                # 4 Fingers
                for id in range(1, 5):
                    if lmList[tipIds[id]][2] < lmList[tipIds[id]-2][2]:
                        fingers.append(1)
                    else:
                        fingers.append(0)

                totalFingers = fingers.count(1)

                if totalFingers == target_value:
                    consecutive_same_count += 1
                    if consecutive_same_count >= 20:
                        print(f"Stesso valore ({totalFingers}) per 20 volte.")
                        NUMBER = totalFingers
                        mqtt_client.publish(f'connect4/user', NUMBER)
                        TRACKER = False
                        NUMBER = -1
                else:
                    consecutive_same_count = 0
                    target_value = totalFingers

                # TODO: MANDARE A ROBOT VALORE OTTENUTO
                print(totalFingers)

        cv2.imshow("Video", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':
    topic_user = 'connect4/user'
    topic_robot = 'connect4/robot'

    mqtt_client = mqtt.Client('Vision')
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # print("Connecting to " + broker + " port: " + str(broker_port))
    mqtt_client.connect(broker, broker_port)

    t1 = threading.Thread()
    t1.start()
    t2 = threading.Thread(target=mqtt_client.loop_forever)
    t2.start()

    VisionTask()
