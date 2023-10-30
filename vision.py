import time
import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
import threading

NUMBER = -1
TRACKER = False
topic_user = 'connect4/user'

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883


def on_connect(client, userdata, flags, rc):
    mqtt_client.subscribe(topic_user)


def on_message(client, userdata, message):
    global NUMBER, TRACKER

    msg = message.payload.decode()
    print('Vision ha ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic):
        if msg == '-1':
            # Richiesta di numero
            TRACKER = True


class handTracker:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
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

    def positionFinder(self, image):
        lmlist = []
        if self.results.multi_hand_landmarks:
            for hand in self.results.multi_hand_landmarks:
                for id, lm in enumerate(hand.landmark):
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
    error = False

    while True:
        success, image = cap.read()

        if TRACKER:
            cv2.putText(image, "It's your turn, make a move!", (150, 30), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 0), 1)
            image = tracker.handsFinder(image)
            lmLists = tracker.positionFinder(image)

            if error:
                cv2.putText(image, f"({NUMBER}) is not a valid column! ", (165, 60), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                            (0, 0, 255), 1)

            if len(lmLists) != 0:
                fingers = []

                for i in range(0, len(lmLists), 21):  # Itera su tutti i landmark di entrambe le mani
                    lmList = lmLists[i:i + 21]  # Prendi i landmark di una mano

                    if lmList[tipIds[0]][1] < 320:  # Thumb dx
                        if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
                            fingers.append(1)
                        else:
                            fingers.append(0)
                    else:  # Thumb sx
                        if lmList[tipIds[0]][1] < lmList[tipIds[0] - 1][1]:
                            fingers.append(1)
                        else:
                            fingers.append(0)

                    # 4 Fingers
                    for id in range(1, 5):
                        if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                            fingers.append(1)
                        else:
                            fingers.append(0)

                totalFingers = fingers.count(1)

                if totalFingers <= 7:
                    error = False

                if totalFingers == target_value:
                    consecutive_same_count += 1
                    if consecutive_same_count >= 20:
                        print(f"Stesso valore ({totalFingers}) per 20 volte.")
                        NUMBER = totalFingers
                        if NUMBER < 1 or NUMBER > 7:
                            error = True
                        else:
                            mqtt_client.publish(f'connect4/user', NUMBER)
                            TRACKER = False
                            NUMBER = -1

                        consecutive_same_count = 0
                else:
                    consecutive_same_count = 0
                    target_value = totalFingers

                print(totalFingers)

        cv2.imshow("Video", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break


mqtt_client = mqtt.Client('Vision')
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# print("Connecting to " + broker + " port: " + str(broker_port))
mqtt_client.connect(broker, broker_port)

if __name__ == '__main__':
    t2 = threading.Thread(target=mqtt_client.loop_forever)
    t2.start()
    VisionTask()
