import time
from flask import Flask
from flask import request
from flask import Response
import requests
import paho.mqtt.client as mqtt
import threading
from coppeliasim.moveblock import moveBlockFunc
import coppeliasim.globalvariables as g
from connect4 import start_game
from vision import VisionTask

TOKEN = "6463703134:AAFsoS-0TWPMOxq4ZEK8OG-p48D8cbaZa0w"
# https://api.telegram.org/bot6463703134:AAFsoS-0TWPMOxq4ZEK8OG-p48D8cbaZa0w/setWebhook?url=https://21a6-2001-b07-6442-aa2f-1800-82e6-397c-61d6.ngrok.io
# ngrok http 5000

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883
topic_user = 'connect4/user'
topic_robot = 'connect4/robot'
topic_outcome = 'connect4/outcome'
topic_turn = 'connect4/turn'

app = Flask(__name__)

global chat_id
txt = ''
username = ''

turn = -1
state = 0
message_id = 0
counter = 0
camera_mode = True   # True camera, False keyboard
NUMBER = -1
ERROR = False

grid = [[0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0]]
diz = {0: 'A',
       1: 'B',
       2: 'C',
       3: 'D',
       4: 'E',
       5: 'F'}
diz_numbers = {1: '1️⃣',
               2: '2️⃣',
               3: '3️⃣',
               4: '4️⃣',
               5: '5️⃣',
               6: '6️⃣',
               7: '7️⃣'}

'''
stato 0 = Start del bot
stato 1 = Start del match
stato 2 = Inserimento username
stato 3 = Conferma username
stato 4 = Messaggio di inizio sfida e riproduzione del match
stato 5 = Arriva il messaggio con il risultato -> Vittoria, pareggio, sconfitta
stato 6 = Rivincita/Esci
'''


def sendGrid(turn):
    global grid, counter, camera_mode, ERROR

    grid_mes = ''
    for row in grid:
        for col in row:
            if col == 0:
                grid_mes += "⚪\t "
            elif col == 1:
                grid_mes += "🔵\t "
            elif col == -1:
                grid_mes += "🟠\t "
        grid_mes += "\n\n"

    grid_mes += "1️⃣\t 2️⃣\t 3️⃣\t 4️⃣\t 5️⃣\t 6️⃣\t 7️⃣\n\n"

    if ERROR:
        tel_del_message(message_id + counter + 5)
        tel_del_message(message_id + counter + 4)

    if not camera_mode and turn:
        tel_del_message(message_id + counter + 3)

    tel_del_message(message_id + counter + 2)
    tel_del_message(message_id + counter + 1)
    tel_del_message(message_id + counter)

    if ERROR:
        counter += 2
        ERROR = False

    if not camera_mode and turn:
        counter += 1
    counter += 3
    tel_send_message(grid_mes)


def on_connect():
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)
    mqtt_client.subscribe(topic_outcome)
    mqtt_client.subscribe(topic_turn)


def on_message(message):
    global grid, counter, username, turn, state

    msg = message.payload.decode()
    print('Telegram ha ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic) and msg != '-1':
        c_row = 0
        c_col = int(msg) - 1

        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        for row in grid[::-1]:
            if row[int(msg)-1] == 0:
                row[int(msg)-1] = 1
                break

            c_row = c_row + 1

        sendGrid(True)
        tel_send_message(f"{username} selected column {diz_numbers[int(msg)]}")
        moveBlockFunc(g.clientID, f'{diz[c_row]}{c_col}', g.position[c_row][c_col], 'blue')

    if mqtt.topic_matches_sub(topic_robot, message.topic):
        c_row = 0
        c_col = int(msg) - 1

        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        for row in grid[::-1]:
            if row[int(msg)-1] == 0:
                row[int(msg)-1] = -1
                break

            c_row = c_row + 1

        sendGrid(False)
        tel_send_message(f"ConnectAI selected column {diz_numbers[int(msg)]}")
        moveBlockFunc(g.clientID, f'{diz[c_row]}{c_col}', g.position[c_row][c_col], 'orange')

    if mqtt.topic_matches_sub(topic_turn, message.topic):
        turn = int(msg)

        if turn == 0:
            if camera_mode:
                tel_send_message('It\'s your turn, use your fingers to choose the column 🖐🏻')
            else:
                tel_send_message('It\'s your turn, send a number to choose the column ✍🏻')
        elif turn == 1:
            tel_send_message('It\'s ConnectAI turn ⚙️')
            time.sleep(3)

    if mqtt.topic_matches_sub(topic_outcome, message.topic):
        result = int(msg)

        if result == -1:
            tel_send_message(f"{username} won the game! Congratulation! 🎉")
        elif result == 1:
            tel_send_message(f"ConnectAI won the game! 🦾")
        elif result == 0:
            tel_send_message("It's a draw! 🤝🫱🏻🫱🏻‍")
        tel_send_message("See you next time! 👋🏻")
        state = 6


def tel_parse_message(message):
    global chat_id, txt

    print("message-->", message)
    try:
        chat_id = message['message']['chat']['id']
        txt = message['message']['text']
        print("chat_id-->", chat_id)
        print("txt-->", txt)

        return chat_id, txt
    except:
        print("NO text found-->>")


def tel_parse_button(message):
    try:
        bottone = message['callback_query']['data']
        print("button_pressed-->", bottone)
        return bottone
    except:
        print("NO button found-->>")


def tel_send_startbutton():
    global chat_id

    url = f'https://api.telegram.org/bot{TOKEN}/sendMessage'

    payload = {
        'chat_id': chat_id,
        'text': f"Do you want to play against ConnectAI? 🤖",
        'reply_markup': {
            "inline_keyboard": [[
                {
                    "text": "Yes, with keyboard ⌨️",
                    "callback_data": 'yes_k'
                }],
                [{
                    "text": "Yes, with camera 📷",
                    "callback_data": 'yes_c'
                }],
                [{
                    "text": "No ✖️",
                    "callback_data": "no"
                }]
            ]
        }
    }

    r = requests.post(url, json=payload)

    return r


def tel_send_confirmbutton(msg):
    global chat_id

    url = f'https://api.telegram.org/bot{TOKEN}/sendMessage'

    payload = {
        'chat_id': chat_id,
        'text': f"Is '{msg}' correct?",
        'reply_markup': {
            "inline_keyboard": [[
                {
                    "text": "Yes",
                    "callback_data": 'yes'
                },
                {
                    "text": "No",
                    "callback_data": "no"
                }]
            ]
        }
    }

    r = requests.post(url, json=payload)

    return r


def tel_send_message(text):
    global chat_id
    url = f'https://api.telegram.org/bot{TOKEN}/sendMessage'
    payload = {
        'chat_id': chat_id,
        'text': text
    }

    r = requests.post(url, json=payload)

    return r


def tel_del_message(del_id):
    global chat_id
    url = f'https://api.telegram.org/bot{TOKEN}/deleteMessage'
    payload = {
        'chat_id': chat_id,
        'message_id': del_id
    }

    r = requests.post(url, json=payload)

    return r


def is_number(txt):
    try:
        value = int(txt)
        if 1 <= value <= 7:
            return True
        else:
            return False
    except ValueError:
        return False


@app.route('/', methods=['GET', 'POST'])
def index():
    global state, chat_id, message_id, txt, username, camera_mode, counter, ERROR

    if request.method == 'POST':
        msg = request.get_json()

        if state == 0:
            chat_id, txt = tel_parse_message(msg)
            tel_send_message("Welcome to ConnectAI Bot! 🟠🔵")
            tel_send_message("Align four of your pieces either horizontally, vertically, or diagonally."
                             "\nBe strategic and watch out for your opponent! 😉")
            tel_send_message("There are two different modes to make your move in this game:"
                             "\n 📷 -> Show the column number by holding up your fingers in front of the webcam"
                             "\n ⌨️ -> Send a message with the column number"
                             "\nThe robot will move the piece for you! 🦾"
                             "\nGood luck! 🍀")
            tel_send_startbutton()
            state = 1

        elif state == 1:
            start = tel_parse_button(msg)

            if start == "yes_c":
                tel_send_message("Ok, you will select your moves using the camera ✌️🏻.\nPlease enter your username:")
                # t_camera = threading.Thread(target=VisionTask)
                # t_camera.start()
                camera_mode = True
                state = 2

            if start == "yes_k":
                tel_send_message("Ok, you will select your moves using the keyboard ⌨️.\nPlease enter your username:")
                camera_mode = False
                state = 2

            if start == "no":
                tel_send_message("Ok, see you! 👋🏻")
                state = 0

        elif state == 2:
            username = tel_parse_message(msg)[1]
            message_id = msg['message']['message_id'] + 4

            tel_send_confirmbutton(username)
            state = 3

        elif state == 3:
            confirm = tel_parse_button(msg)

            if confirm == "yes":
                tel_send_message("Let's start! 🤓"
                                 f"\n🟠: ConnectAI    🔵: {username}")
                tel_send_message("Here's the current grid status:")
                tel_send_message("⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t ⚪\t\n\n"
                                 "1️⃣\t 2️⃣\t 3️⃣\t 4️⃣\t 5️⃣\t 6️⃣\t 7️⃣\t\n\n")

                t_connect3 = threading.Thread(target=start_game, args=(camera_mode, ))
                t_connect3.start()
                time.sleep(1)

                tel_send_message('Drawing lots... 🎲')
                time.sleep(1)

                if not camera_mode:
                    state = 4

            elif confirm == "no":
                tel_send_message("Insert your username again:")
                state = 2

        elif state == 4:
            chat_id, txt = tel_parse_message(msg)
            check = is_number(txt)

            if not check:
                tel_send_message('Input value is not valid, please send a number between 1 and 7.')
                ERROR = True
            else:
                mqtt_client.publish(topic_user, txt)

        return Response('ok', status=200)
    else:
        return "<h1><h1>"


def startTelegram():
    app.run(threaded=True)


mqtt_client = mqtt.Client('Telegram')
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(broker, broker_port)

t2 = threading.Thread(target=mqtt_client.loop_forever)
t2.start()
