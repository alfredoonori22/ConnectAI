import time
from flask import Flask
from flask import request
from flask import Response
import requests
import paho.mqtt.client as mqtt
import threading
from coppeliasim.moveblock import moveBlockFunc
import coppeliasim.globalvariables as g
from connect3 import start_game

TOKEN = "6463703134:AAFsoS-0TWPMOxq4ZEK8OG-p48D8cbaZa0w"
# https://api.telegram.org/bot6463703134:AAFsoS-0TWPMOxq4ZEK8OG-p48D8cbaZa0w/setWebhook?url=https://2aee-2001-b07-6442-aa2f-adf2-4f76-10bf-8f07.ngrok.io
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

grid = [[0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]]
diz = {0: 'A',
       1: 'B',
       2: 'C',
       3: 'D'}
diz_fingers={ 1: '1️⃣',
              2: '2️⃣',
              3: '3️⃣',
              4: '4️⃣',
              5: '5️⃣'}

'''
stato 0 = Start del bot
stato 1 = Start del match
stato 2 = Inserimento username
stato 3 = Conferma username
stato 4 = Messaggio di inizio sfida e riproduzione del match
stato 5 = Arriva il messaggio con il risultato -> Vittoria, pareggio, sconfitta
stato 6 = Rivincita/Esci
'''


def sendGrid():
    global grid, counter

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

    #tel_del_message(message_id + counter + 1)
    #tel_del_message(message_id + counter)
    counter += 2
    tel_send_message(grid_mes)


def on_connect(client, userdata, flags, rc):
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)
    mqtt_client.subscribe(topic_outcome)
    mqtt_client.subscribe(topic_turn)


def on_message(client, userdata, message):
    global grid, counter, username, turn

    msg = message.payload.decode()
    print('Ho ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic) and msg != '-1':
        c_row = 0
        c_col = int(msg) - 1

        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        for row in grid[::-1]:
            if row[int(msg)-1] == 0:
                row[int(msg)-1] = 1
                break

            c_row = c_row + 1

        moveBlockFunc(g.clientID, f'{diz[c_row]}{c_col}', g.position[c_row][c_col], 'blue')
        sendGrid()
        tel_send_message(f"{username} selected column {diz_fingers[int(msg)]}")

    if mqtt.topic_matches_sub(topic_robot, message.topic):
        c_row = 0
        c_col = int(msg) - 1

        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        for row in grid[::-1]:
            if row[int(msg)-1] == 0:
                row[int(msg)-1] = -1
                break

            c_row = c_row + 1

        moveBlockFunc(g.clientID, f'{diz[c_row]}{c_col}', g.position[c_row][c_col], 'orange')
        sendGrid()
        tel_send_message(f"ConnectAI selected column {diz_fingers[int(msg)]}")

    if mqtt.topic_matches_sub(topic_turn, message.topic):
        turn = int(msg)


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


@app.route('/', methods=['GET', 'POST'])
def index():
    global state, chat_id, message_id, txt, username

    if request.method == 'POST':
        msg = request.get_json()

        if state == 0:
            chat_id, txt = tel_parse_message(msg)
            tel_send_message("Welcome to Connect 3 Bot! 🟠🔵")
            tel_send_message("Align three of your pieces either horizontally, vertically, or diagonally."
                             "\nBe strategic and watch out for your opponent! 😉")
            tel_send_message("To make your move, indicate the column number by showing your fingers in front of the webcam when it's your turn."
                             "\nThe robot will move the piece for you! 🦾"
                             "\nGood luck! 🍀")
            tel_send_startbutton()
            state = 1

        elif state == 1:
            start = tel_parse_button(msg)

            if start == "yes":
                tel_send_message("Please enter your username:")
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
                tel_send_message("⚪\t ⚪\t ⚪\t ⚪\t ⚪\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\n\n"
                                 "⚪\t ⚪\t ⚪\t ⚪\t ⚪\n\n")

                t_connect3 = threading.Thread(target=start_game)
                t_connect3.start()
                time.sleep(1)

                if turn == 0:
                    tel_send_message('It\'s your turn, use your fingers to choose the column 🖐🏻')
                elif turn == 1:
                    tel_send_message('It\'s ConnectAI turn ⚙️')

                state = 4

            if confirm == "no":
                tel_send_message("Insert your username again:")
                state = 2

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
