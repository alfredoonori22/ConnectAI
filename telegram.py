from flask import Flask
from flask import request
from flask import Response
import requests
import paho.mqtt.client as mqtt
import threading

TOKEN = "6463703134:AAEpae9NaLJQQ7JB6s6029aBESS_q1mZ4b4"
# https://api.telegram.org/bot6463703134:AAEpae9NaLJQQ7JB6s6029aBESS_q1mZ4b4/setWebhook?url=https://6d54-109-113-99-13.ngrok-free.app
# ngrok http 5000

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883

app = Flask(__name__)

global chat_id
txt = ''
state = 0
grid = [[0,0,0,0,0],
        [0,0,0,0,0],
        [0,0,0,0,0],
        [0,0,0,0,0]]
username = ""
message_id = 0
counter = 0

'''
stato 0 = Start del bot
stato 1 = Start del match
stato 2 = Inserimento username
stato 3 = Conferma username
stato 4 = Messaggio di inizio sfida e riproduzione del match
stato 5 = Arriva il messaggio con il risultato -> Vittoria, pareggio, sconfitta
stato 6 = Rivincita/Esci
'''


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)
    mqtt_client.subscribe(topic_outcome)


def on_message(client, userdata, message):
    global grid, counter, username

    msg = message.payload.decode()
    print('Ho ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic):
        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        if msg != '-1':
            for row in grid[::-1]:
                if row[int(msg)-1] == 0:
                    row[int(msg)-1] = 1
                    break

            grid_mes = ""
            for row in grid:
                for col in row:
                    if col == 0:
                        grid_mes += "⭕\t "
                    elif col == 1:
                        grid_mes += "🔴\t "
                    elif col == -1:
                        grid_mes += "🔵\t "
                grid_mes += "\n\n"

            tel_del_message(message_id + counter + 1)
            tel_del_message(message_id + counter)
            counter += 2
            tel_send_message(grid_mes)
            tel_send_message(f"{username} ha scelto la colonna {int(msg)}")

    if mqtt.topic_matches_sub(topic_robot, message.topic):
        # msg è il numero della colonna in cui deve scendere il dischetto dell'user
        for row in grid[::-1]:
            if row[int(msg)] == 0:
                row[int(msg)] = -1
                break

        grid_mes = ""
        for row in grid:
            for col in row:
                if col == 0:
                    grid_mes += "⭕\t "
                elif col == 1:
                    grid_mes += "🔴\t "
                elif col == -1:
                    grid_mes += "🔵\t "
            grid_mes += "\n\n"

        tel_del_message(message_id + counter + 1)
        tel_del_message(message_id + counter)
        counter += 2
        tel_send_message(grid_mes)
        tel_send_message(f"ConnectAI ha scelto la colonna {int(msg) + 1}")

    if mqtt.topic_matches_sub(topic_outcome, message.topic):
        if msg == '-1':
            tel_send_message(f"{username} vince!")
        if msg == '1':
            tel_send_message(f"ConnectAI vince!")
        if msg == '0':
            tel_send_message(f"Pareggio!")


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
        'text': f"Vuoi giocare contro ConnectAI?",
        'reply_markup': {
            "inline_keyboard": [[
                {
                    "text": "Si",
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
        'text': f"\"{msg}\" è corretto?",
        'reply_markup': {
            "inline_keyboard": [[
                {
                    "text": "Si",
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
            tel_send_message("Benvenuto! Le regole sono pazze ciao."
                             "\nQuando è il tuo turno seleziona con le dita la colonna in cui vuoi mettere le palle (la pedina scusa)"
                             "\nBuona fortuna looser!")

            tel_send_startbutton()
            state = 1

        elif state == 1:
            start = tel_parse_button(msg)

            if start == "yes":
                tel_send_message("Inserisci l'username che vuoi usare nella sfida:")
                state = 2

            if start == "no":
                tel_send_message("Okay, alla prossima :(")
                state = 0

        elif state == 2:
            username = tel_parse_message(msg)[1]
            message_id = msg['message']['message_id'] + 4

            tel_send_confirmbutton(username)
            state = 3

        elif state == 3:
            confirm = tel_parse_button(msg)

            if confirm == "yes":
                tel_send_message("Perfetto, iniziamo!")
                tel_send_message("La situazione della griglia è la seguente:")

                tel_send_message("⭕\t ⭕\t ⭕\t ⭕\t ⭕\n\n"
                                 "⭕\t ⭕\t ⭕\t ⭕\t ⭕\n\n"
                                 "⭕\t ⭕\t ⭕\t ⭕\t ⭕\n\n"
                                 "⭕\t ⭕\t ⭕\t ⭕\t ⭕\n\n")

                tel_send_message(f"Iniziamo!")

                state = 4
            if confirm == "no":
                tel_send_message("Reinseriscilo scemo:")
                state = 2

        return Response('ok', status=200)
    else:
        return "<h1><h1>"


topic_user = 'connect4/user'
topic_robot = 'connect4/robot'
topic_username = 'connect4/username'
topic_outcome = 'connect4/outcome'

mqtt_client = mqtt.Client('Telegram')
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

print("Connecting to " + broker + " port: " + str(broker_port))
mqtt_client.connect(broker, broker_port)

t1 = threading.Thread()
t1.start()
t2 = threading.Thread(target=mqtt_client.loop_forever)
t2.start()

if __name__ == '__main__':
    app.run(threaded=True)