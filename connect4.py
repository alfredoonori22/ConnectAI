import time
import numpy as np
import random
import math
import paho.mqtt.client as mqtt
import threading

ROWS = 6
COLS = 7

PLAYER = 0
AI = 1

EMPTY = 0
PLAYER_PIECE = 1
AI_PIECE = -1

NUMBER = -1
END = 1

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883

topic_user = 'connect4/user'
topic_robot = 'connect4/robot'
topic_username = 'connect4/username'
topic_outcome = 'connect4/outcome'
topic_turn = 'connect4/turn'
topic_end = 'connect4/end'


def on_connect(client, userdata, flags, rc):
    # print("Connected with result code " + str(rc))
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)
    mqtt_client.subscribe(topic_end)


def on_message(client, userdata, message):
    global NUMBER, END

    msg = message.payload.decode()
    print('Connect3 ha ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic):
        if msg != '-1':
            NUMBER = int(msg) - 1

    if mqtt.topic_matches_sub(topic_end, message.topic):
        END = 1


def create_board():
    board = np.zeros((ROWS, COLS))
    return board


def drop_piece(board, row, col, piece):
    board[row][col] = piece


def is_valid_location(board, col):
    return board[0][col] == 0


def get_next_open_row(board, col):
    for r in range(ROWS-1, -1, -1):
        if board[r][col] == 0:
            return r


def winning_move(board, piece):
    for c in range(COLS-3):
        for r in range(ROWS):
            if board[r][c] == piece and board[r][c+1] == piece and board[r][c+2] == piece and board[r][c+3] == piece:
                return True

    for c in range(COLS):
        for r in range(ROWS-3):
            if board[r][c] == piece and board[r+1][c] == piece and board[r+2][c] == piece and board[r+3][c] == piece:
                return True

    for c in range(COLS-3):
        for r in range(3, ROWS):
            if board[r][c] == piece and board[r-1][c+1] == piece and board[r-2][c+2] == piece and board[r-3][c+3] == piece:
                return True

    for c in range(3,COLS):
        for r in range(3, ROWS):
            if board[r][c] == piece and board[r-1][c-1] == piece and board[r-2][c-2] == piece and board[r-3][c-3] == piece:
                return True


def evaluate_window(window, piece):
    score = 0
    opp_piece = PLAYER_PIECE

    if piece == PLAYER_PIECE:
        opp_piece = AI_PIECE

    if window.count(piece) == 4:
        score += 1000
    elif window.count(piece) == 3 and window.count(0) == 1:
        score += 10
    elif window.count(piece) == 2 and window.count(0) == 2:
        score += 2

    if window.count(opp_piece) == 3 and window.count(0) == 1:
        score -= 4

    return score


def score_position(board, piece):
    score = 0

    # Score center column
    center_array = [int(i) for i in list(board[:, COLS // 2])]
    center_count = center_array.count(piece)
    score += center_count * 6

    # Score Horizontal
    for r in range(ROWS):
        row_array = [int(i) for i in list(board[r, :])]
        for c in range(COLS - 3):
            window = row_array[c:c + 4]
            score += evaluate_window(window, piece)

    # Score Vertical
    for c in range(COLS):
        col_array = [int(i) for i in list(board[:, c])]
        for r in range(ROWS - 3):
            window = col_array[r:r + 4]
            score += evaluate_window(window, piece)

    # Score positive sloped diagonal
    for r in range(3, ROWS):
        for c in range(COLS - 3):
            window = [board[r - i][c + i] for i in range(4)]
            score += evaluate_window(window, piece)

    for r in range(3, ROWS):
        for c in range(3, COLS):
            window = [board[r - i][c - i] for i in range(4)]
            score += evaluate_window(window, piece)

    return score


def is_terminal_node(board):
    return winning_move(board, PLAYER_PIECE) or winning_move(board, AI_PIECE) or len(get_valid_locations(board)) == 0


def minimax(board, depth, alpha, beta, maximizingPlayer):
    valid_locations = get_valid_locations(board)
    is_terminal = is_terminal_node(board)

    if depth == 0 or is_terminal:
        if is_terminal:
            if winning_move(board, AI_PIECE):
                return None, 100000000000000
            elif winning_move(board, PLAYER_PIECE):
                return None, -10000000000000
            else:  # Game is over, no more valid moves
                return None, 0
        else:  # Depth is zero
            return None, score_position(board, AI_PIECE)

    if maximizingPlayer:
        value = -math.inf
        column = random.choice(valid_locations)
        for col in valid_locations:
            row = get_next_open_row(board, col)
            b_copy = board.copy()
            drop_piece(b_copy, row, col, AI_PIECE)
            _, new_score = minimax(b_copy, depth - 1, alpha, beta, False)

            if new_score > value:
                value = new_score
                column = col
            alpha = max(alpha, value)

            if alpha >= beta:
                break
        return column, value

    else:  # Minimizing player
        value = math.inf
        column = random.choice(valid_locations)
        for col in valid_locations:
            row = get_next_open_row(board, col)
            b_copy = board.copy()
            drop_piece(b_copy, row, col, PLAYER_PIECE)
            new_score = minimax(b_copy, depth - 1, alpha, beta, True)[1]
            if new_score < value:
                value = new_score
                column = col
            beta = min(beta, value)
            if alpha >= beta:
                break
        return column, value


def get_valid_locations(board):
    valid_locations = []
    for col in range(COLS):
        if is_valid_location(board, col):
            valid_locations.append(col)
    return valid_locations


def start_game(camera_mode):
    global NUMBER, END

    board = create_board()

    game_over = False

    turn = random.randint(PLAYER, AI)

    while not game_over:
        # Ask for Person Player Input
        time.sleep(1)
        while not END:
            continue
        END = 0

        mqtt_client.publish(topic_turn, turn)

        if turn == PLAYER:
            print('Tuo turno')
            if camera_mode:
                mqtt_client.publish(topic_user, -1)

            time.sleep(1)
            while NUMBER == -1:
                continue

            col = NUMBER
            NUMBER = -1

            if is_valid_location(board, col):
                row = get_next_open_row(board, col)
                drop_piece(board, row, col, PLAYER_PIECE)

                if winning_move(board, PLAYER_PIECE):
                    print("Person Player wins!")
                    mqtt_client.publish(topic_outcome, -1)
                    game_over = True

                turn += 1
                turn = turn % 2

                print(board)

        # Ask for Player AI Input
        elif turn == AI and not game_over:
            col, minimax_score = minimax(board, 5, -math.inf, math.inf, True)

            if is_valid_location(board, col):
                row = get_next_open_row(board, col)
                drop_piece(board, row, col, AI_PIECE)

                time.sleep(1)
                mqtt_client.publish('connect4/robot', col+1)
                if winning_move(board, AI_PIECE):
                    print("Player AI wins!")
                    mqtt_client.publish(topic_outcome, 1)
                    game_over = True

                print(board)

                turn += 1
                turn = turn % 2

        if len(get_valid_locations(board)) == 0:
            print("Draw!")
            mqtt_client.publish(topic_outcome, 0)
            game_over = True


mqtt_client = mqtt.Client('Connect3')
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# print("Connecting to " + broker + " port: " + str(broker_port))
mqtt_client.connect(broker, broker_port)

t2 = threading.Thread(target=mqtt_client.loop_forever)
t2.start()
