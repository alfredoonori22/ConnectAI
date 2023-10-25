import time

import numpy as np
import random
import sys
import math
import paho.mqtt.client as mqtt
import threading

ROW_COUNT = 4
COLUMN_COUNT = 5

PLAYER = 0
AI = 1

EMPTY = 0
PLAYER_PIECE = 1
AI_PIECE = -1

WINDOW_LENGTH = 3
NUMBER = -1

# MQTT
broker = 'broker.emqx.io'
broker_port = 1883

topic_user = 'connect4/user'
topic_robot = 'connect4/robot'
topic_username = 'connect4/username'
topic_outcome = 'connect4/outcome'
topic_turn = 'connect4/turn'


def on_connect(client, userdata, flags, rc):
    # print("Connected with result code " + str(rc))
    mqtt_client.subscribe(topic_user)
    mqtt_client.subscribe(topic_robot)


def on_message(client, userdata, message):
    global NUMBER

    msg = message.payload.decode()
    print('Ho ricevuto il messaggio: ' + msg + ' dal topic ' + message.topic)

    if mqtt.topic_matches_sub(topic_user, message.topic):
        if msg != '-1':
            NUMBER = int(msg) - 1


def create_board():
    board = np.zeros((ROW_COUNT, COLUMN_COUNT))
    return board


def drop_piece(board, row, col, piece):
    board[row][col] = piece


def is_valid_location(board, col):
    return board[ROW_COUNT - 1][col] == 0


def get_next_open_row(board, col):
    for r in range(ROW_COUNT):
        if board[r][col] == 0:
            return r


def print_board(board):
    print(np.flip(board, 0))


def winning_move(board, piece):
    # Check horizontal locations for win
    for c in range(COLUMN_COUNT - 2):
        for r in range(ROW_COUNT):
            if board[r][c] == piece and board[r][c + 1] == piece and board[r][c + 2] == piece:
                return True

    # Check vertical locations for win
    for c in range(COLUMN_COUNT):
        for r in range(ROW_COUNT - 2):
            if board[r][c] == piece and board[r + 1][c] == piece and board[r + 2][c] == piece:
                return True

    # Check positively sloped diaganols
    for c in range(COLUMN_COUNT - 2):
        for r in range(ROW_COUNT - 2):
            if board[r][c] == piece and board[r + 1][c + 1] == piece and board[r + 2][c + 2] == piece:
                return True

    # Check negatively sloped diaganols
    for c in range(COLUMN_COUNT - 2):
        for r in range(2, ROW_COUNT):
            if board[r][c] == piece and board[r - 1][c + 1] == piece and board[r - 2][c + 2] == piece:
                return True


def evaluate_window(window, piece):
    score = 0
    opp_piece = PLAYER_PIECE
    if piece == PLAYER_PIECE:
        opp_piece = AI_PIECE

    if window.count(piece) == 3:
        score += 100
    elif window.count(piece) == 2 and window.count(EMPTY) == 1:
        score += 5

    if window.count(opp_piece) == 2 and window.count(EMPTY) == 1:
        score -= 4

    return score


def score_position(board, piece):
    score = 0

    # Score center column
    center_array = [int(i) for i in list(board[:, COLUMN_COUNT // 2])]
    center_count = center_array.count(piece)
    score += center_count * 3

    # Score Horizontal
    for r in range(ROW_COUNT):
        row_array = [int(i) for i in list(board[r, :])]
        for c in range(COLUMN_COUNT - 2):
            window = row_array[c:c + WINDOW_LENGTH]
            score += evaluate_window(window, piece)

    # Score Vertical
    for c in range(COLUMN_COUNT):
        col_array = [int(i) for i in list(board[:, c])]
        for r in range(ROW_COUNT - 2):
            window = col_array[r:r + WINDOW_LENGTH]
            score += evaluate_window(window, piece)

    # Score positive sloped diagonal
    for r in range(ROW_COUNT - 2):
        for c in range(COLUMN_COUNT - 2):
            window = [board[r + i][c + i] for i in range(WINDOW_LENGTH)]
            score += evaluate_window(window, piece)

    for r in range(ROW_COUNT - 2):
        for c in range(COLUMN_COUNT - 2):
            window = [board[r + 2 - i][c + i] for i in range(WINDOW_LENGTH)]
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
            new_score = minimax(b_copy, depth - 1, alpha, beta, False)[1]

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
    for col in range(COLUMN_COUNT):
        if is_valid_location(board, col):
            valid_locations.append(col)
    return valid_locations


def pick_best_move(board, piece):
    valid_locations = get_valid_locations(board)
    best_score = -10000
    best_col = random.choice(valid_locations)
    for col in valid_locations:
        row = get_next_open_row(board, col)
        temp_board = board.copy()
        drop_piece(temp_board, row, col, piece)
        score = score_position(temp_board, piece)
        if score > best_score:
            best_score = score
            best_col = col

    return best_col


def start_game():
    global NUMBER

    board = create_board()
    game_over = False
    turn = random.randint(PLAYER, AI)

    mqtt_client.publish(topic_turn, turn)

    while not game_over:
        # Ask for Person Player Input
        time.sleep(2)

        if turn == PLAYER:
            print('Tuo turno')
            mqtt_client.publish(f'connect4/user', -1)
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

                print_board(board)

        # Ask for Player AI Input
        if turn == AI and not game_over:
            time.sleep(5)
            col, minimax_score = minimax(board, 5, -math.inf, math.inf, True)

            if is_valid_location(board, col):
                row = get_next_open_row(board, col)
                drop_piece(board, row, col, AI_PIECE)

                mqtt_client.publish('connect4/robot', col+1)
                if winning_move(board, AI_PIECE):
                    print("Player AI wins!")
                    mqtt_client.publish(topic_outcome, 1)
                    game_over = True

                print_board(board)

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
