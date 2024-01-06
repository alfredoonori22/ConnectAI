# ConnectAI
Connect 4 is a classic two-player strategy game, where the goal is to be the first to form a line of four of your colored discs either in a horizontal, vertical or diagonal way on the game board.

This project aims to develop an interactive simulation of a robotic arm playing Connect 4, where the user, interacting through a Telegram bot, competes against an AI driven by a min-max algorithm.

<p align="center">
  <img width="1080" height="360" src="https://github.com/alfredoonori22/ConnectAI/assets/62024453/02058a92-f3bb-40a7-a3d5-44a6b58bb398">
</p>

## Robotic Implementation
The chosen manipulator is an UR10, with a ROBOTIQ85 gripper attached to its flange, simulated on CoppeliaSim, a powerful simulation software.
The other elements present in the CoppeliaSim scene are: 
- the handmade cubes used as playing pieces for both the player
- two rotation station used by the manipulator while making the moves

<p align="center">
  <img width="580" height="580" src="https://github.com/alfredoonori22/ConnectAI/assets/62024453/5cff2cbf-9961-4074-9e2d-0ee3422863c4">
</p>

## Min-max Algorithm
Utilizing a recursive algorithm, the system evaluates the best move for a given game state. This approach makes use of the Min-max Algorithm, enabling efficient exploration of potential game states.  

<p align="center">
  <img width="540" height="660" src="https://github.com/alfredoonori22/ConnectAI/assets/62024453/9934e409-8932-4a11-9101-4ddfed1291c8">
</p>

## Telegram Bot
A telegram bot is used as user interface, providing the following functionalities:
- Selection of the game mode, indeed the move selection (i.e. the column in which to rotate the cube) can be done either using the keyboard or by showing the column number with fingers to the camera
- Real-time game progress visualization

<p align="center">
  <img width="380" height="620" src="https://github.com/alfredoonori22/ConnectAI/assets/62024453/49888eca-f499-4cae-bc18-e5fe4c94a1f1">
</p>
