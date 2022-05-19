# CS461_Pacman
CS 461 Group 6 Project README File

Members:

Can Kırımca 21802271

Bora Ege Duygun 21801790

Güven Gergerli 21803393

Can Kırşallıoba 21801768

Burak Yiğit Uslu 21801745


Setup:

For our game we used Unity3D version 2018.4.36, since we specifically used this version; it is better to conduct your tests on this version.
You can use the following link to download Unity3D version 2018.4.36f1:

https://unity3d.com/unity/whats-new/2018.4.36

It is also adviced to download and setup UnityHub in order to use the Unity version 2018.4.36f1 and initialize the game.


IMPORTANT NOTE: The project in "main" branch is the adversarial search game (assignment 2) while the project in "search_algo" branch is the search game (assignment 1) for PacMan.

Initialization:

1. Clone the Github Code to you local computer.
2. Open UnityHub application, make sure the unity version is 2018.4.36f1
3. Open the project from the UnityHub by clicking the project (select as folder).

Run:

Assignment 1 (search_algo branch)

1. It contains only one scene (Assets/Scenes/SearchAlgoMap).
2. It contains only one script (Assets/Scripts/PacMan.cs).
3. There are 4 algorithms in total (BFS, DFS, A* and UCS).
4. Under Start() function, at line 323, one can call "bf_search()", "df_search()", "astar()", and "ucs()" functions to run the corresponding algorithm (where we define variable of "t1").
5. For "ucs()" function, one can determine the cost of going through walls under "getsuccessor_ucs()" function, at line 72 (NOT line 68), by changing the parameter "cost".
6. One can press the play button to see the game running (to terminate running, one has to press again).

IMPORTANT NOTE: The speed of PacMan animation might be too slow/fast depending on the processor. The normal speed in our computer can be seen in the videos where the link can be found below.

Assignment 2 (main branch)

1. It contains two scenes (Assets/Scenes/GhostMiniMap2, Assets/Scenes/MediumAdversarial)
2. It contains three scripts (Assets/Scripts/PacMan.cs, Assets/Scripts/pacman_large.cs, Assets/Scripts/pacman_mini.cs)
3. You should not change the PacMan.cs script, just let it stay as it is.
4. The "pacman_large.cs" code has the evaluation function for "MediumAdversarial" map, the "pacman_mini.cs" code has the evaluation function for "GhostMiniMap2" map.
5. IMPORTANT: The line 6 of the preferred script must be "public class pacman_mini : MonoBehaviour" while it should be different than "public class pacman_mini : MonoBehaviour" for the other script. FOR EXAMPLE: If you wish to run "pacman_large.cs" file, the line 6 of this script should be "public class pacman_mini : MonoBehaviour" and the other script at line 6 should be "public class pacman_mini_not : MonoBehaviour" (you can type anything instead of pacman_mini_not, just be sure that it is NOT pacman_mini too, otherwise you will get an error). It might be confusing to make the name "public class pacman_mini : MonoBehaviour" even though when we call the "pacman_large.cs" file but what matters is that for the script we want to call, its MonoBehaviour name at line 6 must be "public class pacman_mini : MonoBehaviour" while it should be something else for the other, that's all.
7. To run expectimax algorithm for the pacman, one should call "exp_pacmanNextAction()" function under the Start() function. (It is line 559 for "pacman_mini.cs" and line 651 for "pacman_large.cs", just change the function name, not the parameters of the function inside!). Similarly, to run alpha-beta pruning minimax, one should call "ab_pacmanNextAction()" under the Start() function (change the same line, i.e line 559 in "pacman_mini.cs" and line 651 in "pacman_large.cs").
8. One can press the play button to see the game running (to terminate running, one has to press again).

IMPORTANT NOTE: The speed of PacMan/Ghost animation might be too slow/fast depending on the processor. The normal speed in our computer can be seen in the videos where the link can be found below.
