# RosProjet
InROS platform to realize the robot patrol walking function and realize the robot's obstacle avoidance function

Projet ROS

SAR
Valentin COINTREL 21107824
Hanlin LIU 3971558

Dans la dossier launch, vous pouvez trouver: 

1, 'challenge1.launch' qui lance le noeud 'challenge1.py' et fait apparaître le robot au début de la piste.
   Ce fichier a servit à réaliser le challenge1.
   
2, 'challenge2.launch' qui lance le noeud 'challenge2.py' et fait apparaître le robot au début du couloir.
   Ce fichier a servit à réaliser le challenge2.
   
3, 'challenge3.launch' qui lance le noeud 'challenge3.py' et fait apparaître le robot au début du "cluttered environment" (environnement sans lumières avec obstacles).
   Ce fichier a servit à réaliser le challenge3.
   
4, 'challenge_ensemble.launch' qui lance le noeud 'challenge_ensemble.py' et fait apparaître le robot au début de la piste.
   Ce fichier a servit à réaliser tous les challenges et connecter les challenges entre eux.

Afin de lancher ces fichier: 

1, roslaunch projet2022 challenge1.launch
2, roslaunch projet2022 challenge2.launch
3, roslaunch projet2022 challenge3.launch
4, roslaunch projet2022 challenge_ensemble.launch

Dans la dossier scripts, vous pouvez trouver les fichiers .py qui contiennent les codes pour les trois challenges:

1, 'challenge1.py' contient le code de challenge 1.
2, 'challenge2.py' contient le code de challenge 2.
3, 'challenge3.py' contient le code de challenge 3.
4, 'challenge_ensemble.py' contient le code de tous les challenges et le code pour passer d'un challenge à un autre.
