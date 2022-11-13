#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#---------------Source / Inspiration---------------#
#NB: Les fonctions openCV et leur utilisation ont été inspiré du code fourni en annexe du
    #PDF: "ROS and experimental robotics Part 3: project"
    #L'idée de mettre à 0 les pixels du masque dans la zone du masque que l'on souhaite
    # négligé vient du code réalisé par "arjunskumar" que l'on peut retrouver via
    # ce lien https://github.com/arjunskumar/Line-Follower--ROS/blob/master/follower_ros.py
    #L'utilisation de -float(error)/(une constante) pour controller l'erreur est également inspirée
    #de ce code
#--------------------------------------------------#

#-----------------------------------------------------------------------#
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#-----------------------------------------------------------------------#

#-----------------------------------------------------------------------#
# Initialise un noeud qui s'appelle 'challenge' pour publier les informations de vitesse.
rospy.init_node('challenge', anonymous = True)

# Défini le rate/fréquence d'envoi des messages
rate = rospy.Rate(300) #le rate a une influence sur les performances du robot et dépend de l'ordinateur utilisé !!!!!!!!!!!! (il faut donc peut etre modifier cette valeur sur un autre ordinateur)
#le fonctionnement du robot dépend du couple de valeurs "rate" et "vitesse angulaire défini dans la fonction tourne()" (il faut donc peut etre modifier ces valeurs sur un autre ordinateur)
#(la valeur indiqué ici 300 a été obtenu à partir de plusieur essais sur Gazebo en modifiant également la vitesse angulaire)

#le fonctionnement du robot dépend du rate qui lui même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)
#dans le cas de l'evitemment d'obstacle sur la partie correspondant au challenge1, le fonctionnement du robot dépend du couple de valeurs rate et angular.z (cf fonction tourne() )
#qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)

#-----------------------------------------------------------------------#

#-------------------On déclare les variables globales-------------------#
global obstacle  # Variable globale qui détermine si nous approchons d'un obstacle
                 # obstacle = True: le robot rencontre un obstacle
                 # obstacle = False: pas d'obstacle, on avance.
global linear_x  # linear speed / vitesse linéaire/de translation
global angular_z # angular speed / vitesse de rotation
global threshold # Distance à partir de laquelle on considére une surface comme un obstacle
                 # au dessus de treshold les surfaces détéctées ne sont pas considérer comme des obstacles
                 # en dessous de treshold, on est trop proche donc les surfaces détéctées sont considérées comme des obstacles
global direction # Variable globale qui représente la position de l'obstacle
                 # direction = False: L'obstacle est situé à droite du robot, il faut tourner vers la gauche
                 # direction = True: L'obstacle est situé à gauche du robot, il faut tourner vers la droite
# On déclare trois global variable 'front, left, right' qui représente l'existance d'un obstalce dans ces trois plages.
# par exemple, si il y a un obstacle dans l'intervalle front, 'front' = true, sinon, 'front' = False.
global front 
global left  
global right
global redLine  # Variable globale qui est un tableau[0,0] qui représente si l'on peut détecter la ligne rouge
                # la détection du nombre de ligne rouge est compliquée parce que la détection de ligne rouge est un processus 
                # plutôt qu'un instant ou une simple valeur, on ne peut pas simplement dire : "quand on détecte une ligne rouge, le nombre +1"
                # donc on utilise un tableau pour simplifier la détection de ligne rouge.
                # Il est utiliser dans la fonction 'callback_follow', vous pouvez trouver l'explication en détail dans cette fonction.
global nombre_red  # Variable globale qui compte le nombre de ligne rouge détecté
               # En fonction de ce nombre, on peut passer un challenge à un autre
               # nombre_red  = 3: on entre challenge 2, nombre_red = 5, on entre challenge 3
               # Pour les autres valeurs du nombre_red, on est toujours dans challenge 1 
#-----------------------------------------------------------------------#
                 
#-------------------initialisation des valeurs-------------------#
linear_x = 0.1
angular_z = 0.35
#threshold = rospy.get_param("threshold") # récupérer l'information du threshold
threshold = 0.3
obstacle = False # il y a pas d'obstacle au départ
direction = False  
front = False
left = False
right = False
redLine = [0,0] # La valeur initiale de 'redLine' est un tableau
nombre_red = 0  # la valeur initiale est 0 (aucune ligne rouge détecté au départ)
#----------------------------------------------------------------#


#----------------------------------------------------------------#
# on définit l'intervalle des couleurs que l'on veut détecter avec la représentation HSV
#1ère valeur de la matrice = H (Hue = teinte) #2ème valeur = S (Saturation) #3ème valeur = V (Value = luminosité)
lower_red = np.array([160,20,70]) #bornes inférieur de la plage de couleur rouge
upper_red = np.array([190,255,255]) #bornes supérieur de la plage de couleur rouge

lower_yellow = np.array([15,0,0]) #bornes inférieur de la plage de couleur jaune
upper_yellow = np.array([36,255,255]) #bornes supérieur de la plage de couleur jaune

lower_white = np.array([0,0,240]) #bornes inférieur de la plage de couleur blanche
upper_white = np.array([255,10,255]) #bornes supérieur de la plage de couleur blanche
#----------------------------------------------------------------#


#-------------------Fonction enleverInf(tab)-------------------#
# Cette fonction sert à la détection des obstacles via le LiDAR, elle supprime les valeurs infinis 
# et calcule la moyenne du tableau donné en entrée (ici le tableau correspond aux valeurs renvoyées par le LiDAR sur une certaines plage angulaire)
def enleverInf(tab):
    # Nous ne gardons que les valeurs dans ce tableau qui sont inférieures à 0,35 (Enlever les infinis)
    #au dessus de 0,35 on considère que la distance perçu est infini
    res = [i for i in tab if i <0.35]  
    # Si le tableau 'res' n'est pas vide, il y a des obstacles dans cette plage, nous renvoyons donc la moyenne 
    # du tableau. S'il est vide, cela signifie qu'il n'y a pas d'obstacles dans cette plage, nous renvoyons 
    # donc une très grande valeur (ici 10). Cela permet d'avoir une valeur de "moyenne" même
    # si le tableau est vide et cette valeur est très grande pour représenté l'infini
    return np.mean(res) if res else 10
#--------------------------------------------------------------#
    

#-------------------Fonction publisher_obsta(linear, angular)-------------------#
# La fonction envoie les informations de vitesse du challenge 2 et du challenge 3
# Cette fonction est appelée dans la fonction callback 
def publisher_obsta(linear, angular):
    data = Twist()   #assigne les messages Twist à la variable data

    data.linear.x= linear #on assigne le parametre d'entrée linear à l'attribut linear.x du data type (ici c'est donc la vitesse lineaire)
    data.angular.z = angular #on assigne le parametre d'entrée angular à l'attribut angular.z du data type (ici c'est donc la vitesse angulaire)

    pub.publish(data)  #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()   
#--------------------------------------------------------------#
    
    
#-------------------Fonction tourne()-------------------#
# La fonction suivante fait tourner le robot. Elle est appellée quand le robot rencontre un obstacle
# Cette fonction est utilisée pour le challenge 1            
def tourne():
    global linear_x 
    global angular_z 

    data = Twist()
    data.linear.x = linear_x
    if direction:        # L'obstacle est situé à droite du robot, on fait tourner le robot vers la gauche
        data.angular.z= -angular_z *2  #le coefficient (ici 2) dépend du rate imposé et donc de l'ordinateur. Elle 
        #a été obtenu à partir de plusieur essais sur Gazebo en modifiant également le rate
        #le fonctionnement du robot dépend du couple de valeurs rate et angular.z qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)
        
    else:                # L'obstacle est situé à gauche du robot, on fait tourner le robot vers la droite
        data.angular.z= angular_z*2   #le coefficient (ici 2) dépend du rate imposé et donc de l'ordinateur. Elle 
        #a été obtenu à partir de plusieur essais sur Gazebo en modifiant également le rate
        #le fonctionnement du robot dépend du couple de valeurs rate et angular.z qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)
        
    pub.publish(data)    #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()   
#-------------------------------------------------------#
    
#-------------------Fonction follow(error)-------------------#
# La fonction suivante permet au robot de suivre la ligne et de modifier la direction en fonction de l'erreur
def follow(error):
    global linear_x

    data = Twist() 
    #NB: La ligne suivante (-float(error)/(une constante)) est fortement inspirée 
    # du code réalisé par "arjunskumar" que l'on peut retrouver via
    # ce lien https://github.com/arjunskumar/Line-Follower--ROS/blob/master/follower_ros.py)
    data.angular.z = -float(error)/60 # En fonction de l'erreur, on ajuste la direction du robot
    #la valeur 60 a été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
    data.linear.x=linear_x 
    pub.publish(data) #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()    
#-------------------------------------------------------# 
    
    
#-------------------Fonction callback_obsta(msg)-------------------#
#La fonction callback_obsta détecte l'obstacle et détermine l'orientation de l'obstacle
#De plus, il est capable de faire la transition entre deux challenge en fonction dela variable 'nombre_red'    
#Enfin, en raison du fait que le comportement du robot vis à vis des obstacle doit être différents selon les challenges, 
#nous devons adapter les parametres de détection aux différents challenges, pour cette raison 
#nous choisissons différentes plages de LiDAR et valeurs d'intervalle en fonction des challenges.
def callback_obsta(msg):
    global obstacle 
    global direction
    global threshold
    global linear_x
    global nombre_red 
    global front 
    global left
    global right
    
    #----------------------les différents challenges----------------------#
    
    # Si le 'nombre_red' arrive à 3, on passe au challenge 2 
    if nombre_red ==3:
        print("Entrer challenge 2")
        linear_x = 0  # Initialise la vitesse linéaire
        angular_z = 0 # Initialise la vitesse angulaire
        
        # On définit trois intervalles (parmi les données renvoyé par le LiDAR) 
        # une plage de données correspondant à l'avant-gauche du robot, une à l'avant-droite et une à l'avant du robot
        tab_front = msg.ranges[0:25] + msg.ranges[335:360]  # Avant
        tab_leftFront = msg.ranges[10:40]                   # Avant-Gauche
        tab_rightFront = msg.ranges[310:350]                # Avant-Droit

        # On calcule les moyennes (des distances/données du LiDAR) des deux tableaux Avant-Gauche et Avant-Droit précédents en utilisant la fonciton 'enleverInf(tab)'
        mean_leftFront = enleverInf(tab_leftFront)
        mean_rightFront = enleverInf(tab_rightFront)
        
        # On détermine la distance la plus courte renvoyée par les faisceaux LiDAR situé dans l'intervalle "Avant"
        # On compare ensuite cette distance au nombre 10 car cela permet d'enlever une potentielle valeur infini
        # Si aucune surface ne se trouve en face du robot le LiDAR renvoi uniquement des "Inf" 
        # La valeur min sera donc "Inf" (pas exploitable en terme de calcul)
        # C'est pour cette raison que l'on compare au nombre 10 afin de remplacer un potentiel "Inf" par le nombre 10
        # On assigne ensuite la plus petite valeur parmi les deux (le min du tableau et 10) à la variable min_front
        min_front = min(min(tab_front), 10)
        
        # On compare la valeur minimale déterminée précédemment avec la valeur limite threshold défini plutot
        # si la valeur minimale est supérieure au threshold, le robot ira tout droit.
        if min_front > threshold:
            linear_x = 0.1  #vitesse linéaire #la valeur a été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
            angular_z = 0  #vitesse angulaire nulle (on ne veut pas tourner)
        # si la valeur minimale est inférieur à la valeur limite threshold, on compare les moyennes 
        # de la plage de données avant-gauche et de la plage de données avant-droite
        # le robot tourne vers le côté ayant la plus grande moyenne car il s'agira du coté où la distance entre le robot
        # et une surface/obstacle est la plus grande 
        else: 
            if mean_leftFront > mean_rightFront:  # la moyenne de l'avant-gauche est plus grande que celle de l'avant-droit, donc on tourne vers la gauche
                linear_x = 0.0 #vitesse linéaire nulle (on veut pas avancer, on veut juste tourner)
                angular_z = 0.8 #vitesse angulaire #la valeur a été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
            elif mean_leftFront <= mean_rightFront: # la moyenne de de l'avant-droit est plus grande que celle de l'avant-gauche, donc on tourne vers la droite
                linear_x = 0.0 #vitesse linéaire nulle (on veut pas avancer, on veut juste tourner)
                angular_z = -0.8  #vitesse angulaire #la valeur a été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
                
        publisher_obsta(linear_x, angular_z )    

    # Si le 'nombre_red' arrive à 5, on passe au challenge 3 
    elif nombre_red ==5:
        print("Entrer challenge 3 !")
        # On définit trois intervalles (parmi les données renvoyé par le LiDAR) 
        # une plage de données correspondant à l'avant-gauche du robot, une à l'avant-droite et une à l'avant du robot
        tab_front = msg.ranges[0:10] + msg.ranges[350:360]  # Avant
        tab_left = msg.ranges[20:40]                        # Gauche
        tab_right = msg.ranges[320:340]                     # Droite

        # On détermine la distance la plus courte renvoyée par les faisceaux LiDAR situé dans les intervalles définit précédemment
        # On compare ensuite ces distances au nombre 1 car cela permet d'enlever une potentielle valeur infini
        # Si aucune surface ne se trouve en face du robot le LiDAR renvoi uniquement des "Inf" 
        # La valeur min sera donc "Inf" (pas exploitable en terme de calcul)
        # C'est pour cette raison que l'on compare au nombre 1 afin de remplacer un potentiel "Inf" par le nombre 1
        # On assigne ensuite la plus petite valeur parmi les deux (le min du tableau et 1) aux variables min_front, min_left, min_right 
        # Dans la suite 'min_left' et 'min_right' vont aider le robot 
        # à choisir le sens de rotation dans les différentes situations que va rencontrer le robot
        min_front = min(min(tab_front),1)
        min_left = min(min(tab_left),1)
        min_right = min(min(tab_right),1)

        #NB: Nous avions essayé de choisir le sens de rotation en nous basant non pas sur le minimum comme nous l'avions
        # écrit dans les 4 lignes de dessus mais en nous basant sur les moyennes des intervalles gauche et
        # droite en utilisant la fonctiton 'enleverInf(tab)' comme on peut le voir dans les 2 lignes en commentaires ci dessous
        # mean_left = enleverInf(tab_left)
        # mean_right = enleverInf(tab_right)
        #Les résultats obtenus sur la simulation Gazebo étaient meilleurs lorsque l'on utilisait le minimum, pour cette raison nous avons
        #gardé l'utilisation du minimum dans cette version du code

        # si la distance minimale est inférieur au seuil (threshold), alors on considère qu'il existe un obstacle dans cette intervalle
        # si la distance minimale est supérieur au seuil (threshold), alors on considère qu'il n'y a aucun obstacle dans cette intervalle
        if min_front < (threshold+ 0.1) :
            front  = True  # Si il y a un obstacle, front  = True
        else: 
            front  = False # Sinon, front  = False
        
        # De même, on ajuste la valeur de 'left' et 'right' en utilisant le même principe
        if min_left < (threshold-0.05):
            left = True
        else:
            left = False
            
        if min_right < (threshold-0.05) :
            right= True
        else:
            right = False
        
        # Déclare la vitesse linéaire et angulaire (par default) #les valeurs ont été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
        linear_x = 0.15
        angular_z = 0.2
        #NB: Dans la suite on appel plusieurs fois la fonction publisher_obsta(linear,angular)
        #les valeurs mis en paramètre de cette fonction ont été déterminé par des test successifs dans la simulation de notre robot sur Gazebo
        
        
        #----------------------les différentes situations----------------------#
        # En fonction des trois variables 'avant', 'gauche' et 'droite', Le robot choisit 
        # l'une des situations suivantes et publie la vitesse (angulaire et lineaire) correspondante
    
        # Situation 1, quand le robot sort de l'environnement d'obstacle (=sprt du challenge 3), on s'arrête
        if (min(msg.ranges[0:100] +msg.ranges[260:360]) > 0.8):
            print('Le robot est sorti, le challenge 3 est fini !')
            publisher_obsta(0, 0)  # La vitesse linéaire et angulaire = 0 #on stop le robot
        else:
            #  Situation 2, quand il y a l'obstacle à gauche mais pas à l'avant et ni à droite, on tourne vers la droite    
            if left and not front and not right:
                print("tourne vers la droite ")
                publisher_obsta(0.02, -angular_z) 
    
            #  Situation 3, quand il y a l'obstacle à droite mais pas à l'avant et ni à gauche, on tourne vers la gauche    
            elif right and not front and not left:
                print("tourne vers la gauche ")
                publisher_obsta(0.02, angular_z)
            
            #  Situation 3, quand il y a l'obstacle à l'avant, on tourne en fonction de la distance minimale de gauche et droite  
            elif front and not right and not left:
                # Si la distance minimale de gauche est plus grande que celle de droite, on tourne vers la gauche
                if min_left > min_right:
                    print("tourne vers la gauche ")
                    publisher_obsta(0.01, angular_z)
    
                # Si la distance minimale de droite est plus grande que celle de gauche, on tourne vers la droite
                else:
                    print("tourne vers la droite ")
                    publisher_obsta(0.01, -angular_z)
    
            # Situation 4, quand il y a un obstacle à gauche et à l'avant mais pas à droite, on tourne vers la droite   
            elif left and front and not right:
                print("tourne vers la droite ")
                publisher_obsta(0.01, (-angular_z))
    
            # Situation 5, quand il y a un obstacle à droite et à l'avant mais pas à gauche, on tourne vers la gauche   
            elif right and front and not left:
                print("tourne vers la gauche ")
                publisher_obsta(0.01, (angular_z))
    
            # Situation 6, quand il y a un obstacle à droite et à gauche mais pas à l'avant, on va vers l'avant (tout droit) à une vitesse plus lente que d'habitude  
            elif right and left and not front :
                print("tout droit")
                publisher_obsta(linear_x*0.5, 0)
    
            # Situation 7, quand il y a un obstacle à droite, à gauche et à l'avant, on se retourne
            elif right and front and left:
                print("se retourne")
                publisher_obsta(0.01, angular_z*1.5)
    
            # Situation 8, quand il n'y a pas d'obstacle ni à droite, ni à gauche et ni àl'avant, on va vers l'avant (tout droit)
            elif not front and not right and not left:
                print("Pas d'obstacle")
                publisher_obsta(linear_x, 0)
                
    #Pour les autres situation/cas, on est dans challenge 1
    else:
        # On définit deux intervalles (parmi les données renvoyé par le LiDAR) 
        # une plage de données correspondant à l'avant-gauche du robot et une à l'avant-droite
        tab_leftFront = msg.ranges[20:50]            # l'intervalle de gauche
        tab_rightFront = msg.ranges[310:340]        # l'intervalle de droite
    
        # On Calcule les moyennes (des distances/données du LiDAR) des deux tableaux précédents en utilisant la fonciton 'enleverInf(tab)'
        mean_leftFront = enleverInf(tab_leftFront)
        mean_rightFront = enleverInf(tab_rightFront)
        
        # Si la distance entre l'obstacle et le robot est inférieur à la valeur limite threshold, 
        # 'obstacle' = True, sinon 'obstacle = False'
        if mean_leftFront <threshold or mean_rightFront <threshold :
            obstacle = True
        else:
            obstacle = False
    
        # si la moyenne de gauche est inférieure à celle de droite cela signifie que l'obstacle est à gauche, 
        # il faut donc tourner à droite, 'direction' = True 
        if mean_leftFront < mean_rightFront:
            direction  = True
        elif mean_leftFront >= mean_rightFront: # sinon, l'obstacle est à droite, 'direction' = False
            direction = False
#-------------------------------------------------------------------#
            
#-------------------Fonction callback_follow(msg)-------------------#
# La fonction callback_follow détecte la couleur et calcule le centre de la ligne 
# et la compare avec le centre du robot
def callback_follow(msg):
    global obstacle
    global linear_x 
    global angular_z 
    global redLine
    global nombre_red
    
    #NB: Les fonctions openCV et leur utilisation on été inspiré du code fourni en annexe du
    #PDF: "ROS and experimental robotics Part 3: project"
    #L'idée de mettre à 0 les pixels du masque dans la zone du masque que l'on souhaite
    # négligé vient du code réalisé par "arjunskumar" que l'on peut retrouver via
    # ce lien https://github.com/arjunskumar/Line-Follower--ROS/blob/master/follower_ros.py
    
    # on créer un bridge 
    cvBridge = cv_bridge.CvBridge ()
    # on convertit l'image vers openCV et on encode avec la représentation
    cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
    
    # On récupére les informations de la taille de l'image, qui vont nous servir à définir le masque
    hight, width, deep = cvImage.shape
    
    # Définit le centre de l'image, qui est également le centre du robot 
    centre_robot = width/2
    
    # on passe les couleurs de l'image de la représentation RGB à HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    
    #Image binarisation
    #-----On définit les masques de la couleur jaune et blanche-----#
    #le masque jaune va permettre de détecter la ligne jaune
    #le masque blanc va permettre de détecter la ligne blanche
    #on indique la représentation des couleurs utilisée ainsi que les bornes/plages de couleurs des differents masques
    #les masque vont détecter les couleurs comprises entre les bornes indiquées (cf bornes définit plus haut)
    mask_yellow = cv2.inRange (hsv, lower_yellow, upper_yellow) 
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    mask_red = cv2.inRange(hsv,lower_red,upper_red)
    
    # On couvre/cache la partie supérieure des masques de l'image afin de permettre au robot de se concentrer uniquement 
    # sur la partie du chemin proche de lui et d'éviter d'être "distrait" par des 
    # changements de trajectoires/positions des lignes (chemin) loin de lui
    partieCache = int(4*hight/6)    #La partie cachée, qui est 4/6 hauteur de l'image (le point 0,0 correspond au coin haut gauche de l'image)
    #on cache les parties des masques en remplaçant les valeurs correspondantes dans les matrices par zéro
    
    #(NB: idée fortement inspiré du code réalisé par "arjunskumar" que l'on peut retrouver via
    # ce lien https://github.com/arjunskumar/Line-Follower--ROS/blob/master/follower_ros.py)
    mask_yellow[0:partieCache, 0:width] = 0 
    mask_white[0:partieCache, 0:width] = 0
    mask_red[0:int(7*hight/8), 0:width] = 0
    #--------------------------------------------------------------------#
    
    
    # -----Afin de passer l'intersection (rond-point)-----: 
    # nous utilisons cachons une autre partie des masques et nous faisons en sorte que
    # les deux masques soient asymétriques (pour que les images jaunes et 
    # blanches soient détectées de manière asymétriques) afin de passer l'intersection. 
    # Pour ce faire: 

    # Nous masquons d'abord la moitié gauche de l'image blanche et la moitié droite de l'image jaune, de sorte que
    # lorsque le robot entre dans l'intersection, il ne puisse pas voir la partie jaune du cercle/ilot central sur 
    # l'image jaune, et de même avec la partie blanche du cercle/ilot central sur l'image blanche.    
    demi_width = int(width/2)  # le centre de l'image
    
    # Ensuite, nous déclarons une variable 'ecart' pour créer un déséquilibre: Nous réduisons la partie cachée sur l'image jaune 
    # sans changer la partie cachée sur l'image blanche, de sorte que lorsque le robot entre dans l'intersection, 
    # il détectera la ligne jaune du cercle central uniquement mais pas la ligne blanche, donc il tournera à droite 
    ecart = 40  
    
    # Grace à cette astuce le robot arrive à l'intersection et il peut détecter la ligne jaune 
    # de l'anneau central mais pas la ligne blanche, donc il tourne à droite
    mask_yellow[0:hight, (demi_width+ecart):width] = 0
    mask_white[0:hight, 0:(demi_width)] = 0
    # Afin de mieux compter le nombre de ligne rouge franchi (redline), nous couvrons la majeure partie de la zone de la fenêtre rouge,
    #ne laissant qu'une petite zone au milieu et en bas pour la détection
    mask_red[0:hight, 0:(demi_width-5)] = 0
    mask_red[0:hight, (demi_width+5):width] = 0
    
    # Nous aurions également pu écrire ceci (les 5 lignes suivants) pour que le robot tourne
    # #à gauche au  lieu de tourner à droite à au niveau de l'intersection (rond-point). 
    # Quand le robot arrive à l'intersection, il peut détecter la ligne blanche de 
    # l'anneau central mais pas la ligne jaune, donc il tourne à gauche
    #mask_yellow[0:hight, (demi_width):width] = 0
    #mask_white[0:hight, 0:(demi_width - ecart)] = 0

    # Enfin, pour que ce déséquilibre puisse s'appliquer à d'autres situations que l'intersection, 
    # on sélectionne strictement la valeur de ce paramètre "ecart"
    #-----------------------------------------------------------------------#
    
    #---------Calculer le centre de l'image apres avoir appliqué un masque ----------#
    #Nous calculons donc les centres géométrique uniquement à partir des pixels qui ne sont pas nuls
    #les pixels cachés par le masque ne sont pas pris en compte dans le calcul du centre
    #Cela revient donc dans notre cas à calculer le centre des lignes détécté
        
    #On calcule les moments des images (toujours apres avoir appliqué les masques)
    M_yellow = cv2.moments(mask_yellow)
    M_white  = cv2.moments(mask_white )
    M_red = cv2.moments(mask_red)
    
    # On calcule le centre de l'image (apres avoir appliqué un masque jaune) à l'aide des moments
    # Considérant que le robot peut ne pas détecter la ligne lors du passage de l'obstacle, 
    # lorsque la ligne n'est pas détectée, on donne forcément une valeur à cX et cY, afin que le robot puisse 
    # également corriger la direction selon la ligne tout en évitant l'obstacle.
    if M_yellow[ "m00" ] >0: # Calcule les coordonnées x,y du centre
        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
    else:               #Si on détecte pas la ligne, donne une valeur à cX et cY
        cX_yellow =5
        cY_yellow = 180
        
    # On dessine un cercle à la position du centre "jaune" calculé juste avant
    # On l'affichera (cf code un peu plus bas) ensuite sur la figure représentant l'image sur lequels le masque "jaune" a été appliqué 
    cv2.circle(mask_yellow, (cX_yellow ,cY_yellow ), 15, 50,2)

    #On répète la même opération pour l'image une fois que l'on a appliqué le masque blanc 
    if M_white [ "m00" ] >0 :   # Calcule les coordonnées x,y du centre         
        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
    else:              #Si on détecte pas la ligne, donne une valeur à cX et cY
        cX_white = 315
        cY_white = 180
    #On dessine un cercle à la position du centre "blanc" calculé juste avant
    cv2.circle(mask_white, (cX_white ,cY_white ), 15, 150,2)
    
    
    # Détection du nombre de ligne rouge (redLine  = [0,0])
    # le principe de détection est lle suivant: quand en premier temps on détecte la ligne rouge, et ensuite on ne détecte plus la ligne rouge, 
    # Alors nous avons passé un ligne rouge, donc le nombre de ligne rouge franchi augmente (+ 1).
    if M_red["m00"]>0:  #Si on peut détecter la ligne rouge : redLine[0] = 1
        redLine[0] = 1
    else:              # Si on ne détect pas la ligne rouge: 
        if redLine[0] ==1: # Si on a déja détecté la ligne rouge, alors redLine[1] = 1
            redLine[1] =1
        if redLine[0] ==1 and redLine[1] ==1:  # Si on a déja détecté la ligne rouge, et ensuite on ne détecte plus la ligne rouge, 
                                               # Cela signifie que nous avons passé une ligne rouge, donc le nombre de ligne rouge franchi augmente (+ 1)                  
            nombre_red +=1
            redLine[0], redLine[1] = 0,0    # Enfin, si on passe un ligne rouge, on reinitialise le tableau 'redLine' = 0
            
    #--------------------------------------------------------#
        
    #On détermine le centre entre les deux lignes (jaune et blanche) en prenant le milieu de
    #la distance entre les coordonnées x du centre "jaune" et du centre "blanc"
    centre_element = (cX_yellow + cX_white)/2
    
    # On calcule l'erreur entre le centre du robot (= le centre de l'écran) et du centre des deux lignes, qui servira à ajuster la direction du robot
    error = centre_element - centre_robot

    #On affiche la figure représentant l'image une fois le masque jaune appliqué. De même avec le masque blanc et rouge
    cv2.imshow("Image window yellow", mask_yellow )
    cv2.imshow("Image window white", mask_white  )
    cv2.imshow("Image window red", mask_red)
    cv2.waitKey(3)
    
    # Si on n'est pas dans le challenge 2 et 3
    if nombre_red != 3 and nombre_red != 5:
        #Si il a y un obstacle en avant, on tourne afin d'éviter l'obstacle
        if obstacle:
            tourne()     #appel de la fonction tourne()
        #si il y a pas de l'obstacle, on suit les lignes et on ajuste la direction en fonction de l'erreur
        elif not obstacle:
            follow(error)  #appel de la fonction follow avec l'erreur calculé juste avant comme input
#-------------------------------------------------------------------#   
    
#-------------------Main-------------------#  
if __name__ == '__main__':
    try:
        # On crée un publisher, et on publie l'information de vitesse sur le topic nommé 'cmd_vel', 
        # le type de message est Twist. La longueur de la file d'attente est de 10, on peut donc
        #avoir 10 messages mis en file d'attente pour être executé si le code prend du temps
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        #On définit un subscriber qui est abonné au topic /camera/image, et on envoi 
        # le message Image (provenant du topic /camera/image) à la fonction callback_follow
        rospy.Subscriber("/camera/image", Image, callback_follow)

        #On définit un subscriber qui est abonné au topic /scan, et on envoi
        # le message LaserScan (provenant du topic /scan) à la fonction callback_obsta
        rospy.Subscriber("/scan", LaserScan, callback_obsta)

        #spin() permet de maintenir l'existance de python jusqu'a ce que le noeud soit stoppé (avec un CTRL V dans le terminal par exemple)
        rospy.spin() 

    #les lignes suivantes "attrape" une "rospy.ROSInterruptException" exception, 
    #qui peut provenir des méthodes rospy.sleep() et rospy.Rate.sleep() lorsque l'on fait
    # Ctrl-C ou que notre noeud est shutdown/stoppé. 
    #On relève cette exception pour qu'on ne continue pas d'executer le code accidentellement apres le sleep().
    except rospy.ROSInterruptException:
        pass
#-------------------------------------------#