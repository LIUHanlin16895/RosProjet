#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#-----------------------------------------------------------------------#
#Importe les modules et les informations sur les types de messages utilisés
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#-----------------------------------------------------------------------#

#-----------------------------------------------------------------------#
# Initialise un noeud qui s'appelle 'navigation' pour publier les informations de vitesse.
rospy.init_node('navigation', anonymous = True)

# Défini le rate/fréquence d'envoi des messages
rate = rospy.Rate(300) #le rate a une influence sur les performances du robot et dépend de l'ordinateur utilisé !!!!!!!!!!!! (il faut donc peut etre modifier cette valeur sur un autre ordinateur)
#(la valeur indiqué ici 300 a été obtenu à partir de plusieur essais sur Gazebo)

#le fonctionnement du robot dépend du rate qui lui même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)

#-----------------------------------------------------------------------#

#-------------------On déclare les variables globales-------------------#
# On déclare trois global variable 'front, left, right' qui représente l'existance d'un obstalce dans ces trois plages.
# par exemple, si il y a un obstacle dans l'intervalle front, 'front' = true, sinon, 'front' = False.
global front  
global left
global right
global threshold  # Distance à partir de laquelle on considére une surface comme un obstacle
                 # au dessus de treshold les surfaces détéctées ne sont pas considérer comme des obstacles
                 # en dessous de treshold, on est trop proche donc les surfaces détéctées sont considérées comme des obstacles
#-----------------------------------------------------------------------#

#-------------------initialisation des valeurs-------------------#
front = False
left = False
right = False
threshold = 0.25
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
# La fonction envoie les informations de vitesse
# cette fonction est appelée dans la fonction callback
def publisher_obsta(linear, angular):
    data = Twist()   #assigne les messages Twist à la variable data

    data.linear.x= linear #on assigne le parametre d'entrée linear à l'attribut linear.x du data type (ici c'est donc la vitesse lineaire)
    data.angular.z = angular #on assigne le parametre d'entrée angular à l'attribut angular.z du data type (ici c'est donc la vitesse angulaire)

    pub.publish(data)  #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()   
#--------------------------------------------------------------#

#-------------------Fonction callback(msg)-------------------#
# Cette fonction permet de déterminer l'orientation de l'obstacle 
# et de faire un choix dans diverses situations
def callback(msg):
    global front 
    global left
    global right
    global threshold

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
    if min_front < (threshold+ 0.15) :
        front  = True  # Si il y a un obstacle, front  = True
    else: 
        front  = False # Sinon, front  = False
    
    # De même, on ajuste la valeur de 'left' et 'right' en utilisant le même principe
    if min_left < threshold:
        left = True
    else:
        left = False
        
    if min_right < threshold :
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
#-------------------------------------------------------------------#

#-------------------Main-------------------#
if __name__ == '__main__':
    try:
        # On crée un publisher, et on publie l'information de vitesse sur le topic nommé 'cmd_vel', 
        # le type de message est Twist. La longueur de la file d'attente est de 10, on peut donc
        #avoir 10 messages mis en file d'attente pour être executé si le code prend du temps
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        #On définit un subscriber qui est abonné au topic /scan, et on envoi
        # le message LaserScan (provenant du topic /scan) à la fonction callback_obsta
        rospy.Subscriber("/scan", LaserScan, callback)

        #spin() permet de maintenir l'existance de python jusqu'a ce que le noeud soit stoppé (avec un CTRL V dans le terminal par exemple)
        rospy.spin()

    #les lignes suivantes "attrape" une "rospy.ROSInterruptException" exception, 
    #qui peut provenir des méthodes rospy.sleep() et rospy.Rate.sleep() lorsque l'on fait
    # Ctrl-C ou que notre noeud est shutdown/stoppé. 
    #On relève cette exception pour qu'on ne continue pas d'executer le code accidentellement apres le sleep().
    except rospy.ROSInterruptException:
        pass
#-------------------------------------------#