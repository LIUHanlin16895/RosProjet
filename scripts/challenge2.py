#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#-----------------------------------------------------------------------#
#Importe les modules et les informations sur les types de messages utilisés
import numpy as np
import rospy
from geometry_msgs.msg import Twist   #message type de la Vitesse
from sensor_msgs.msg import LaserScan  #message type du LiDAR
#-----------------------------------------------------------------------#

#-----------------------------------------------------------------------#
# Initialise un noeud qui s'appelle 'corridor' pour publier les informations de vitesse.
rospy.init_node('corridor', anonymous = True)

# Défini le rate/fréquence d'envoi des messages
rate = rospy.Rate(300) #le rate a une influence sur les performances du robot et dépend de l'ordinateur utilisé !!!!!!!!!!!! (il faut donc peut etre modifier cette valeur sur un autre ordinateur)
#(la valeur indiqué ici 300 a été obtenu à partir de plusieur essais sur Gazebo)

#le fonctionnement du robot dépend du rate qui lui même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)

#-----------------------------------------------------------------------#

#-------------------On déclare les variables globales-------------------#
global threshold # Distance à partir de laquelle on considére une surface comme un obstacle
                 # au dessus de treshold les surfaces détéctées ne sont pas considérer comme des obstacles
                 # en dessous de treshold, on est trop proche donc les surfaces détéctées sont considérées comme des obstacles
#-----------------------------------------------------------------------#

#-------------------initialisation des valeurs-------------------#
#threshold = rospy.get_param("threshold")
threshold = 0.3
#----------------------------------------------------------------#

data = Twist() #assigne les messages Twist à la variable data

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

#-------------------Fonction callback_obsta(msg)-------------------#
# la fonction callback_obsta analyse les données émis par le LiDAR du robot et 
# publie des informations sur la vitesse que doit avoir le robot en fonction de ces données
def callback_obsta(msg):
    global threshold
    
    data = Twist()  #assigne les messages Twist à la variable data
    
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
        linear_x = 0.18  #vitesse linéaire #la valeur a été déterminé par des test successifs dans la simulation de notre robot sur Gazebo 
        angular_z = 0  #vitesse angulaire nulle (on ne veut pas tourner)

        # Si les distances minimales à l'avant, à l'avant-gauche et à l'avant-droite sont toutes supérieures à 1, 
        # alors on considère que le robot a quitté le couloir et le challenge est terminé
        if min(msg.ranges[0:60] + msg.ranges[300:360])>1:
            linear_x = 0   #vitesse linéaire #on stop le robot
            angular_z = 0  #vitesse angulaire #on stop le robot
            print('Le Robot est sorti du carridor, le challenge 2 est fini !')

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

    # Publication des informations de vitesse    
    data.linear.x = linear_x
    data.angular.z = angular_z 
    pub.publish(data)  #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()    
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