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
#Importe les modules et les informations sur les types de messages utilisés
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image     #message type de la Caméra
from geometry_msgs.msg import Twist   #message type de la Vitesse
from sensor_msgs.msg import LaserScan #message type du LiDAR
#-----------------------------------------------------------------------#

#-----------------------------------------------------------------------#
# Initialise un noeud qui s'appelle 'follow' pour publier les informations de vitesse.
rospy.init_node('follow', anonymous = True)

# Défini le rate/fréquence d'envoi des messages
rate = rospy.Rate(500) #le rate a une influence sur les performances du robot et dépend de l'ordinateur utilisé !!!!!!!!!!!! (il faut donc peut etre modifier cette valeur sur un autre ordinateur)
#le fonctionnement du robot dépend du couple de valeurs "rate" et "vitesse angulaire défini dans la fonction tourne()" (il faut donc peut etre modifier ces valeurs sur un autre ordinateur)
#(la valeur indiqué ici 500 a été obtenu à partir de plusieur essais sur Gazebo en modifiant également la vitesse angulaire)

#le fonctionnement du robot dépend du couple de valeurs rate et angular.z qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)


#-----------------------------------------------------------------------#

#-------------------On déclare les variables globales-------------------#
global obstacle  # Variable globale qui détermine si nous approchons d'un obstacle
                 # obstacle = True: le robot rencontre un obstacle
                 # obstacle = False: pas d'obstacle, avance.
global linear_x  # linear speed / vitesse linéaire/de translation
global angular_z # angular speed / vitesse de rotation
global threshold # Distance à partir de laquelle on considére une surface comme un obstacle
                 # au dessus de treshold les surfaces détéctées ne sont pas considérer comme des obstacles
                 # en dessous de treshold, on est trop proche donc les surfaces détéctées sont considérées comme des obstacles
global direction # Variable globale qui représente la position de l'obstacle
                 # direction = False: L'obstacle est situé à droite du robot, il faut tourner vers la gauche
                 # direction = True: L'obstacle est situé à gauche du robot, il faut tourner vers la droite
#-----------------------------------------------------------------------#

#-------------------initialisation des valeurs-------------------#
linear_x = 0.1
angular_z = 0.35
#threshold = rospy.get_param("threshold") # récupérer l'information du threshold
threshold = 0.3
obstacle = False # il y a pas d'obstacle au départ
direction = False  
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

#-------------------Fonction callback_obsta(msg)-------------------#
#La fonction callback_obsta détecte l'obstacle et détermine l'orientation de l'obstacle
def callback_obsta(msg):
    global obstacle 
    global direction
    global threshold

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
# La fonciton callback_follow qui détecte la couleur, calcule le centre de la ligne 
# et le compare avec le centre du robot
def callback_follow(msg):
    global obstacle
    global linear_x 
    global angular_z 
    
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
    
    # On couvre/cache la partie supérieure des masques de l'image afin de permettre au robot de se concentrer uniquement 
    # sur la partie du chemin proche de lui et d'éviter d'être "distrait" par des 
    # changements de trajectoires/positions des lignes (chemin) loin de lui
    partieCache = int(4*hight/6)    #La partie cachée, qui est 4/6 hauteur de l'image (le point 0,0 correspond au coin haut gauche de l'image)
    #on cache les parties des masques en remplaçant les valeurs correspondantes dans les matrices par zéro

    #(NB: idée fortement inspirée du code réalisé par "arjunskumar" que l'on peut retrouver via
    # ce lien https://github.com/arjunskumar/Line-Follower--ROS/blob/master/follower_ros.py)
    mask_yellow[0:partieCache, 0:width] = 0 
    mask_white[0:partieCache, 0:width] = 0
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
    mask_yellow[0:hight, (demi_width + ecart):width] = 0
    mask_white[0:hight, 0:(demi_width)] = 0

    # Nous aurions également pu écrire ceci (les 5 lignes suivants) pour que le robot tourne
    # #à gauche au  lieu de tourner à droite à au niveau de l'intersection (rond-point). 
    # Quand le robot arrive à l'intersection, il peut détecter la ligne blanche de 
    # l'anneau central mais pas la ligne jaune, donc il tourne à gauche
    #mask_yellow[0:hight, (demi_width):width] = 0
    #mask_white[0:hight, 0:(demi_width - ecart)] = 0

    # Enfin, pour que ce déséquilibre puisse s'appliquer à d'autres situations que l'intersection, 
    # on sélectionne strictement la valeur de ce paramètre ecart
    #-----------------------------------------------------------------------#

    #---------Calculer le centre de l'image apres avoir appliqué un masque ----------#
    #Nous calculons donc les centres géométrique uniquement à partir des pixels qui ne sont pas nuls
    #les pixels cachés par le masque ne sont pas pris en compte dans le calcul du centre
    #Cela revient donc dans notre cas à calculer le centre des lignes détécté

    #On calcule les moments des images (toujours apres avoir appliqué les masques)
    M_yellow = cv2.moments(mask_yellow)
    M_white  = cv2.moments(mask_white )
    
    # On calcule le centre de l'image (apres avoir appliqué un masque jaune) à l'aide des moments
    # Considérant que le robot peut ne pas détecter la ligne lors du passage de l'obstacle, 
    # lorsque la ligne n'est pas détectée, on donne forcément une valeur à cX et cY, afin que le robot puisse 
    # également corriger la direction selon la ligne tout en évitant l'obstacle.
    if M_yellow[ "m00" ] >0: # Calcule les coordonnées x,y du centre
        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
    else:              #Si on détecte pas la ligne, donne une valeur à cX et cY
        cX_yellow =5   
        cY_yellow = 180
    
    # On dessine un cercle à la position du centre "jaune" calculé juste avant
    # On l'affichera (cf code un peu plus bas) ensuite sur la figure représentant l'image sur lequels le masque "jaune" a été appliqué 
    cv2.circle(mask_yellow, (cX_yellow ,cY_yellow ), 15, 50,2)

    #On répète la même opération pour l'image une fois que l'on a appliqué le masque blanc 
    if M_white [ "m00" ] >0 :   # Calcule les coordonnées x,y du centre    
        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
    else:                #Si on détecte pas la ligne, donne une valeur à cX et cY
        cX_white = 315  
        cY_white = 180

    #On dessine un cercle à la position du centre "blanc" calculé juste avant
    cv2.circle(mask_white, (cX_white ,cY_white ), 15, 150,2)
    #--------------------------------------------------------#

    #On détermine le centre entre les deux lignes (jaune et blanche) en prenant le milieu de
    #la distance entre les coordonnées x du centre "jaune" et du centre "blanc"
    centre_element = (cX_yellow + cX_white)/2
    
    # On calcule l'erreur entre le centre du robot (= le centre de l'écran) et du centre des deux lignes, qui servira à ajuster la direction du robot
    error = centre_element - centre_robot

    #On affiche la figure représentant l'image une fois le masque jaune appliqué. De même avec le masque blanc
    cv2.imshow("Image window1", mask_yellow )
    cv2.imshow("Image window2", mask_white  )
    cv2.waitKey(3)

    #Si il a y un obstacle en avant, on tourne afin d'éviter l'obstacle
    if obstacle:
        tourne()     #appel de la fonction tourne()
    #si il y a pas de l'obstacle, on suit les lignes et on ajuste la direction en fonction de l'erreur
    elif not obstacle:
        follow(error)  #appel de la fonction follow avec l'erreur calculé juste avant comme input
#-------------------------------------------------------------------#

#-------------------Fonction tourne()-------------------#
# La fonction suivante fait tourner le robot. Elle est appelée quand le robot rencontre un obstacle           
def tourne():
    global linear_x 
    global angular_z 

    data = Twist()
    data.linear.x = linear_x
    if direction:        # L'obstacle est situé à droite du robot, on fait tourner le robot vers la gauche
        data.angular.z= -angular_z *2.2  #le coefficient (ici 2.2) dépend du rate imposé et donc de l'ordinateur. Elle 
        #a été obtenu à partir de plusieur essais sur Gazebo en modifiant également le rate
        #le fonctionnement du robot dépend du couple de valeurs rate et angular.z qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)
    else:                # L'obstacle est situé à gauche du robot, on fait tourner le robot vers la droite
        data.angular.z= angular_z*2.2  #le coefficient (ici 2.2) dépend du rate imposé et donc de l'ordinateur. Elle 
        #a été obtenu à partir de plusieur essais sur Gazebo en modifiant également le rate
        #le fonctionnement du robot dépend du couple de valeurs rate et angular.z qui eux même dépendent de l'ordinateur utilisé (car le temps d'execution du code varie)
    pub.publish(data)    #on publie la consigne de vitesse linéaire et angulaire sur le topic dédié
    #on met en sleep (=repos/attente) le code si jamais le code c'est executé plus vite que le rate imposé
    #cela permet de maintenir le rate indiqué
    rate.sleep()   
#-------------------------------------------------------#

#-------------------Fonction follow(error)-------------------#
# La fonction suivante permet au robot de suivre la ligne et de modifier le direction en fonction de l'erreur
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