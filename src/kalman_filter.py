"""
Reste à faire :
- Vérifier la jacobienne 'C' de l'odom
- Transformer les coordonnées (x, y, theta) du lydar par scan2d en (x, y, theta) du centre du chassis
dans la fonction 'callback_pos_scan2d' avec la tf
- Gérer la sortie des donnéees -> mise en forme par tf et envoies dans les bons topics

Prérequis :
- Créer un message 'Scan2d' tel qu'on puisse accéder aux 'attributs' :
Scan2d.x, Scan2d.y, Scan2d.theta
- Créer un topic 'pos_scan_2d' avec ce type de message et mettre à jour le message seulement lors de nouveaux scan
(c'est à dire ne pas envoyer d'anciennes informations, sinon ces dernières faussent le filtre)
MODIFICATION :
- Pour modifier la fiabilité accordée aux mesures, faire varier les variables :
'std_odometry', 'std_position_map', 'std_theta_map'
- Pour modifier la fiabilité accordée au modèle :
Faire varier les coefficient de 'SIGMA_OMEGA'
- Pour modifier la fiabilité accordée à la position initiale, faire varier la variable :
'doute_pos_initial' -> proche de 0 = Très confiant, proche de 10 = Peu confiant
REMARQUE :
-On s'embête à gérer les matrices sans Numpy, mais si le package peut être rajouter, ça simplifierai le code
et accélèrerai peut-être significativement les calculs (peut important à l'initialisation, plus dans la boucle
"""

import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Header
from math import cos, sin, pi
import numpy as np



LARGEUR = 0.215  # Largeur du chassis
RAYON = 0.05  # Rayon des roues
EYE5 = np.matrix([
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1]
])
SIGMA_OMEGA =np.matrix([  # Variance du Modèle, pris dans la littérature (cf Justin Cano)
    [0.01**2, 0, 0, 0, 0],  # Confiance en x
    [0, 0.01**2, 0, 0, 0],  # Confiance en y
    [0, 0, 0.2**2, 0, 0],   # Confiance en vt
    [0, 0, 0, 0.01**2, 0],  # Confiance en theta
    [0, 0, 0, 0, 0.2**2]    # Confiance en omega
])
doute_pos_initial = 2
sigma = doute_pos_initial * SIGMA_OMEGA  # Initialisation pour les itérations du filtre de Kalman

std_odometry = 0.01
std_odometry_translation = std_odometry*RAYON/(2**.5)
std_odometry_rotation = std_odometry*(2**.5)*RAYON/LARGEUR
SIGMA_NU_ODOM = np.matrix([  # Variance des acquisitions odométriques
    [std_odometry_translation**2, 0],
    [0, std_odometry_rotation**2]
])
std_position_map = 0.02
std_theta_map = 5 * (pi/180)  # Imprécision en radians
SIGMA_NU_SCAN2D = np.matrix([  # Variance des acquisitions scan2d
    [std_position_map**2, 0, 0],
    [0, std_position_map**2, 0],
    [0, 0, std_theta_map**2]
])

pos_l = 0
pos_r = 0
vit_l = 0
vit_r = 0
eff_r = 0
eff_l = 0
cmd_r = 0
cmd_l = 0
pos_scan_2d = [0, 0, 0]  # les coordonées x, y et théta obtenues par le lydar
new_scan = False  # variable indicant à la boucle principale si les données du scan ont été mises à jour


def callback_vit_r(msg):
    """On veut la vitesse de déplacement (!= rotation) de la roue droite"""
    global vit_r
    vit_r = msg.data
    return ()


def callback_vit_l(msg):
    """On veut la vitesse de déplacement (!= rotation) de la roue gauche"""
    global vit_l
    vit_l = msg.data
    return ()


def callback_cmd_r(msg):
    """On veut la consigne de déplacement (!= rotation) de la roue droite"""
    global cmd_r
    cmd_r = msg.data
    return ()


def callback_cmd_l(msg):
    """On veut la consigne de déplacement (!= rotation) de la roue gauche"""
    global cmd_l
    cmd_l = msg.data
    return ()


################################## À modifier une fois que le message du scan2D sera codé si besoin ######################
def callback_pos_scan_2d(msg):
    """On veut les coordonnées (x, y, theta) pour le centre du chassis"""
    global pos_scan_2d
    pos_scan_2d[0] = msg.x  # ############# Utiliser la TF pour passer de lydar -> centre du chassis
    pos_scan_2d[1] = msg.y  # ############# Utiliser la TF pour passer de lydar -> centre du chassis
    pos_scan_2d[2] = msg.theta  # Theta est sensé être le même pour tout le robot (?)
    global new_scan
    new_scan = True


rospy.init_node('kalman_filter')
rate = rospy.Rate(10)


## Définir le message à envoyer ici en l'initialisant (avec le topic /initialpose) ################################
# X = [x, y, vt, theta, omega] -> pris sur la position centrale du Robot !
X_hat = np.matrix([0 for i in range(5)]).transpose()  # matrice colonne
"""
Subscribe to "initialpose", et obtenir x, y, theta
vt = omega = 0 (vitesses nulles à l'initialisation)
En attendant on écrit que tout est nul
"""


old_time = rospy.Time.now()
while not rospy.is_shutdown():
    # Étape 0 : ACQUISITION DES DONNÉES
    rospy.Subscriber('wheel_vel_r', Float32, callback_vit_r)
    rospy.Subscriber('wheel_vel_l', Float32, callback_vit_l)
    #rospy.Subscriber('motor_cmd_r', Float32, callback_cmd_r)  # ces topics donnent une commande moteur € [-255;255]
    #rospy.Subscriber('motor_cmd_l', Float32, callback_cmd_l)  # et ne sont pas utilisables pour calculer des vitesses..
    rospy.Subscriber('rwheel_vtarget', Float32, callback_cmd_r)  # on utilise les vitesses de commande, même si
    rospy.Subscriber('lwheel_vtarget', Float32, callback_cmd_l)  # elles sont moins adaptées...
    ###################### nom du topic et nom du format de message à modifier une fois ceux-ci créés #####################
    rospy.Subscriber('pos_scan_2d', Scan2d, callback_pos_scan_2d)

    new_time = rospy.Time.now()
    dt = new_time - old_time
    old_time = new_time

    # Étape 1 : PRÉDICTION
    # -> utilisation de la vitesse cible pour la prédiction
    theta = X_hat[3]
    vt = (cmd_l + cmd_r) / 2
    omega = (cmd_l - cmd_r) / LARGEUR
    f = [vt*cos(theta), vt*sin(theta), 0, omega, 0]
    X_pred = X_hat.copy()
    X_pred[0] += vt * cos(theta) * dt
    X_pred[1] += vt * sin(theta) * dt
    X_pred[2] = vt
    X_pred[3] += omega * dt
    X_pred[4] = omega
    # Calcul de Ak ...
    A = np.matrix([
        [0, 0, cos(theta), -vt*sin(theta), 0],
        [0, 0, sin(theta), vt*cos(theta), 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0]
    ])
    Ak = EYE5 + A * dt
    sigma_pred = SIGMA_OMEGA + Ak * sigma * (Ak.transpose())

    # Étape '3' : CHOIX DES MESURES
    if new_scan:  # on utilise le scan_2d
        new_scan = False
        # La mesure
        y = np.matrix(pos_scan_2d.copy()).transpose
        # Jacobienne
        C = np.matrix([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0]
        ])
        g_x_pred = np.matrix([X_pred[0, 0], X_pred[1, 0], X_pred[3, 0]]).transpose()
        # innovation
        innovation = y - g_x_pred
        # Variance des capteurs
        sigma_nu = SIGMA_NU_SCAN2D

    else:  # on utilise l'odométrie
        # La mesure
        y = [vit_l, vit_r]
        # Jacobienne
        C = [
            [0, 0, 1, 0, LARGEUR/2],
            [0, 0, 1, 0, -LARGEUR/2]
        ]
        # Mesure prédite
        g_x_pred = np.matrix([X_pred[2, 0]+X_pred[4, 0]*LARGEUR/2, X_pred[2, 0]-X_pred[4, 0]*LARGEUR/2]).transpose()
        # innovation
        innovation = y - g_x_pred
        # Variance des capteurs
        sigma_nu = SIGMA_NU_ODOM

    # Étape '2' : GAIN DE KALMAN
    G = sigma_pred * (C.transpose()) * (C * sigma_pred * (C.transpose()) + sigma_nu)**-1

    # CALCULS FINAUX
    X_hat = X_pred + G * innovation
    sigma = (EYE5 - G * C) * sigma_pred


    # TRANSMISSION DES RÉSULTATS
    # On a construit notre estimée X_hat = matrix([x, y, vt, theta, omega]) du centre de notre chassis
    # Il faut mettre en forme cette information et la transmettre dans les bons topics

