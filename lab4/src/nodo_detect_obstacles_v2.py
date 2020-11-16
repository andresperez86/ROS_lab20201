#!/usr/bin/env python
# encoding: utf-8

#Linea 1 - “Shebang”,le indicamos a la máquina con qué programa lo vamos a ejecutar.
#Linea 2 - Python 2.7 - asume que solo se utiliza ASCII en el código fuente
# para usar utf-8 hay que indicarlo al principio de nuestro script encoding: utf-8


import rospy                                                            # Importamos ropsy (interface de python-ROS)
from sensor_msgs.msg import LaserScan                                   #Importamas el tipo de mensaje Lasersan


def callback(mensaje):                  #Definimos una función callback
   
    for i in range(len(mensaje.ranges)):
        print("[%d] =%f" % (i,mensaje.ranges[i])) #Damos formato a la impresión en pantalla
    print("------------\n")


def nodo():                                                             # Definimos una función nodo

    rospy.init_node('nodo_detect_obstacles')                            # Inicializamos nuestro nodo y le asignamos un nombre = transmisor

    #Nos suscribimos al tópico /base_scan
                               #Name Topic|tipo de mensaje|función
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback)

    rospy.spin()                                                        # Mantiene corriendo el script hasta que se detiene la ejecución del script con Crtl+C


if __name__ == '__main__':                                              # Llamamos a la función principal main
    try:
        nodo()                                                          # Lamamos a la función nodo    
    except rospy.ROSInterruptException :                            #Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass
