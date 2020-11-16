#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from helpers import *

pi = np.pi

# Eslabon 0 del manipulador con respecto a la base del robot
Tb0 = np.matrix([[1.0, 0.0, 0.0, -0.086875+0.119525],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.37743+0.34858],
                 [0.0, 0.0, 0.0, 1.0]])


def forward_kinematics(q):
    """
    Cinematica directa del robot
    """
    # Longitudes (completar)


    # Matrices DH (completar)
    T1 =
    T2 =
    T3 =
    T4 =
    T5 =
    T6 =
    T7 =
    Ttotal =
    return Ttotal


if __name__ == "__main__":

    rospy.init_node("sendJointsNode")
    pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
    bmarker = BallMarker(color['RED'])

    # Nombres de las articulaciones
    jnames = ('shoulder_pan_joint', 'shoulder_lift_joint',
              'upperarm_roll_joint', 'elbow_flex_joint',
              'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint',
              'r_gripper_finger_joint', 'l_gripper_finger_joint', 'bellows_joint',
              'r_wheel_joint', 'l_wheel_joint', 'torso_lift_joint',
              'head_pan_joint', 'head_tilt_joint')

    # ========================================================================
    #                Modificar aqui
    # ========================================================================

    # Configuracion articular (en radianes)
    q = [0.1, -0.5, 0.1, -0.5, 0.5, -0.6, 0.4]

    # Cinematica directa para la configuracion articular (efector final con
    # respecto al eslabon 0 que es diferente de la base misma del robot, en
    # este caso)
    T0e = forward_kinematics(q)
    print np.round(T0e,3)

    # Hacer que Tf muestrela posicion con respecto a la base del robot y no
    # solo la base del manipulador
    Tf = 
    
    bmarker.position(Tf)
    print np.round(Tf,3)

    # ========================================================================


    # Objeto (mensaje) de tipo JointState
    jstate = JointState()
    # Asignar valores al mensaje
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    other_joints = [0.04,0.04,0.,0.,0.,0.,0.,0.]
    jstate.position = q + other_joints

    # Frecuencia del envio (en Hz)
    rate = rospy.Rate(100)
    # Bucle de ejecucion continua
    while not rospy.is_shutdown():
        # Tiempo actual (necesario como indicador para ROS)
        jstate.header.stamp = rospy.Time.now()
        # Publicar mensaje
        pub.publish(jstate)
        bmarker.publish()
        # Esperar hasta la siguiente  iteracion
        rate.sleep()
