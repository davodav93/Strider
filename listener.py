#!/usr/bin/env python
#coding=utf-8
import roslib; roslib.load_manifest('strider')
import rospy
from strider.msg import Mensaje

def msgCallback(data):
  rospy.loginfo(rospy.get_name()+" Velocidad: %.2f m/s", data.vel)
  rospy.loginfo(rospy.get_name()+" Voltaje de la batería: %.2f V.", data.volt)
  rospy.loginfo(rospy.get_name()+" Ángulo del timón: %d º", data.ang)  
  rospy.loginfo(rospy.get_name()+" Aceleración: %.3f %.3f %.3f m/s²", data.acel.x, data.acel.y, data.acel.z)
  rospy.loginfo(rospy.get_name()+" Distancias de los sensores: %.0fcmN  %.0fcmS  %.0fcmE %.0fcmO", data.dist.r, data.dist.a, data.dist.b, data.dist.g)
  rospy.loginfo(rospy.get_name()+" Orientación: %.2f º", data.bruj)
  rospy.loginfo(rospy.get_name()+" Posición GPS: %.5f N, %.5f E", data.lati, data.longi) 
  rospy.loginfo(rospy.get_name()+" Altitud: %.f metros", data.alti) 
  print( )

def listener():
    rospy.Subscriber("strider", Mensaje, msgCallback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener()
