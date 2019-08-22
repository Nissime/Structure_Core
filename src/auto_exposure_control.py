#!/usr/bin/env python


#import roslib
import math
import numpy as np
import cv2
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
from std_msgs.msg import Float32

pub = rospy.Publisher("gain", Float32MultiArray, queue_size=0)
pub1 = rospy.Publisher("ae_meanvalue", Float32, queue_size=0)
# I term for PI controller
pub2 = rospy.Publisher("ae_errp", Float32, queue_size=0)
pub3 = rospy.Publisher("ae_erri", Float32, queue_size=0)
pub_exposure = rospy.Publisher("ae_exposure", Float32, queue_size=0)

err_i = 0
exposure = 0.015
gain = Float32MultiArray()
def get_exposure(dyn_client):
    values = dyn_client.get_configuration()
    return values['exposure_value']

def set_exposure(dyn_client, exposure):
    params = {'exposure_value' : exposure}
    config = dyn_client.update_configuration(params)

def image_callback(image, args):
    global err_i
    bridge = args['cv_bridge']
    #dyn_client = args['dyn_client']
    cv_image = bridge.imgmsg_to_cv2(image,
                                    desired_encoding = "bgr8")

    (rows, cols, channels) = cv_image.shape
    if (channels == 3):
        brightness_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[:,:,2]
    else:
        brightness_image = cv_image

    #crop_size = 10
    #brightness_image = brightness_image[rows-crop_size:rows+crop_size, cols-crop_size:cols+crop_size]

    #(rows, cols) = brightness_image.shape

    hist = cv2.calcHist([brightness_image],[0],None,[5],[0,256])
    mean_sample_value = 0
    for i in range(len(hist)):
        mean_sample_value += hist[i]*(i+1)

    mean_sample_value /= (rows*cols)
    pub1.publish(mean_sample_value)
    #focus_region = brightness_image[rows/2-10:rows/2+10, cols/2-10:cols/2+10]
    #brightness_value = numpy.mean(focus_region)

    # Middle value MSV is 2.5, range is 0-5
    # Note: You may need to retune the PI gains if you change this
    desired_msv = 2.5
    # Gains
    k_p = 0.75
    k_i = 1.5
    exposure_max = 0.02
    exposure_defalut = 0.015
    gain_max = 8.0
    gain_min = 0.2
    # Maximum integral value
    max_i = 5
    min_i = 0

    err_p = desired_msv-mean_sample_value
    err_i += err_p/30 #30hz
    if abs(err_i) > max_i:
        err_i = np.sign(err_i)*max_i
    if (err_i) < min_i:
        err_i = 0
    #print err_p , err_i
    gain_calc = k_p*err_p + k_i*err_i
    if gain_calc>gain_max:
        gain_calc=gain_max

    if gain_calc < gain_min:
        gain_calc = gain_min

    exposure = (gain_calc)/gain_max*exposure_max
    if exposure<=(gain_min)/gain_max*exposure_max:
        exposure = 0.001
    print exposure
    gain.data = [gain_calc , exposure]

    pub3.publish(err_i)
    pub2.publish(err_p)
    pub.publish(gain)
    pub_exposure.publish(exposure)
    # Don't change exposure if we're close enough. Changing too often slows
    # down the data rate of the camera.
    #print k_p*err_p+k_i*err_i
    #if abs(err_p) > 0.5:
    #    set_exposure(dyn_client, get_exposure(dyn_client)+k_p*err_p+k_i*err_i)

def main(args):
    rospy.init_node('auto_exposure_control')
    bridge = CvBridge()
    #camera_name = rospy.get_param('~camera_name')
    #dyn_client = dynamic_reconfigure.client.Client(camera_name)

    params = {'auto_exposure' : False}
    #config = dyn_client.update_configuration(params)

    args = {}
    args['cv_bridge'] = bridge
    #args['dyn_client'] = dyn_client

    img_sub=rospy.Subscriber('/visible/image_raw', Image, image_callback, args)

    rospy.spin()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
