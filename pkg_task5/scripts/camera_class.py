#! /usr/bin/env python
'''
The module consist of Camera1 class which is used by the 2d Camera of warehouse
'''

import rospy

# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
# pylint: disable=pointless-string-statement
from hrwros_gazebo.msg import *
from geometry_msgs.msg import *
from pkg_vb_sim.srv import *


from pyzbar.pyzbar import decode
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2




class Camera1(object):
    """This class is dedicated for the image processing of the images recieved by
       the 2D camera of the Warehouse
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.image = np.zeros((300, 300, 3), np.uint8)
        self.empty_list = []

    def get_qr_data(self, arg_image):
        """This method will deode the QR code present in the image.


           :param arg_image: The image in which QR code is present

           :return: The decoded info about the QR code in the cropped image
           :rtype: element of :class:'pyzbar.pyzbar.Decoded' object
        """

        """The method will use decode function of module pyzbar.pyzbar to decode the QR code"""
        qr_result = decode(arg_image)

        if qr_result != self.empty_list:
            return qr_result[0]
        return 'NA'


    def callback(self, data):
        '''This is the function callback for the subscriber of the topic
           /eyrc/vb/camera_1/image_raw

           :param data: This is the data which is published on the topic /eyrc/vb/camera_1/image_raw
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as _e:
            rospy.logerr(_e)

        self.image = cv_image



    def decode_qr_on_package(self, row_number, col_number):
        '''This method will crop the original image and will add contrast to the image
           so that the image becomes clear for decoding QR code

           :param row_number: Specifies the pkg row no. whose QR code is to be decoded
           :type row_number: int
           :param col_number: Specifies the pkg col no. whose QR code is to be decoded
           :type col_number: int


           :return: This function will return required decoded information of the cropped image.
           :rtype: element of :class: 'pyzbar.pyzbar.Decoded' object
        '''
        #defining percentages for cropping the image
        crop_row = [0.20, 0.35, 0.48, 0.60, 0.72]
        crop_col = [0.10, 0.40, 0.65, 1.00]
        height, width = self.image.shape[0:2]
        start_r = int(height * crop_row[row_number])
        start_c = int(width * crop_col[col_number])
        end_r = int(height * crop_row[row_number + 1])
        end_c = int(width * crop_col[col_number + 1])

        #adding contrast to the image
        contrast_img = cv2.addWeighted(self.image, 2.5, \
        np.zeros(self.image.shape, self.image.dtype), 0, 0)

        #cropping the image
        cropped_image = contrast_img[start_r:end_r, start_c:end_c]


        #getting the qr data from the cropped image of the package qr code
        result = self.get_qr_data(cropped_image)
        cv2.waitKey(3)

        return result
