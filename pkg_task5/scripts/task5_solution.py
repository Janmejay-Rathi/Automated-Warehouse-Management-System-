#! /usr/bin/env python
'''
This module contains the functions that are required for the simultaneous working of two ur5 arms.
This is required module for warehouse automation
The data structure/algorithm used for sorting the packages is PRIORITY QUEUES
'''


from datetime import datetime
from datetime import timedelta
from datetime import date

#This disable statements are used because we were unable to solve this import errors
# pylint: disable=ungrouped-imports
# pylint: disable=wildcard-import
# pylint: disable=unused-wildcard-import
# pylint: disable=unused-import

import copy
import os
import math
import time
import sys
import threading

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import yaml


from hrwros_gazebo.msg import *
from geometry_msgs.msg import *
from pkg_vb_sim.srv import *
from pkg_ros_iot_bridge.msg import order

from std_srvs.srv import Empty
from action_client import IotRosBridgeActionClient
from order_data import OrderData
from ur5_classes import Ur51
from ur5_classes import Ur52
from camera_class import Camera1


class Warehouse(object):
    """This class is the representation of the automated warehouse.
       It has capability of controlling the two ur5 arms and processing the orders.
    """

    def __init__(self):
        self.ur5_1 = Ur51()
        self.ur5_2 = Ur52()
        self.action_client = IotRosBridgeActionClient()
        self._conveyor_flag = False
        self._orders_hp = list()
        self._orders_mp = list()
        self._orders_lp = list()
        self._dispatched_order = list()
        self.colors = dict()
        self.colors_copy = dict()
        self.param_config_gripper = rospy.get_param('vacuum_gripper_plugin_in_usage')


        self.drop = [math.radians(8),
                     math.radians(-133),
                     math.radians(-70),
                     math.radians(-67),
                     math.radians(91),
                     math.radians(0)]

        self.home_pose = [math.radians(180),
                          math.radians(-49),
                          math.radians(63),
                          math.radians(-104),
                          math.radians(-90),
                          math.radians(0)]

        self.pick = [math.radians(8),
                     math.radians(-132),
                     math.radians(-52),
                     math.radians(-81),
                     math.radians(90),
                     math.radians(0)]


        self.red_bin = [math.radians(-85),
                        math.radians(-140),
                        math.radians(-52),
                        math.radians(-81),
                        math.radians(90),
                        math.radians(0)]

        self.yellow_bin = [math.radians(-172),
                           math.radians(-140),
                           math.radians(-52),
                           math.radians(-81),
                           math.radians(90),
                           math.radians(0)]

        self.green_bin = [math.radians(90),
                          math.radians(-140),
                          math.radians(-52),
                          math.radians(-81),
                          math.radians(90),
                          math.radians(0)]



    def func_callback_incomingorders(self, message):
        '''
        Function callback for the subscriber of topic /2437/incomingorders

        :param message: The message that is being published on the topic /2437/incomingorders
        :type: class: 'pkg_ros_iot_bridge.msg._order.order'

        '''

        if message.priority == "HP":
            self._orders_hp.append(message)
        elif message.priority == "MP":
            self._orders_mp.append(message)
        elif message.priority == "LP":
            self._orders_lp.append(message)
        rospy.loginfo("Order Received")

        new_order1 = OrderData(message, "IncomingOrders")
        new_order2 = OrderData(message, "Dashboard")
        new_order2.data["Quantity"] = message.qty
        self.action_client.send_goal("google_apps", "push", \
        self.action_client.config_mqtt_pub_topic, str(new_order2.data))

        self.action_client.send_goal("google_apps", "push", \
        self.action_client.config_mqtt_pub_topic, str(new_order1.data))


    def get_row_col(self, val):
        '''To get the row and col number. of a particular pkg

        :param val: The key in the colors dictionary (package name)
        :type: str

        :return: returns row no. and col no. of the required package.
        :rtype: int

        '''
        for key, value in self.colors.items():
            if val == value:
                row = int(key[8])
                col = int(key[9])
                del self.colors[key]
                return row, col



    def get_order(self):
        '''To get new order from the orders that are being placed in the queue

           :return: This method returns the order with highest priority that is currently \
           present in the queues
           :rtype: 'pkg_ros_iot_bridge.msg._order.order'

        '''
        empty_list = list()
        while True:
            if self._orders_hp != empty_list:
                new = self._orders_hp[0]
                self._orders_hp.pop(0)
                return new

            elif self._orders_mp != empty_list:
                new = self._orders_mp[0]
                self._orders_mp.pop(0)
                return new

            elif self._orders_lp != empty_list:
                new = self._orders_lp[0]
                self._orders_lp.pop(0)
                return new



    def process_ur5_1(self):
        '''
        This function is dedicated for the proper functioning of ur51

        1. The high priority package among all present in the priority queue is returned\
        by get_order() function and saved in variable current_order for processing it.
        2. Appending the self._dispatched_order dictionary with current_order so that the\
        the process_ur5_2() function can ship the order that is being dispathced.
        3. Checking the shelf to get the row number and column number of the package type \
        described in the order.
        4. Setting the index for using waypoint and gripper lists
        5. Sending data to Dashboard and OrdersDispatched spreadheet tab
        6. After placing the order on the conveyor checking the status of conveyor_flag\
        which specifies that whether the previous order is being picked by ur5_2 or not.

        '''

        waypoints = [5, 4, 3, 2, 2, 2, 4, 2, 2]
        gripper = [2, 1, 1, 1, 1, 1, 1, 1, 1]
        index = 0

        self.ur5_1.hard_set_joint_angles(self.home_pose, 1)

        for _ in range(0, 9):


            current_order = self.get_order()

            self._dispatched_order.append(current_order)


            row_no, col_no = self.get_row_col(current_order.item)
            rospy.loginfo(current_order)


            rospy.loginfo('....'+str(row_no)+str(col_no)+'......')
            file_path = self.ur5_1.file_path + 'p' + str(row_no) + str(col_no) + '/'


            index = 3*row_no + col_no

            for waypoint_no in range(waypoints[index]):


                self.ur5_1.moveit_hard_play_planned_path_from_file(file_path, \
                      'pkg_' + str(row_no) + str(col_no) + '_waypoint_' + str(waypoint_no+1)\
                       + '.yaml', 5)

                if waypoint_no+1 == gripper[index]:
                    while self.param_config_gripper:
                        pass
                    self.ur5_1.activate_gripper(True)

            while self.param_config_gripper:
              pass
            self.ur5_1.activate_gripper(False)
            d_1 = date.today().strftime("%d/%m/%Y")
            current_time = datetime.now().strftime("%H:%M:%S")

	    while self._conveyor_flag:
                pass
            self.ur5_1.activate_conveyor(100)
            self._conveyor_flag = True

            new_order = OrderData(current_order, "OrdersDispatched")
            new_order.data["Dispatch Status"] = "YES"
            new_order.data["Dispatch Quantity"] = 1
            new_order.data["Dispatch Date and Time"] = d_1 + " " + current_time
            self.action_client.send_goal("google_apps", "push", \
            self.action_client.config_mqtt_pub_topic, str(new_order.data))
            time.sleep(1)
            data1 = dict()
            data1["id"] = "Dashboard"
            data1["Order ID"] = current_order.order_id
            data1["Order Dispatched"] = "YES"
            data1["Dispatch Time"] = new_order.data["Dispatch Date and Time"]
            self.action_client.send_goal("google_apps", "push", \
            self.action_client.config_mqtt_pub_topic, str(data1))







# pylint: disable=too-many-statements
    def process_ur5_2(self):
        '''
        This function is dedicated for the proper functioning of ur52

        1. Ur52 will wait for the package to come under logical_camera_2.
        2. Once package is detected the self._dispatched_order queue is used to get\
        the order which needs to be shipped.
        3. Getting the type of the item that package signifies
        4. Once the package is picked by ur52, conveyor_flag status is set so that for \
        next order the conveyor can be activated.
        5. On the basis of the package type, dropping package into its respective bin
        6. Sending data into OrdersShipped and Dashboard spreadsheet.
        '''


        rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, \
        self.ur5_2.function_callback)

        self.ur5_2.hard_set_joint_angles(self.drop, 1)
        j = 0


        while not (rospy.is_shutdown() or j == 9):

            if len(self.ur5_2.box_detected) == j+1:


                current_order = self._dispatched_order.pop(0)
                rospy.loginfo(self.ur5_2.pkg_under_camera)

                pkg_to_pick = self.colors_copy[self.ur5_2.pkg_under_camera]

                while self.param_config_gripper:
                    pass
                self.ur5_2.activate_gripper(True)

                self.ur5_2.hard_set_joint_angles(self.pick, 1)


                self._conveyor_flag = False


                if pkg_to_pick == 'Medicine':
                    self.ur5_2.hard_set_joint_angles(self.red_bin, 1)

                elif pkg_to_pick == 'Food':
                    self.ur5_2.hard_set_joint_angles(self.yellow_bin, 1)

                elif pkg_to_pick == 'Clothes':
                    self.ur5_2.hard_set_joint_angles(self.green_bin, 1)


                while self.param_config_gripper:
                    pass
                self.ur5_2.activate_gripper(False)



                shipped_order = OrderData(current_order, "OrdersShipped")
                shipped_order.data["Shipped Quantity"] = current_order.qty
                shipped_order.data["Shipped Status"] = "YES"
                shipped_order.data["Shipped Date and Time"] = \
                date.today().strftime("%d/%m/%Y") + " " + datetime.now().strftime("%H:%M:%S")

                if shipped_order.data["Priority"] == "HP":
                    end_date = date.today() + timedelta(days=1)

                elif shipped_order.data["Priority"] == "MP":
                    end_date = date.today() + timedelta(days=3)

                elif shipped_order.data["Priority"] == "LP":
                    end_date = date.today() + timedelta(days=5)

                shipped_order.data["Estimated Time of Delivery"] = end_date.strftime("%Y/%m/%d")

                self.action_client.send_goal("google_apps", "push", \
                self.action_client.config_mqtt_pub_topic, str(shipped_order.data))


                order_time_of_pkg = datetime.strptime(current_order.order_time, '%Y-%m-%d %H:%M:%S')
                shipping_time_of_pkg = datetime.strptime(\
                shipped_order.data["Shipped Date and Time"], '%d/%m/%Y %H:%M:%S')

                time_taken = (shipping_time_of_pkg - order_time_of_pkg).total_seconds()


                data2 = dict()
                data2["id"] = "Dashboard"
                data2["Time Taken"] = time_taken
                data2["Order ID"] = current_order.order_id
                data2["Order Shipped"] = "YES"
                data2["Shipping Time"] = shipped_order.data["Shipped Date and Time"]

                self.action_client.send_goal("google_apps", "push", \
                self.action_client.config_mqtt_pub_topic, str(data2))

                self.ur5_2.hard_set_joint_angles(self.drop, 1)

                j = j+1



def init_package_data(data, color, row_number, col_number):
    '''
       This function will initialize the data about the package which is on shelf

       :param data: dictionary to store information about the package
       :param color: color of the packages
       :param row_number: row number of the package
       :param col_number: col number of the package

       :return: returns the information about the specified package on the shelf
       :rtype: dictionary
    '''
    data["id"] = "Inventory"
    data["Team Id"] = "VB#2437"
    data["Unique Id"] = "JjAaHhBb"
    data["Quantity"] = 1

    #for getting the SKU format of month and year
    datetime_object = datetime.now()
    year = datetime_object.year
    product_ = 1
    sum_ = 0
    for _ in range(2):
        digit = year % 10
        sum_ = sum_ + product_*digit
        product_ = product_*10
        year = year//10

    month = ""
    if datetime_object.month < 10:
        month = month + "0"
    month = month + str(datetime_object.month)


    if color == "red":
        data["Item"] = "Medicine"
        data["Priority"] = "HP"
        data["Cost"] = 450
        data["SKU"] = "R" + str(row_number) + str(col_number) + month + str(sum_)
        data["Storage Number"] = "R" + str(row_number) + " C" + str(col_number)
    elif color == "yellow":
        data["Item"] = "Food"
        data["Priority"] = "MP"
        data["Cost"] = 250
        data["SKU"] = "Y" + str(row_number) + str(col_number) + month + str(sum_)
        data["Storage Number"] = "R" + str(row_number) + " C" + str(col_number)
    elif color == "green":
        data["Item"] = "Clothes"
        data["Priority"] = "LP"
        data["Cost"] = 150
        data["SKU"] = "G" + str(row_number) + str(col_number) + month + str(sum_)
        data["Storage Number"] = "R" + str(row_number) + " C" + str(col_number)

    return data


def main():
    '''
       This is the main driver function. The auto_warehouse is the object that is being created by\
       using the class Warehouse() to represent the automated warehouse.

       1. Creating Subscriber of /2437/incomingorders/ topic to store the incoming orders

       2. Getting the decoded info about QR code present on the package at \
       location [row_number, col_number] on the shelf and append it into auto_warehouse.colors dict.

       3. Once the QR code is decode sending data into Inventory spreadsheet.

       4. After processing the QR codes on the packages, two threads are initiated \
       i.e. auto_warehouse.process_ur5_1 and auto_warehouse.process_ur5_2 to start\
       functioning of the two ur5 arms simultaneously

    '''

    rospy.sleep(10)
    rospy.init_node('node_task5_solution', anonymous=True)
    auto_warehouse = Warehouse()



    rospy.Subscriber("/2437/incomingorders/", order, auto_warehouse.func_callback_incomingorders)

    i_c = Camera1()
    rospy.sleep(2)
    row_number = 0
    col_number = 0


    for i in range(12):
        data = dict()


        result = i_c.decode_qr_on_package(row_number, col_number)
        rospy.loginfo(result.data)
        data = init_package_data(data, result.data, row_number, col_number)

        if i < 9:
            auto_warehouse.colors['packagen' + str(row_number)+ str(col_number)] = data["Item"]


        auto_warehouse.action_client.send_goal("google_apps", "push", \
        auto_warehouse.action_client.config_mqtt_pub_topic, str(data))

        rospy.sleep(2)
        del data

        col_number = col_number + 1

        if col_number > 2:
            row_number = row_number + 1
            col_number = 0

    print auto_warehouse.colors
    auto_warehouse.colors_copy = auto_warehouse.colors.copy()


    t_1 = threading.Thread(target=auto_warehouse.process_ur5_1, args=())
    t_2 = threading.Thread(target=auto_warehouse.process_ur5_2, args=())

    t_1.start()
    t_2.start()

    t_1.join()
    t_2.join()

    rospy.sleep(2)





if __name__ == '__main__':
    main()
