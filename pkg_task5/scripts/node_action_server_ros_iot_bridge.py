#!/usr/bin/env python

''' ROS Node - Action Server - IoT ROS Bridge'''
import threading
import rospy
import actionlib
# pylint: disable=unused-import
# pylint: disable=unused-variable
# pylint: disable=eval-used
# pylint: disable=unused-argument
# pylint: disable=too-many-instance-attributes
# Message Class used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import msgMqttSub
# Message Class used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Feedback Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback
from pkg_ros_iot_bridge.msg import order
# Custom Python Module to perform MQTT Tasks
from pyiot import iot




class IotRosBridgeActionServer(object):
    ''' This class is responsible for all action server related work
        It will handle the goals sent by the action client or MQTT and process it.
    '''
    def __init__(self):
        ''' Initialize the Action Server'''
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the function pointer which points to a function which will be
called when the Action Server receives a Goal.

            * self.on_cancel - It is the function pointer which points to a function which will be
called when the Action Server receives a Cancel Request.
        '''

        param_config_pyiot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_pyiot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_pyiot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_pyiot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_pyiot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_pyiot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_pyiot['mqtt']['sub_cb_ros_topic']
        self._config_sspreadsheet_url = param_config_pyiot['google_apps']['spread_sheet']
        print param_config_pyiot
        self.goal_id = ''
        self._var_handle_pub = rospy.Publisher('/2437/incomingorders/', order, queue_size=10)

        # Initialize ROS Topic Publication
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)


        # Subscribe to MQTT Topic (eyrc/JjAaHhBb/iot_to_ros)
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")


        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")


    def mqtt_sub_callback(self, client, userdata, message):
        '''This is a callback function for MQTT Subscriptions

           :param client: This is the client which has sent the goal
           :param userdata: The information about the action client
           :param message: The message which is being published on MQTT
        '''
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)

        payload = eval(payload)
        msg = order()
        msg.order_id = payload["order_id"]
        msg.city = payload["city"]
        msg.item = payload["item"]
        msg.qty = payload["qty"]
        msg.latitude = payload["lat"]
        msg.longitude = payload["lon"]
        msg.order_time = payload["order_time"]
        if payload["item"] == "Clothes":
            msg.priority = "LP"
            msg.cost = 150
        elif payload["item"] == "Food":
            msg.priority = "MP"
            msg.cost = 250
        elif payload["item"] == "Medicine":
            msg.priority = "HP"
            msg.cost = 450

        self._var_handle_pub.publish(msg)




        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        '''This function will be called when Action Server receives a Goal

           :param goal_handle: The goal sent by the action client will reside in this
        '''
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        if goal.protocol == "mqtt":

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()


                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        elif goal.protocol == "google_apps":
            goal_handle.set_accepted()
            thread = threading.Thread(name="worker",
                                      target=self.process_goal_1,
                                      args=(goal_handle,))
            thread.start()

        else:
            goal_handle.set_rejected()
            return


    def process_goal(self, goal_handle):
        '''This function is called in a separate thread to process Goal.
           For publishing or getting messages from MQTT this function is used.

           :param goal_handle: The goal sent by the action client will reside in this
        '''

        #flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()


        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True

                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    def process_goal_1(self, goal_handle):
        '''This function is called in a separate thread to process Goal.
           For sending data to spreadsheet this function is used

           :param goal_handle: The goal sent by the action client will reside in this
        '''

        #flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()


        # Goal Processing
        if goal.protocol == "google_apps":
            rospy.logwarn("Sending data to spreadsheet")
            data = eval(goal.message)
            iot.to_spreadsheet(self._config_sspreadsheet_url, data)
            result.flag_success = True

        rospy.loginfo("Send goal result to client")
        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")


    def on_cancel(self, goal_handle):
        '''This function will be called when Goal Cancel request is send to the Action Server'''
        rospy.loginfo("Received cancel request.")
        self.goal_id = goal_handle.get_goal_id()


def main():
    '''The driver function that will start the action server to recieve goals'''
    rospy.init_node('node_iot_ros_bridge_action_server')
    var_handle_pub = rospy.Publisher('/eyrc/vb/JjAaHhBb/orders', msgMqttSub, queue_size=10)

    action_server = IotRosBridgeActionServer()

    rospy.spin()



if __name__ == '__main__':
    main()
