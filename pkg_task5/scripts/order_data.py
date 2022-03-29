'''
This module contains a utility class that will be required when the data is \
being sent to the action server for pushing it to the spreadsheet
'''

class OrderData(object):
    '''
    This class consist of the data dictionary which is required as data field
    for sending information to action server for pushing it to the spreadsheet

    :param msg: The order msg that is recieved by the subscriber of topic /2437/incomingorders
    :type msg: class: 'pkg_ros_iot_bridge.msg._order.order'
    :param sheet_id: The tab id of the google spreadsheet in which data is to be sent
    :type sheet_id: str

    '''
    def __init__(self, msg, sheet_id):
        '''constructor'''

        self.data = dict()
        self.data["id"] = sheet_id
        self.data["Team Id"] = "VB#2437"
        self.data["Unique Id"] = "JjAaHhBb"
        self.data["Order ID"] = msg.order_id
        self.data["Order Date and Time"] = msg.order_time
        self.data["Order Time"] = self.data["Order Date and Time"]
        self.data["Item"] = msg.item
        self.data["Order Quantity"] = msg.qty
        self.data["City"] = msg.city
        self.data["Latitude"] = msg.latitude
        self.data["Longitude"] = msg.longitude
        self.data["Priority"] = msg.priority
        self.data["Cost"] = msg.cost

    def print_data(self):
        '''to print the data'''
        print self.data


    def set_id(self, sheet_id):
        '''For replacing the current sheet_id with another sheet id

           :param sheet_id: The new tab id of the google spreadsheet in which data is to be sent
           :type sheet_id: str

        '''
        self.data["id"] = sheet_id
