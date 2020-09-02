#! /usr/bin/env python

from __future__ import print_function
import rospy
from man_msgs.srv import Help, HelpResponse
import os
import zmq
import json
import sys
import threading
import time


def tprint(msg):
    """like print, but won't get newlines confused with multiple threads"""
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()


class ClientTask():
    """ClientTask"""
    _server = "127.0.0.1"
    _port_server = "1668"

    
    def __init__(self, id = 1, server = None, port_server = None):
        self.id = id
        #threading.Thread.__init__ (self)
        
        BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if port_server != None:
            self._port_server = port_server
        if server != None:
            self._server = server
        self.link = 'tcp://'+ self._server + ':' + self._port_server

    def run(self, send = False):
        context = zmq.Context()
        socket = context.socket(zmq.DEALER)
        identity = u'worker-%d' % self.id
        socket.identity = identity.encode('ascii')
        socket.connect(self.link)
        print('Client %s started' % (identity))
        poll = zmq.Poller()
        poll.register(socket, zmq.POLLIN)
        reqs = 0
        
        while send == False:
            #reqs = reqs + 1
            reqs = "{\"waitforhelp\":\"true\"}"
            print('Req #%s sent..' % (reqs))
            socket.send_json(reqs)
            #sockets = dict(poll.poll())
            if socket:
                msg = socket.recv()
                tprint('Client %s received: %s' % (identity, msg))

                msg_to_str = msg.decode('utf8').replace("'", '"')
                msg_json = json.loads(msg_to_str)
                if msg_json["help_state"] == "true":
                    send = True
            rospy.Rate(10).sleep()
        socket.close()
        context.term()
        return send


class Help_Server_Node:

    # initial help_server_node
    def help_server_init(self):
        rospy.init_node('help_server')
        #callback_lambda = lambda x: get_help(x, socket)
        s = rospy.Service('help_server', Help, self.get_help)
        print("start help_server node")
    
    # handle request
    # req: "needhelp: true"
    # do something: sending zmq request to zmq server
    def get_help(self, req):
        #zmq_client = Zmq_Client()
        # zmq request
        # zmq response
        if req.needhelp == True:
            client = ClientTask(1)
            finish = client.run()
            result = True
        #print("Execution Help Solution [%b]"%(result))
            return HelpResponse(result)
        return HelpResponse(False)
 



# def serial_connection():
#     print("serial connection")
#     if not rospy.is_shutdown():      
#         if ser.isOpen():
#             print ("Port Open")
#             for i in range(1):               
#                 ser.write(bytes(sending_data) + '\n')
#                 time.sleep(0.001)

# def main():
#     """main function"""
#     #server = ServerTask()
#     #server.start()

#     client = ClientTask(1)
#     client.start()

if __name__ == '__main__':
    #zmq_client = Zmq_Client()
    #sock_zmq = zmq_client.on_connection()

    help_server = Help_Server_Node()
    help_server.help_server_init()
    rate = rospy.Rate(100)
    rospy.spin()
