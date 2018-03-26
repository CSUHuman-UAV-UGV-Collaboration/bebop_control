# telnet based
# TODO: instead of using select to monitor the socket and
# stdin streams we need to use ros to monitor msgs,
# maybe need multithreading

import socket, select, string, sys
import rospy
import threading
from std_msgs.msg import Empty

class InterfaceClientDrone():
    def __init__(self, host, port):
        # initialize vars and ros
        self.host = host
        self.port = port
        self.message = '' # drone state to send through interface
        self.disconnect = False

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Interface Client running...")

        # connect to server
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.settimeout(2)

        # connect to remote host
        try:
            self.s.connect((host, port))
        except:
            print 'Unable to connect'
            sys.exit()

        print 'Connected to remote host. Start sending messages'
        self.prompt()

        # start the socket handler thread
        socket_thread = threading.Thread(target=self.socket_handler)
        socket_thread.start()

        # subscribers, TODO: change from empty msg to drone states
        rospy.Subscriber('drone_states', Empty, self.states_callback)

        # publishers, TODO: change from empty msg to turtle states
        self.pub_turtle_states = rospy.Publisher('turtle_states', Empty, queue_size=1, latch=True)


    def shutdown(self):
        # do cleanup here if necessary
        rospy.loginfo("Interface Client Shutting down...")
        self.disconnect = True


    def states_callback(self, data):
        # get drone state and prepare to send
        # self.message = str(data.drone_state)
        self.message = 'test'
        # get the readable sockets
        self.s.send(self.message)
        self.prompt()


    def prompt(self):
        sys.stdout.write('<My State> ')
        sys.stdout.flush()


    def socket_handler(self):
        host = self.host
        port = self.port

        while True:
            socket_list = [self.s]

            read_sockets, write_sockets, error_sockets = select.select(socket_list, [], [])

            for sock in read_sockets:
                # incoming messages from remote server
                if sock == self.s:
                    data = sock.recv(4096)
                    if not data:
                        print "\n disconnected from chat server"
                        sys.exit()
                    else:
                        sys.stdout.write(data)
                        self.prompt()
                        # publish here
                        data = Empty()
                        self.pub_turtle_states.publish(data)   

            if self.disconnect == True:
                sys.exit(0)



if __name__ == "__main__":
    rospy.init_node('interface_drone_client')
    try:
        if(len(sys.argv) < 3):
            print 'Usage python interface_tcp_client.py hostname port'
            sys.exit()

        host = sys.argv[1]
        port = int(sys.argv[2])

        InterfaceClientDrone(host, port)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interface Client ROS node terminated.")

    

