import threading, socket, sys, rospy
from std_msgs.msg import Empty, String
from botsapp.msg import DroneStates, TurtleStates, ResourceString

class Client(object):
    def __init__(self, host, port):

        # initialize vars
        self.sending = False

        # initialize ros
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Interface Client Running")

        # server socket connect
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      
        try:
            self.sock.connect((host,port))
        except:
            print 'Unable to connect'
            sys.exit()

        # listen daemon (thread)
        listen_thread = threading.Thread(target=self.listen)
        listen_thread.daemon = True
        listen_thread.start()

        # subscribers
        rospy.Subscriber('drone_states', DroneStates, self.states_callback)
        rospy.Subscriber('drone_response', String, self.response_callback)
        rospy.Subscriber('drone_request', String, self.drone_request_callback)
        rospy.Subscriber('turtle_request', String, self.turtle_request_callback)

        # publishers
        self.pub_turtle_states = rospy.Publisher('turtle_states', TurtleStates, queue_size=1)
        self.pub_turtle_response = rospy.Publisher('turtle_response', String, queue_size=1)
        #self.pub_turtle_request = rospy.Publisher('turtle_request', String, queue_size=1)


    def shutdown(self):
        # do cleanup here if necessary
        rospy.loginfo("Interface Client Shutting down")
        self.disconnect = True


    def states_callback(self, data):
        # get drone state and prepare to send
        message = 'drone_states ' + str(data.DroneState)
        self.sending = True
        self.send(message)
        rospy.loginfo("Sent drone state")


    def response_callback(self, data):
        # get response after a task is done from the turtlebot and send
        message = 'drone_response ' + str(data.data)
        self.sending = True
        self.send(message)
        rospy.loginfo("Sent drone response")


    def drone_request_callback(self, data):
        # get request from robotcollab and send
        message = 'drone_request ' + str(data.data)
        self.sending = True
        self.send(message)
        rospy.loginfo("Sent drone request")


    def turtle_request_callback(self, data):
        # get request from robotcollab and send
        message = 'turtle_request ' + str(data.data)
        self.sending = True
        self.send(message)
        rospy.loginfo("Sent turtle request")


    def parse_publish(self, message):
        parts = message.strip().split()
        # maybe use resource strings here
        # parse which message type it is and publish
        # add more message types here
        if parts[0] == 'turtle_states':
            msg = TurtleStates()
            msg.BotState = int(parts[1])
            self.pub_turtle_states.publish(msg)
        elif parts[0] == 'turtle_response':
            msg = String()
            msg.data = parts[1]
            self.pub_turtle_response.publish(msg)
        else:
            rospy.loginfo("Invalid message recieved. Ignoring.")


    def send(self, message):
        if self.sending == True:
            self.sending = False
            msg = message
            self.sock.send(msg)


    def listen(self):
        while True:
            data = self.sock.recv(1024)
            if not data:
                print "Connection lost"
                sys.exit(0)

            print "Recieved ",data
            self.parse_publish(data)
            

if __name__ == "__main__":
    try:
        rospy.init_node('interface_drone_client')
        if(len(sys.argv) < 3):
            print 'Usage python interface_tcp_client.py hostname port'
            sys.exit()

        host = sys.argv[1]
        port = int(sys.argv[2])

        Client(host,port)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interface Client ROS node terminated.")

