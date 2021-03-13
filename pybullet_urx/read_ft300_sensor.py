import rospy
from std_msgs.msg import Float32
import socket

class socket_connection():
    def __init__(self,  publisher_object, rate, ip, port):
        self.host_ip = ip
        self.port = port
        try:
            self.socket_object = socket.create_connection((self.host_ip, self.port), timeout=0.5)
        except Exception as e:
            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))
        self._dataqueue = bytes()
        self._trystop = False
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False

    def _get_data(self):
        while True:
            tmp = self.socket_object.recv(2048)
            return tmp

    def processData(self):
        data = self._get_data()
        # print(data.decode("utf-8"))
        denemeS = data.decode('utf-8')
        denemeFx = float(denemeS.split(',')[0].split('(')[1])
        denemeFy = float(denemeS.split(",")[1])
        denemeFz = float(denemeS.split(",")[2])
        denemeMx = float(denemeS.split(",")[3])
        denemeMy = float(denemeS.split(",")[4])
        denemeMz = float(denemeS.split(",")[5].split(')')[0])
        ForceData = [denemeFx, denemeFy, denemeFz, denemeMx, denemeMy, denemeMz]
        rospy.loginfo(ForceData)
        self.publisher_object.publish(ForceData)
        self.publisher_rate.sleep()

    def run(self):
        try:
            while not self._trystop:
                self.processData()
        except KeyboardInterrupt:
            self.socket_object.close()
            return False

def main():
    pub = rospy.Publisher('ft300_force_torque', Float32, queue_size=10)
    rospy.init_node('torque_force_sensor_data', anonymous=True)
    rate = rospy.Rate(10)
    socket_connection_obj = socket_connection(pub, rate, "192.168.56.1", 63351)
    socket_connection_obj.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass