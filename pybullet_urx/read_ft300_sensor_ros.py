import rospy
from std_msgs.msg import Float32
import socket

class socket_connection():
    def __init__(self, publisher_object, rate, ip, port):
        try:
            self.host_ip = ip
            self.port = port
            self.output_file_location = "None"
            self.write_object = True

            if self.output_file_location is "":
                self.output_file_location = ""
            elif self.output_file_location is "None":
                self.write_object = False
            else:
                self.output_file_location = self.output_file_location + "/"
        except:
            print("Warning: Config.ini file problem, please check Config.ini")

        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False

    def connect(self):
        global f
        try:
            print("Warning: Connecting to ur3 ip adress: " + self.host_ip)
            self.socket_object.connect((self.host_ip, self.port))

            if self.write_object is True:
                print("Warning: File Write Location: " + self.output_file_location)
                f = open(self.output_file_location + "DataStream.csv", "w")
            try:
                print("Writing in DataStream.csv, Press ctrl + c to stop")
                while (1):
                    deneme = self.socket_object.recv(1024)
                    denemeS = deneme.decode('utf-8')
                    denemeFx = float(denemeS.split(",")[0].split('(')[1])
                    denemeFy = float(denemeS.split(",")[1])
                    denemeFz = float(denemeS.split(",")[2])
                    denemeMx = float(denemeS.split(",")[3])
                    denemeMy = float(denemeS.split(",")[4])
                    denemeMz = float(denemeS.split(",")[5].split(')')[0])
                    ForceData = [denemeFx,denemeFy,denemeFz,denemeMx,denemeMy,denemeMz]
                    rospy.loginfo(ForceData)
                    self.publisher_object.publish(ForceData)
                    self.publisher_rate.sleep()

            except KeyboardInterrupt:
                f.close()
                self.socket_object.close()
                return False

        except Exception as e:
            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))
            return False

def main():
    pub = rospy.Publisher('ft300_force_torque', Float32, queue_size=10)
    rospy.init_node('torque_force_sensor_data', anonymous=True)
    rate = rospy.Rate(10)
    socket_connection_obj = socket_connection(pub,rate,"192.168.56.1", 63351)
    socket_connection_obj.connect()

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass