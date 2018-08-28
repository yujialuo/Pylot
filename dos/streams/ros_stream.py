
import rospy
import std_msgs.msg as std
import sensor_msgs.msg as sensor

from dos.streams.stream import Stream


ROS_TYPE_MAP = {
    str: std.String,
    list: sensor.CompressedImage
}


def to_ros_type(stream_type):
    try:
        return ROS_TYPE_MAP[stream_type]
    except KeyError:
        raise ValueError('Unsupported ros stream type ' + str(stream_type))


class RosStream(Stream):
    def __init__(self, name, data_type):
        if name[0] != "/":  # ROS topic has a slash in front
            name = "/" + name
        super(RosStream, self).__init__(name, data_type)

    def init_sender(self):
        pub = rospy.Publisher(self.name, to_ros_type(self.data_type),
                              queue_size=10)
        self.sender = pub
        print("[Publish to] {}".format(self.name))

    def init_receiver(self):
        # wait for the topic to be published
        while not self._topic_exist(self.name):
            print('wait for the topic {} to be published'.format(self.name))
        rospy.Subscriber(self.name, to_ros_type(self.data_type),
                         callback=self.callback,
                         callback_args=self.callback_index)
        print("[Subscribe to] {}".format(self.name))

    def add(self, data):
        self.sender.publish(data)
        print("-- publish {} to {}".format("data", self.name))

    def _topic_exist(self, topic):
        topics = rospy.get_published_topics()
        for item in topics:
            if topic == item[0]:
                return True
        return False
