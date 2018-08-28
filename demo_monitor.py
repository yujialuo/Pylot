"""
This is a demo of how to use the monitor
"""

from dos.envs.vehicle_env import VehicleEnv
from dos.op import Op, make_op
from dashboard import Monitor


def replay_left_camera():
    import rosbag
    bag = rosbag.Bag('test.bag')
    left_topic = ["/usb_cam1/image_raw/compressed"]
    for topic, msg, t in bag.read_messages(topics=left_topic):
        yield msg


def replay_right_camera():
    import rosbag
    bag = rosbag.Bag('test.bag')
    right_topic = ["/usb_cam3/image_raw/compressed"]
    for topic, msg, t in bag.read_messages(topics=right_topic):
        yield msg


class MonitorOp(Op):
    def __init__(self):
        super(MonitorOp, self).__init__(name="dashboard")
        self.monitor = Monitor()
        self.register(self.monitor.camera_callback)

    def init(self):
        for _ in range(self.in_num):
            self.monitor.add("camera")

    def spin(self):
        self.monitor.run()


if __name__ == '__main__':

    # Build Graph
    camera_op = make_op((replay_left_camera, replay_right_camera),
                        output_types=(list, list),
                        output_names=('/image_sender_0', 'image_sender_1'),
                        name="camera")
    monitor_op = MonitorOp()

    # Connect Graph
    image1, image2 = camera_op()
    monitor_op(image1, image2)

    # Initiate Env & Run Graph
    with VehicleEnv() as env:
        env.run()
