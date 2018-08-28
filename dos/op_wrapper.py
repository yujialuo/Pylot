
import os
from op import Op
from subprocess import call


class OpWrapper(Op):
    def __init__(self, path, package, executable,
                 output_names=None, output_types=None):
        super(OpWrapper, self).__init__(None,
                                        output_names=output_names,
                                        output_types=output_types,
                                        name=package)
        self.path = path
        self.package = package
        self.executable = executable

    def run(self):
        self._run_ros_package()

    def _run_ros_package(self):
        """
        Run ROS Package
        Example:
            cd ~/driving-dev/alpha/yucong-he
            catkin_make
            source devel/setup.bash
            rosrun bfsdriver_1_1 bfsdriver_1_1_trigger_publisher
        """
        os.chdir(os.path.expanduser(self.path))
        call("catkin_make")
        os.system("source devel/setup.bash")
        os.system("rosrun {} {}".format(self.package, self.executable))
