
from dos.envs.env import Env


class VehicleEnv(Env):
    def __init__(self):
        super(VehicleEnv, self).__init__()
        # Set up envrionment
        # ...

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        return
