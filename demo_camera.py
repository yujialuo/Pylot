
from dos.envs.vehicle_env import VehicleEnv
from dos.op import make_op
from dos.op_wrapper import OpWrapper


def preprocess(data):
    return data


if __name__ == '__main__':

    # Build Graph
    camera_op = OpWrapper(path='~/driving-dev/alpha/yucong-he',
                          package='bfsdriver_1_1',
                          executable='bfsdriver_1_1_trigger_publisher',
                          output_names=("/image_sender_0", "/image_sender_1"),
                          output_types=(list, list))

    image_processor_op = make_op(preprocess, name="image_processor")

    # Connect Graph
    image = camera_op()
    image_processor_op(image)

    # Initiate Env & Run Graph
    with VehicleEnv() as env:
        env.run()
