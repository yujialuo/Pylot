"""
This is a demo of how to use the interface
"""

from time import sleep
from dos.envs.vehicle_env import VehicleEnv
from dos.op import make_op


def generate_images():
    i = 0
    while True:
        data = "data {}".format(i)
        yield data
        sleep(1)
        i += 1


def model_predict(image):
    return "action " + image.data


def execute_control(control):
    print("Execute {}".format(control.data))


if __name__ == '__main__':

    # Build Graph
    camera_op = make_op(generate_images, output_types=str, name='camera')
    model_op = make_op(model_predict, output_types=str, name='model')
    control_op = make_op(execute_control, freq=0.2, name='control')

    # Connect Graph
    image = camera_op()
    prediction = model_op(image)
    control_op(prediction)

    # Initiate Env & Run Graph
    with VehicleEnv() as env:
        env.run()
