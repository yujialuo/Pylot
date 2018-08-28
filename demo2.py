"""
This is a demo of how to use the interface
"""

from dos.envs.vehicle_env import VehicleEnv
from dos.op import make_op


def generate_left_images():
    from time import sleep
    i = 0
    while True:
        left_image = "left image {}".format(i)
        yield left_image
        sleep(1)
        i += 1


def generate_right_images():
    from time import sleep
    i = 0
    while True:
        right_image = "right_image {}".format(i)
        yield right_image
        sleep(1)
        i += 1


def model_predict(image, i):
    return "action {} from camera {}".format(image.data, i)


def model2_predict(left_image, right_image):
    return "action {} and {}".format(left_image.data, right_image.data)


def execute_control(action):
    print("Execute {}".format(action.data))


if __name__ == '__main__':

    # Build Graph
    camera_op = make_op((generate_left_images, generate_right_images),
                        output_types=(str, str), name='camera')
    model_op = make_op(model_predict, output_types=str, name='model')
    model2_op = make_op(model2_predict, output_types=str,
                        sync=True, freq=2.0, name='model_sync')
    control_op = make_op(execute_control, name='control')

    # Connect Graph
    image1, image2 = camera_op()
    action1, action2 = model_op(image1, image2)
    action3 = model2_op(image1, image2)
    control_op(action3)

    # Initiate Env & Run Graph
    with VehicleEnv() as env:
        env.run()
