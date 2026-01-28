import yaml
from builtin_interfaces.msg import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from typing import Dict


def get_tfs_from_yaml(file_path: str) -> Dict:
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
    return data


def format_tf_msg(ts: Time, tfs: Dict) -> TFMessage:
    tf_message = TFMessage()

    for tf_conf in tfs["transforms"]:
        tf = TransformStamped()
        tf.header.stamp = ts
        tf.header.frame_id = tf_conf["frame_id"]
        tf.child_frame_id = tf_conf["child_frame_id"]

        # convert from euler angles in degrees to quaternion
        r_arr = [tf_conf["rotation"][axis] for axis in ["x", "y", "z"]]
        rotation = Rotation.from_euler("XYZ", r_arr, degrees=True)
        if tf_conf["invert"]:
            rotation = rotation.inv()

        quaternion = rotation.as_quat()

        tf.transform.translation.x = tf_conf["translation"]["x"]
        tf.transform.translation.y = tf_conf["translation"]["y"]
        tf.transform.translation.z = tf_conf["translation"]["z"]
        tf.transform.rotation.x = quaternion[0]
        tf.transform.rotation.y = quaternion[1]
        tf.transform.rotation.z = quaternion[2]
        tf.transform.rotation.w = quaternion[3]

        tf_message.transforms.append(tf)

    return tf_message


if __name__ == "__main__":
    print("This is a library file and cannot be executed directly.")
