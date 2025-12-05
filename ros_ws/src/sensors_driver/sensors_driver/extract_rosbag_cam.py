import os
import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

def extract_images_from_bag(bag_path, output_dir, topic="/ids/image"):
    os.makedirs(output_dir, exist_ok=True)
    bridge = CvBridge()

    # Storage setup for MCAP bags
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'      # <--- IMPORTANT
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    print("Bag topics:")
    for t in topics_and_types:
        print(f"  {t.name} ({t.type})")

    print(f"\nExtracting images from topic: {topic}\n")

    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()

        if topic_name != topic:
            continue

        # Convert serialized message → ROS Image msg
        msg = deserialize_message(data, Image)

        # Convert ROS Image → OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Save image
        ts = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        filename = os.path.join(output_dir, f"image_{ts}.jpg")

        cv2.imwrite(filename, cv_image)
        print(f"Saved: {filename}")


if __name__ == "__main__":
    # Path must be the BAG FOLDER, NOT the .mcap file
    bag_path =''
    output_dir = ''

    extract_images_from_bag(bag_path, output_dir)
