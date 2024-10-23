import os
import rosbag
import rospy # rospy se importa desde ROS cuando lo ejecuto en la terminal

def split_rosbag(input_bag, split_size, output_dir, output_bag_prefix):
    """
    Split a ROS bag into multiples parts, ensuring continuity of data and adding 
    the last record of the first part as the first record of the next part.

    Args:
    - input_bag (str): Path to the input bag file.
    - split_size (int): Number of messages per split.
    - output_dir (str): Directory where the output bags saved.
    - output_bag_prefix (str): Prefix for the output bag files.
    """

    # Create output directory if it does not exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory {output_dir} for output bag files")

    bag = rosbag.Bag(input_bag, 'r')
    total_messages = bag.get_message_count()
    split_count = 0

    messages = []
    for topic, msg, t in bag.read_messages():
        messages.append((topic, msg, t))

    bag.close()

    # Split messages into chunks of split_size, ensuring overlap of last and first 
    for i in range(0, total_messages, split_size):
        split_count += 1
        output_bag = os.path.join(output_dir, f"{output_bag_prefix}_part_{split_count}.bag")

        with rosbag.Bag(output_bag, 'w') as outbag:
            # Add messages from current split
            for j in range(i, min(i + split_size, total_messages)):
                outbag.write(messages[j][0], messages[j][1], messages[j][2])

            # If not the last split, add the last message from this split
            # as the first message in the next split
            if i + split_size < total_messages:
                last_message = messages[min(i + split_size - 1, total_messages - 1)]
                next_message = messages[min(i + split_size, total_messages - 1)]


                # Adjust the timestamp of the first message in the next bag
                time_diff = next_message[2] - last_message[2]

                adjusted_messages = [
                    (next_message[0], next_message[1], last_message[2] + time_diff)
                ]
                for topic, msg, t in adjusted_messages:
                    outbag.write(topic, msg, t)

    print(f"Created {output_bag} with {min(i + split_size, total_messages) - i} messages")

if __name__ == "__main__":
    input_bag = '../config/lidar_hesai/comedorCompleto.bag' # Replace with actual input bag file
    split_size = 1000 # Number of messages per split, adjust as needed
    output_dir = 'output_bags'
    output_bag_prefix = 'output_splt' # Prefix for the split bag files

    split_rosbag(input_bag, split_size, output_dir, output_bag_prefix)
