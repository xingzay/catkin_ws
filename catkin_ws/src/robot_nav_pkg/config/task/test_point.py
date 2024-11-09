#!/usr/bin/env python3

from collections import OrderedDict
import rospy
import yaml
import os
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

def load_yaml_file(filename):
    try:
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
            if data and 'waypoints' in data:
                waypoints = OrderedDict(sorted(data['waypoints'].items()))
                return waypoints
    except Exception as e:
        rospy.logerr(f"Error loading YAML file {filename}: {e}")
        return None

def create_pose_array(pose_data):
    pose_array = PoseArray()
    pose_array.header = Header()
    pose_array.header.frame_id = "map"

    if not isinstance(pose_data, dict):
        rospy.logerr(f"Expected a list of poses, got {type(pose_data)}")
        return pose_array

    poses = pose_data.values()

    for pose in poses:
        if not isinstance(pose, dict):
            rospy.logwarn(f"Skipping invalid pose data: {pose}\n{type(pose)}")
            continue

        new_pose = Pose()
        try:
            new_pose.position.x = float(pose.get('position', {}).get('x', 0))
            new_pose.position.y = float(pose.get('position', {}).get('y', 0))
            new_pose.position.z = float(pose.get('position', {}).get('z', 0))
            new_pose.orientation.x = float(pose.get('orientation', {}).get('x', 0))
            new_pose.orientation.y = float(pose.get('orientation', {}).get('y', 0))
            new_pose.orientation.z = float(pose.get('orientation', {}).get('z', 0))
            new_pose.orientation.w = float(pose.get('orientation', {}).get('w', 1))
        except ValueError as e:
            rospy.logwarn(f"Error parsing pose data: {e}")
            continue

        pose_array.poses.append(new_pose)

    return pose_array

def main():
    rospy.init_node('pose_array_publisher')

    config_path = rospy.get_param('~', '')

    filenames = {
        'patrolling': os.path.join(config_path, 'patrolling.yaml'),
        'emergency': os.path.join(config_path, 'emergency.yaml'),
        'charging': os.path.join(config_path, 'charging.yaml'),
        'initial': os.path.join(config_path, 'initial.yaml')
    }

    publishers = {
        'patrolling': rospy.Publisher('point_patrolling', PoseArray, queue_size=10),
        'emergency': rospy.Publisher('point_emergency', PoseArray, queue_size=10),
        'charging': rospy.Publisher('point_charging', PoseArray, queue_size=10),
        'initial': rospy.Publisher('point_initial', PoseArray, queue_size=10)
    }

    pose_arrays = {}

    for key, filename in filenames.items():
        yaml_data = load_yaml_file(filename)
        if yaml_data is not None:
            pose_arrays[key] = create_pose_array(yaml_data)
        else:
            rospy.logerr(f"Failed to load data for {key}")

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        for key, pose_array in pose_arrays.items():
            pose_array.header.stamp = rospy.Time.now()
            publishers[key].publish(pose_array)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
