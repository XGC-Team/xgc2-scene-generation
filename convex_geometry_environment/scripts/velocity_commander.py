#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Velocity Commander for Dynamic Obstacles
Publishes constant velocity commands for dynamic obstacles

Reads initial velocities directly from scenario configuration on parameter server:
  - /convex_geometry_environment_node/obstacles (array)
"""

import rospy
from geometry_msgs.msg import Twist


class ObstacleVelocityCommander:
    """Publishes constant velocity commands for dynamic obstacles"""

    def __init__(self):
        rospy.init_node('velocity_commander', anonymous=False)

        # Read obstacle array from parameter server (loaded by scenario.launch)
        obstacles = rospy.get_param('/convex_geometry_environment_node/obstacles', [])

        # Find dynamic obstacles with IDs 20 and 21
        self.obstacle_configs = {}
        for obs in obstacles:
            obs_id = obs.get('id')
            motion_type = obs.get('motion_type', 'static')

            # Only process dynamic obstacles with velocity_obstacle enabled
            if motion_type == 'dynamic' and obs_id in [20, 21]:
                vel_obstacle = obs.get('velocity_obstacle', {})
                if vel_obstacle.get('enabled', False):
                    initial_vel = vel_obstacle.get('initial_velocity', [0.0, 0.0, 0.0])
                    topic = vel_obstacle.get('topic', '')

                    self.obstacle_configs[obs_id] = {
                        'velocity': initial_vel,
                        'topic': topic
                    }

                    rospy.loginfo("Found dynamic obstacle %d: velocity=[%.2f, %.2f, %.2f], topic=%s",
                                  obs_id, initial_vel[0], initial_vel[1], initial_vel[2], topic)

        if not self.obstacle_configs:
            rospy.logwarn("No dynamic obstacles found in parameter server. Node will exit gracefully.")
            rospy.signal_shutdown("No dynamic obstacles configured")
            return

        # Create publishers for each dynamic obstacle
        self.publishers = {}
        for obs_id, config in self.obstacle_configs.items():
            self.publishers[obs_id] = rospy.Publisher(config['topic'], Twist, queue_size=10)

        # Publish rate: 10 Hz
        self.rate = rospy.Rate(10)

        rospy.loginfo("Velocity Commander initialized with %d dynamic obstacles",
                      len(self.obstacle_configs))

    def publish_commands(self):
        """Publish constant velocity commands"""
        for obs_id, config in self.obstacle_configs.items():
            cmd = Twist()
            cmd.linear.x = config['velocity'][0]
            cmd.linear.y = config['velocity'][1]
            cmd.linear.z = config['velocity'][2]
            self.publishers[obs_id].publish(cmd)

    def run(self):
        """Main loop"""
        rospy.loginfo("Velocity Commander running at 10 Hz...")

        # Wait a bit for subscribers to connect
        rospy.sleep(1.0)

        while not rospy.is_shutdown():
            # Publish constant velocity commands
            self.publish_commands()

            # Sleep to maintain rate
            self.rate.sleep()


if __name__ == '__main__':
    try:
        commander = ObstacleVelocityCommander()
        commander.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Velocity Commander shutdown")
