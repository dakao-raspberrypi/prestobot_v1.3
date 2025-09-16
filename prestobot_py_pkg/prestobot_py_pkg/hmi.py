#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import pygame


class HmiNode(Node):
    def __init__(self):
        super().__init__("hmi_node")

        pygame.init()
        self.screen_ = pygame.display.set_mode((1000, 500))
        pygame.display.set_caption('Prestobot HMI')
        self.WHITE_ = (255, 255, 255)
        self.BLACK_ = (0, 0, 0)
        self.RED_ = (255, 0, 0)
        self.GREEN_ = (0, 255, 0)
        self.BLUE_ = (0, 0, 255)
        self.font1_ = pygame.font.SysFont('sans', 30)
        self.text0_ = self.font1_.render('Home', True, self.BLACK_)
        self.text_box0_ = self.text0_.get_rect()
        self.text_pos0_ = (250, 250)
        self.text1_ = self.font1_.render('1', True, self.BLACK_)
        self.text_box1_ = self.text1_.get_rect()
        self.text_pos1_ = (500, 250)
        self.text2_ = self.font1_.render('2', True, self.BLACK_)
        self.text_box2_ = self.text2_.get_rect()
        self.text_pos2_ = (750, 250)

        self.navigator_ = BasicNavigator()
        self.navigator_.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Ready to send goals.")
        self.home0_pose_ = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.room1_pose_ = self.create_pose_stamped(5.4, 0.0, -1.57)
        self.room2_pose_ = self.create_pose_stamped(8.5, 0.0, -1.57)

        self.timer_ = self.create_timer(0.02, self.update)
        self.get_logger().info("HMI has been started.")
    
    def update(self):
        self.screen_.fill(self.WHITE_)
        mouse_x, mouse_y = pygame.mouse.get_pos()
        pygame.draw.rect(self.screen_, self.BLUE_, (self.text_pos0_[0], self.text_pos0_[1], self.text_box0_[2], self.text_box0_[3]))
        pygame.draw.rect(self.screen_, self.BLUE_, (self.text_pos1_[0], self.text_pos1_[1], self.text_box1_[2], self.text_box1_[3]))
        pygame.draw.rect(self.screen_, self.BLUE_, (self.text_pos2_[0], self.text_pos2_[1], self.text_box2_[2], self.text_box2_[3]))
        self.screen_.blit(self.text0_, self.text_pos0_)
        self.screen_.blit(self.text1_, self.text_pos1_)
        self.screen_.blit(self.text2_, self.text_pos2_)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if (self.text_pos0_[0] <= mouse_x <= self.text_pos0_[0] + self.text_box0_[2] and
                        self.text_pos0_[1] <= mouse_y <= self.text_pos0_[1] + self.text_box0_[3]):
                    self.get_logger().info("Home button clicked")
                    self.navigator_.goToPose(self.home0_pose_)
                elif (self.text_pos1_[0] <= mouse_x <= self.text_pos1_[0] + self.text_box1_[2] and
                      self.text_pos1_[1] <= mouse_y <= self.text_pos1_[1] + self.text_box1_[3]):
                    self.get_logger().info("Room 1 button clicked")
                    self.navigator_.goToPose(self.room1_pose_)
                elif (self.text_pos2_[0] <= mouse_x <= self.text_pos2_[0] + self.text_box2_[2] and
                      self.text_pos2_[1] <= mouse_y <= self.text_pos2_[1] + self.text_box2_[3]):
                    self.get_logger().info("Room 2 button clicked")
                    self.navigator_.goToPose(self.room2_pose_)
        pygame.display.flip()

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    node = HmiNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
