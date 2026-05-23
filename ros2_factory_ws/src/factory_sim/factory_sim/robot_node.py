# robot_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

BELT_Y      = 5.0
STATION_XS  = [4.0, 8.0, 12.0]
QC_X        = 16.0
ITEM_RADIUS = 0.3

ARM_REST_Y    = BELT_Y + 2.5
ARM_REACH_Y   = BELT_Y
WORK_DURATION = 1.8
ARM_SPEED     = 0.08


class Robot:
    def __init__(self, robot_id, x, required_stage, result_stage, label, is_qc=False):
        self.id             = robot_id
        self.x              = x
        self.arm_y          = ARM_REST_Y
        self.required_stage = required_stage
        self.result_stage   = result_stage
        self.label          = label
        self.is_qc          = is_qc
        self.state          = 'idle'
        self.target_item_id = None
        self.work_timer     = 0.0

    def to_dict(self):
        return {
            "id":             self.id,
            "x":              self.x,
            "arm_y":          round(self.arm_y, 3),
            "state":          self.state,
            "label":          self.label,
            "target_item_id": self.target_item_id,
        }


class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.config = None
        self.items  = []
        self.dt     = 0.05

        self.robots = [
            Robot(0, STATION_XS[0], 0, 1, "W1"),
            Robot(1, STATION_XS[1], 1, 2, "W2"),
            Robot(2, STATION_XS[2], 2, 3, "W3"),
            Robot(3, QC_X,          3, 4, "QC", is_qc=True),
        ]

        self.create_subscription(String, '/world/config', self.config_cb,  1)
        self.create_subscription(String, '/world/items',  self.items_cb,  10)

        self.robots_pub  = self.create_publisher(String, '/robots/state',    10)
        self.lock_pub    = self.create_publisher(String, '/station/lock',    10)
        self.release_pub = self.create_publisher(String, '/station/release', 10)
        self.qc_pub      = self.create_publisher(String, '/qc/result',       10)

        self.create_timer(self.dt, self.update)
        self.get_logger().info("RobotManagerNode ready.")

    def config_cb(self, msg):
        self.config = json.loads(msg.data)

    def items_cb(self, msg):
        self.items = json.loads(msg.data)

    def find_item_at(self, station_x, required_stage):
        for item in self.items:
            if item['locked']:
                continue
            if item['stage'] != required_stage:
                continue
            if abs(item['x'] - station_x) < ITEM_RADIUS + 0.3:
                return item
        return None

    def update(self):
        if self.config is None:
            return
        for robot in self.robots:
            self._update_robot(robot)
        msg      = String()
        msg.data = json.dumps([r.to_dict() for r in self.robots])
        self.robots_pub.publish(msg)

    def _update_robot(self, robot):
        if robot.state == 'idle':
            robot.arm_y = ARM_REST_Y
            item = self.find_item_at(robot.x, robot.required_stage)
            if item:
                robot.target_item_id = item['id']
                robot.state          = 'extending'
                self._lock(item['id'])
                self.get_logger().info(
                    f"{robot.label}: item {item['id']} detected.")

        elif robot.state == 'extending':
            robot.arm_y -= ARM_SPEED
            if robot.arm_y <= ARM_REACH_Y:
                robot.arm_y      = ARM_REACH_Y
                robot.state      = 'working'
                robot.work_timer = WORK_DURATION
                self.get_logger().info(
                    f"{robot.label}: working on item {robot.target_item_id}.")

        elif robot.state == 'working':
            robot.work_timer -= self.dt
            if robot.work_timer <= 0.0:
                if robot.is_qc:
                    self._qc_result(robot.target_item_id)
                else:
                    self._release(robot.target_item_id, robot.result_stage)
                robot.state = 'retracting'

        elif robot.state == 'retracting':
            robot.arm_y += ARM_SPEED
            if robot.arm_y >= ARM_REST_Y:
                robot.arm_y          = ARM_REST_Y
                robot.target_item_id = None
                robot.state          = 'idle'
                self.get_logger().info(f"{robot.label}: idle.")

    def _lock(self, item_id):
        msg      = String()
        msg.data = json.dumps({"item_id": item_id})
        self.lock_pub.publish(msg)

    def _release(self, item_id, new_stage):
        msg      = String()
        msg.data = json.dumps({"item_id": item_id, "new_stage": new_stage})
        self.release_pub.publish(msg)

    def _qc_result(self, item_id):
        msg      = String()
        msg.data = json.dumps({"item_id": item_id})
        self.qc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()