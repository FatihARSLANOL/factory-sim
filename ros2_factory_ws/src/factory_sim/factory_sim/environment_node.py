# environment_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# ── World layout ──────────────────────────────────────────────────────────────
BELT_Y        = 5.0
BELT_X_START  = 0.0
BELT_X_END    = 20.0
BELT_SPEED    = 0.04

INLET_X       = 0.0
OUTLET_X      = 19.0

STATION_XS    = [4.0, 8.0, 12.0]
QC_X          = 16.0

SPAWN_INTERVAL = 6.0
MAX_ITEMS      = 6

STAGE_RAW    = 0
STAGE_PASSED = 4
STAGE_DONE   = 5

ITEM_RADIUS  = 0.3


class Item:
    _id_counter = 0

    def __init__(self, x):
        Item._id_counter += 1
        self.id     = Item._id_counter
        self.x      = x
        self.y      = BELT_Y
        self.stage  = STAGE_RAW
        self.locked = False
        self.done   = False

    def to_dict(self):
        return {
            "id":     self.id,
            "x":      round(self.x, 3),
            "y":      round(self.y, 3),
            "stage":  self.stage,
            "locked": self.locked,
            "done":   self.done,
        }


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        self.items       = []
        self.spawn_timer = 0.0
        self.dt          = 0.05

        self.config_pub = self.create_publisher(String, '/world/config',  1)
        self.items_pub  = self.create_publisher(String, '/world/items',  10)

        self.create_subscription(String, '/station/lock',    self.lock_cb,    10)
        self.create_subscription(String, '/station/release', self.release_cb, 10)
        self.create_subscription(String, '/qc/result',       self.qc_cb,      10)

        self.create_timer(self.dt, self.update)
        self.create_timer(0.1,     self.publish_state)
        self.create_timer(1.0,     self.publish_config)

        self._spawn_item()
        self.get_logger().info("EnvironmentNode ready.")

    def _spawn_item(self):
        self.items.append(Item(INLET_X))
        self.get_logger().info(f"Spawned item {self.items[-1].id}.")

    def lock_cb(self, msg):
        item_id = json.loads(msg.data)['item_id']
        for item in self.items:
            if item.id == item_id:
                item.locked = True
                break

    def release_cb(self, msg):
        data = json.loads(msg.data)
        for item in self.items:
            if item.id == data['item_id']:
                item.stage  = data['new_stage']
                item.locked = False
                self.get_logger().info(
                    f"Item {item.id} → stage {item.stage}.")
                break

    def qc_cb(self, msg):
        item_id = json.loads(msg.data)['item_id']
        for item in self.items:
            if item.id == item_id:
                item.stage  = STAGE_PASSED
                item.locked = False
                self.get_logger().info(f"Item {item.id} passed QC.")
                break

    def update(self):
        self.spawn_timer += self.dt
        if self.spawn_timer >= SPAWN_INTERVAL and len(self.items) < MAX_ITEMS:
            self._spawn_item()
            self.spawn_timer = 0.0

        for item in self.items:
            if item.done or item.locked:
                continue
            item.x += BELT_SPEED
            if item.stage == STAGE_PASSED and item.x >= OUTLET_X:
                item.done = True
                self.get_logger().info(f"Item {item.id} delivered.")
            elif item.x >= BELT_X_END:
                item.done = True

        self.items = [i for i in self.items if not i.done]

    def publish_state(self):
        msg      = String()
        msg.data = json.dumps([i.to_dict() for i in self.items])
        self.items_pub.publish(msg)

    def publish_config(self):
        msg      = String()
        msg.data = json.dumps({
            "belt_y":       BELT_Y,
            "belt_x_start": BELT_X_START,
            "belt_x_end":   BELT_X_END,
            "station_xs":   STATION_XS,
            "qc_x":         QC_X,
            "outlet_x":     OUTLET_X,
            "item_radius":  ITEM_RADIUS,
        })
        self.config_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()