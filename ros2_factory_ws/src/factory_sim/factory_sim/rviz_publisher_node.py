# rviz_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import json

FRAME_ID = "map"
BELT_Y   = 5.0
BELT_Z   = 0.1

STAGE_COLORS = {
    0: (0.70, 0.55, 0.31, 1.0),
    1: (0.31, 0.63, 0.86, 1.0),
    2: (0.31, 0.86, 0.55, 1.0),
    3: (0.86, 0.47, 0.86, 1.0),
    4: (0.20, 0.86, 0.20, 1.0),
}

ROBOT_COLORS = {
    "W1": (0.31, 0.63, 0.86, 1.0),
    "W2": (0.31, 0.86, 0.55, 1.0),
    "W3": (0.86, 0.47, 0.86, 1.0),
    "QC": (1.00, 0.78, 0.20, 1.0),
}


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c

def point(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p

def vec3(x, y, z):
    v = Vector3()
    v.x, v.y, v.z = float(x), float(y), float(z)
    return v

def base_marker(ns, uid, mtype):
    m = Marker()
    m.header.frame_id    = FRAME_ID
    m.ns                 = ns
    m.id                 = uid
    m.type               = mtype
    m.action             = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


class RvizPublisherNode(Node):
    def __init__(self):
        super().__init__('rviz_publisher_node')

        self.config        = None
        self.items         = []
        self.robots        = []
        self.prev_item_ids = set()

        self.create_subscription(String, '/world/config', self.config_cb,  1)
        self.create_subscription(String, '/world/items',  self.items_cb,  10)
        self.create_subscription(String, '/robots/state', self.robots_cb, 10)

        self.pub_belt   = self.create_publisher(MarkerArray, '/viz/belt',    1)
        self.pub_items  = self.create_publisher(MarkerArray, '/viz/items',  10)
        self.pub_robots = self.create_publisher(MarkerArray, '/viz/robots', 10)

        self.create_timer(0.05, self.publish_all)
        self.get_logger().info("RvizPublisherNode ready.")

    def config_cb(self, msg): self.config = json.loads(msg.data)
    def items_cb(self,  msg): self.items  = json.loads(msg.data)
    def robots_cb(self, msg): self.robots = json.loads(msg.data)

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        if self.config:
            self._publish_belt(now)
            self._publish_items(now)
            self._publish_robots(now)

    def _publish_belt(self, now):
        markers = []
        cfg     = self.config
        uid     = 0

        # Belt surface
        belt_len = cfg['belt_x_end'] - cfg['belt_x_start']
        b = base_marker("belt", uid, Marker.CUBE)
        b.header.stamp  = now
        b.pose.position = point(cfg['belt_x_start'] + belt_len / 2,
                                BELT_Y, BELT_Z / 2)
        b.scale         = vec3(belt_len, 0.8, BELT_Z)
        b.color         = rgba(0.25, 0.25, 0.27)
        markers.append(b); uid += 1



        # Outlet box
        ob = base_marker("outlet", uid, Marker.CUBE)
        ob.header.stamp  = now
        ob.pose.position = point(cfg['outlet_x'] + 0.5, BELT_Y, 0.2)
        ob.scale         = vec3(0.8, 0.8, 0.4)
        ob.color         = rgba(0.2, 0.6, 0.2, 0.8)
        markers.append(ob)

        arr = MarkerArray(); arr.markers = markers
        self.pub_belt.publish(arr)

    def _publish_items(self, now):
        markers    = []
        active_ids = set()

        for item in self.items:
            uid = item['id']
            active_ids.add(uid)
            col_t = STAGE_COLORS.get(item['stage'], (0.8, 0.8, 0.8, 1.0))

            m = base_marker("items", uid, Marker.SPHERE)
            m.header.stamp  = now
            m.pose.position = point(item['x'], item['y'], BELT_Z + 0.3)
            m.scale         = vec3(0.5, 0.5, 0.5)
            m.color         = rgba(*col_t)
            markers.append(m)

            if item['locked']:
                ring = base_marker("item_rings", uid, Marker.CYLINDER)
                ring.header.stamp  = now
                ring.pose.position = point(item['x'], item['y'], BELT_Z + 0.05)
                ring.scale         = vec3(0.7, 0.7, 0.04)
                ring.color         = rgba(1.0, 0.8, 0.2, 0.8)
                markers.append(ring)

        # Delete items that left the belt
        for old_id in self.prev_item_ids - active_ids:
            for ns in ["items", "item_rings"]:
                d = Marker()
                d.header.frame_id = FRAME_ID
                d.header.stamp    = now
                d.ns              = ns
                d.id              = old_id
                d.action          = Marker.DELETE
                markers.append(d)

        self.prev_item_ids = active_ids
        if markers:
            arr = MarkerArray(); arr.markers = markers
            self.pub_items.publish(arr)

    def _publish_robots(self, now):
        # In RViz2, Z is up. Belt sits at Z=0.1.
        # Robots are fixed at belt X position, belt Y position,
        # and extend their arm downward in Z.
        # arm_y from robot_node: resting=7.5, extended=5.0 (belt_y)
        # We map this to Z: resting=2.5, extended=0.3 (just above belt)
        ARM_REST_Z   = 2.5
        ARM_REACH_Z  = 0.3
        BODY_Z       = 3.0
        ARM_REST_Y_V = BELT_Y + 2.5   # resting value from robot_node

        markers = []

        for robot in self.robots:
            uid   = robot['id']
            col_t = ROBOT_COLORS.get(robot['label'], (0.7, 0.7, 0.7, 1.0))
            rx    = robot['x']
            arm_y = robot['arm_y']   # value between BELT_Y and BELT_Y+2.5

            # Map arm_y → arm_z (inverted: higher arm_y = higher z)
            t     = (arm_y - BELT_Y) / 2.5       # 0.0 (extended) to 1.0 (resting)
            arm_z = ARM_REACH_Z + t * (ARM_REST_Z - ARM_REACH_Z)

            # Body — sits above belt at fixed position
            body = base_marker("robot_bodies", uid, Marker.CUBE)
            body.header.stamp  = now
            body.pose.position = point(rx, BELT_Y, BODY_Z)
            body.scale         = vec3(0.5, 0.5, 0.4)
            body.color         = rgba(*col_t)
            markers.append(body)

            # Arm — always published, points collapse to same spot when idle
            arm = base_marker("robot_arms", uid, Marker.LINE_STRIP)
            arm.header.stamp = now
            arm.points = [
                point(rx, BELT_Y, BODY_Z - 0.2),   # bottom of body
                point(rx, BELT_Y, arm_z),           # tip (same as above when idle)
            ]
            arm.scale.x = 0.07
            arm.color   = rgba(0.75, 0.75, 0.8)
            markers.append(arm)

            # Arm tip — always published, moves to rest z when idle
            tip = base_marker("arm_tips", uid, Marker.SPHERE)
            tip.header.stamp  = now
            tip.pose.position = point(rx, BELT_Y, arm_z)
            tip.scale         = vec3(0.18, 0.18, 0.18)
            # Hide tip when fully retracted
            tip.color = rgba(*col_t) if arm_z < ARM_REST_Z - 0.1 else rgba(0,0,0,0)
            markers.append(tip)

            # Label above body
            lbl = base_marker("robot_labels", uid, Marker.TEXT_VIEW_FACING)
            lbl.header.stamp  = now
            lbl.pose.position = point(rx, BELT_Y, BODY_Z + 0.5)
            lbl.scale.z       = 0.4
            lbl.color         = rgba(1.0, 1.0, 1.0)
            lbl.text          = robot['label']
            markers.append(lbl)

        arr = MarkerArray(); arr.markers = markers
        self.pub_robots.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()