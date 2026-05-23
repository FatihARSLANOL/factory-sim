# visualizer_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import sys

try:
    import pygame
except ImportError:
    print("Install pygame: pip install pygame")
    sys.exit(1)

WIN_W  = 900
WIN_H  = 500
PAD    = 40
WORLD_W = 20.0
WORLD_H = 10.0
SCALE_X = (WIN_W - 2 * PAD) / WORLD_W
SCALE_Y = (WIN_H - 120) / WORLD_H

BG         = ( 18,  18,  22)
BELT_COL   = ( 60,  60,  65)
BELT_BDR   = ( 90,  90,  95)
TEXT_C     = (220, 220, 220)
DIM_C      = (120, 120, 120)

STAGE_COLORS = {
    0: (180, 140,  80),
    1: ( 80, 160, 220),
    2: ( 80, 220, 140),
    3: (220, 120, 220),
    4: ( 80, 220,  80),
}

ROBOT_COLORS = {
    "W1": ( 80, 160, 220),
    "W2": ( 80, 220, 140),
    "W3": (220, 120, 220),
    "QC": (255, 200,  50),
}

STATE_COLORS = {
    "idle":       (100, 100, 110),
    "extending":  (255, 200,  50),
    "working":    ( 50, 200, 100),
    "retracting": (200, 150,  50),
}


def wx(x):  return int(PAD + x * SCALE_X)
def wy(y):  return int(WIN_H - 120 - y * SCALE_Y)
def ws(v):  return max(1, int(v * SCALE_X))


class VisualizerNode(Node):
    def __init__(self, screen, font, sfont):
        super().__init__('visualizer_node')
        self.screen = screen
        self.font   = font
        self.sfont  = sfont
        self.config = None
        self.items  = []
        self.robots = []

        self.create_subscription(String, '/world/config', self.config_cb,  1)
        self.create_subscription(String, '/world/items',  self.items_cb,  10)
        self.create_subscription(String, '/robots/state', self.robots_cb, 10)

        self.create_timer(0.033, self.draw)
        self.get_logger().info("VisualizerNode ready.")

    def config_cb(self, msg): self.config = json.loads(msg.data)
    def items_cb(self,  msg): self.items  = json.loads(msg.data)
    def robots_cb(self, msg): self.robots = json.loads(msg.data)

    def draw(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)

        self.screen.fill(BG)

        if self.config is None:
            pygame.display.flip()
            return

        cfg    = self.config
        belt_y = cfg['belt_y']
        belt_h = 0.6

        # ── Belt ──────────────────────────────────────────────────────────────
        bx1 = wx(cfg['belt_x_start'])
        bx2 = wx(cfg['belt_x_end'])
        by1 = wy(belt_y + belt_h / 2)
        by2 = wy(belt_y - belt_h / 2)
        pygame.draw.rect(self.screen, BELT_COL,
                         pygame.Rect(bx1, by1, bx2 - bx1, by2 - by1))
        pygame.draw.rect(self.screen, BELT_BDR,
                         pygame.Rect(bx1, by1, bx2 - bx1, by2 - by1), 2)

        # Belt direction arrows
        for ax in range(1, 20, 2):
            sx = wx(ax); sy = wy(belt_y)
            pygame.draw.line(self.screen, (90, 90, 100),
                             (sx, sy), (sx + 12, sy), 2)
            pygame.draw.polygon(self.screen, (90, 90, 100),
                                [(sx+12,sy),(sx+8,sy-3),(sx+8,sy+3)])

        # ── Station markers ───────────────────────────────────────────────────
        for i, sx_w in enumerate(cfg['station_xs']):
            sx = wx(sx_w)
            pygame.draw.line(self.screen, (60,60,70),
                             (sx, wy(0)), (sx, wy(WORLD_H)), 1)
            s = self.sfont.render(f"STN {i+1}", True, DIM_C)
            self.screen.blit(s, (sx - 18, wy(0) + 4))

        qx = wx(cfg['qc_x'])
        pygame.draw.line(self.screen, (60,60,50),
                         (qx, wy(0)), (qx, wy(WORLD_H)), 1)
        s = self.sfont.render("QC", True, (150,140,60))
        self.screen.blit(s, (qx - 8, wy(0) + 4))

        # Outlet box
        out_x = wx(cfg['outlet_x'])
        pygame.draw.rect(self.screen, (50,150,50),
                         pygame.Rect(out_x, wy(belt_y) - 20, 20, 20), 2)
        s = self.sfont.render("OUT", True, (80,200,80))
        self.screen.blit(s, (out_x + 2, wy(belt_y) - 18))

        # ── Items ─────────────────────────────────────────────────────────────
        for item in self.items:
            ix  = wx(item['x'])
            iy  = wy(item['y'])
            r   = ws(cfg['item_radius'])
            col = STAGE_COLORS.get(item['stage'], (200,200,200))
            pygame.draw.circle(self.screen, col, (ix, iy), r)
            pygame.draw.circle(self.screen, (255,255,255), (ix, iy), r, 1)
            stage_names = ["R","1","2","3","✓"]
            if item['stage'] < len(stage_names):
                t = self.sfont.render(stage_names[item['stage']], True, (0,0,0))
                self.screen.blit(t, (ix - 4, iy - 6))
            if item['locked']:
                pygame.draw.circle(self.screen, (255,200,50),
                                   (ix, iy), r + 3, 2)

        # ── Robots ────────────────────────────────────────────────────────────
        for robot in self.robots:
            rx      = wx(robot['x'])
            rest_sy = wy(belt_y + 2.5)
            arm_tip = wy(robot['arm_y'])
            col     = ROBOT_COLORS.get(robot['label'], (200,200,200))
            sc      = STATE_COLORS.get(robot['state'], (100,100,100))
            bw, bh  = 28, 20
            pygame.draw.rect(self.screen, col,
                             pygame.Rect(rx - bw//2, rest_sy - bh, bw, bh))
            pygame.draw.rect(self.screen, (255,255,255),
                             pygame.Rect(rx - bw//2, rest_sy - bh, bw, bh), 1)
            pygame.draw.line(self.screen, sc, (rx, rest_sy), (rx, arm_tip), 3)
            pygame.draw.circle(self.screen, sc, (rx, arm_tip), 5)
            t = self.sfont.render(robot['label'], True, (0,0,0))
            self.screen.blit(t, (rx - 8, rest_sy - 15))
            pygame.draw.circle(self.screen, sc, (rx + 16, rest_sy - 16), 4)

        # ── Status panel ──────────────────────────────────────────────────────
        panel_y = WIN_H - 110
        pygame.draw.line(self.screen, (50,50,55), (0,panel_y), (WIN_W,panel_y), 1)

        title = self.font.render("Factory Assembly Line", True, (255,255,255))
        self.screen.blit(title, (PAD, panel_y + 8))

        info = f"Items on belt: {len(self.items)}"
        s = self.sfont.render(info, True, DIM_C)
        self.screen.blit(s, (PAD, panel_y + 30))

        rx_off = PAD
        for robot in self.robots:
            col = STATE_COLORS.get(robot['state'], DIM_C)
            s   = self.sfont.render(
                f"{robot['label']}: {robot['state'].upper()}", True, col)
            self.screen.blit(s, (rx_off, panel_y + 50))
            rx_off += 180

        # Legend
        lx, ly = WIN_W - 200, panel_y + 8
        legend = [
            (STAGE_COLORS[0], "Raw"),
            (STAGE_COLORS[1], "Stage 1"),
            (STAGE_COLORS[2], "Stage 2"),
            (STAGE_COLORS[3], "Stage 3"),
            (STAGE_COLORS[4], "Complete"),
        ]
        for col, label in legend:
            pygame.draw.circle(self.screen, col, (lx, ly + 6), 6)
            s = self.sfont.render(label, True, DIM_C)
            self.screen.blit(s, (lx + 12, ly))
            ly += 18

        pygame.display.flip()


def main(args=None):
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Factory Assembly Line")
    font  = pygame.font.SysFont("monospace", 15, bold=True)
    sfont = pygame.font.SysFont("monospace", 12)
    rclpy.init(args=args)
    node = VisualizerNode(screen, font, sfont)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()