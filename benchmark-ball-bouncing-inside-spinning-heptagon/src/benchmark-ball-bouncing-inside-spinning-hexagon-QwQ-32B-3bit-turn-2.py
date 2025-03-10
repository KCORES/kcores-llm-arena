import tkinter as tk
import math
import random
from typing import List, Tuple

WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
BALL_RADIUS = 10
HEPTAGON_RADIUS = 350  # Radius larger than balls' radius

colors = ['#f8b862', '#f6ad49', '#f39800', '#f08300', '#ec6d51', '#ee7948', '#ed6d3d', '#ec6800', '#ee7800', '#eb6238', '#ea5506', '#ea5506', '#eb6101', '#e49e61', '#e45e32', '#e17b34', '#dd7a56', '#db8449', '#d66a35']

class Ball:
    def __init__(self, x, y, vx, vy, color, number):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.color = color
        self.number = number

def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def collision_between_balls(ball1: Ball, ball2: Ball):
    dx = ball1.x - ball2.x
    dy = ball1.y - ball2.y
    dist_sq = dx**2 + dy**2
    if dist_sq < 4 * BALL_RADIUS**2 and dist_sq > 1e-6:
        # Calculate normal direction
        norm = math.hypot(dx, dy)
        nx = dx / norm
        ny = dy / norm

        # Compute normal components of velocities
        v_self_n = ball1.vx * nx + ball1.vy * ny
        v_other_n = ball2.vx * nx + ball2.vy * ny

        # Swap normal components
        new_self_n = v_other_n
        new_other_n = v_self_n

        # Update velocities
        ball1.vx -= 2 * nx * v_self_n
        ball1.vy -= 2 * ny * v_self_n
        ball2.vx -= 2 * nx * v_other_n
        ball2.vy -= 2 * ny * v_other_n

def distance_to_segment(px, py, ax, ay, bx, by) -> float:
    # Vector calculations for distance to segment AB from point P(px, py)
    # Omitted for brevity, implement using projection logic

def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg='white')
    canvas.pack()

    balls = []
    # Initialize balls with random positions near center and random velocities

    total_rotation_angle = 0
    angular_speed_rad_per_sec = 2 * math.pi / 5  # 360 degrees per 5 seconds

    def animate():
        nonlocal total_rotation_angle

        dt = 0.016  # 16 ms frame time

        # Update heptagon vertices positions rotated around the center
        vertices = []
        # Calculate each vertex's position

        edges = []
        # Create edges between vertices

        # Move balls and handle collisions

        # Handle collisions with heptagon edges

        # Redraw everything

        root.after(16, animate)

    animate()
    root.mainloop()

if __name__ == "__main__":
    main()
