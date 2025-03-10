import math
import tkinter as tk
from random import uniform
from dataclasses import dataclass

# Configuration constants
NUM_BALLS = 20
HEPTAGON_RADIUS = 300
BALL_RADIUS = 15
GRAVITY = 0.8
FRICTION = 0.99
BOUNCE_FACTOR = 0.85
SPIN_SPEED = 360 / 5  # degrees per second
COLORS = [
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ec6800",
    "#ee7800",
    "#eb6238",
    "#ea5506",
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
    "#db8449",
    "#d66a35",
]


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    color: str
    number: int
    angular_velocity: float = 0


class HeptagonBounce:
    def __init__(self, master):
        # ... existing initialization code ...
        self.balls = [
            Ball(0, 0, uniform(-2, 2), uniform(-2, 2), COLORS[i], i + 1, uniform(-5, 5))
            for i in range(NUM_BALLS)
        ]
        self.rotation_angle = 0
        self.last_time = 0

        # Canvas setup
        self.canvas = tk.Canvas(master, width=800, height=800)
        self.canvas.pack()

        # Start animation
        self.animate()

    def get_heptagon_points(self):
        # Calculate rotated heptagon vertices
        points = []
        angle_step = math.radians(360 / 7)
        current_angle = math.radians(self.rotation_angle)
        for i in range(7):
            x = HEIGHT / 2 + HEPTAGON_RADIUS * math.cos(current_angle + i * angle_step)
            y = WIDTH / 2 + HEPTAGON_RADIUS * math.sin(current_angle + i * angle_step)
            points.extend([x, y])
        return points

    def check_wall_collisions(self, ball):
        # Transform ball position to heptagon's rotated coordinate system
        angle = -math.radians(self.rotation_angle)
        dx = ball.x - WIDTH / 2
        dy = ball.y - HEIGHT / 2
        rotated_x = dx * math.cos(angle) - dy * math.sin(angle)
        rotated_y = dx * math.sin(angle) + dy * math.cos(angle)

        # Check distance to heptagon center with buffer
        distance = math.hypot(rotated_x, rotated_y)
        max_distance = HEPTAGON_RADIUS - BALL_RADIUS
        if distance > max_distance:
            # ... collision response calculations ...
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.angular_velocity = (ball.vx - ball.vy) * 0.1

    def check_ball_collisions(self):
        for i in range(NUM_BALLS):
            for j in range(i + 1, NUM_BALLS):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                distance = math.hypot(dx, dy)

                if distance < 2 * BALL_RADIUS:
                    # ... elastic collision response ...
                    angle = math.atan2(dy, dx)
                    sin = math.sin(angle)
                    cos = math.cos(angle)

                    # Rotate velocities
                    v1 = cos * b1.vx + sin * b1.vy
                    v2 = cos * b2.vx + sin * b2.vy

                    # Swap velocities
                    b1.vx, b2.vx = v2 * cos, v1 * cos
                    b1.vy, b2.vy = v2 * sin, v1 * sin

                    # Add spin effect
                    b1.angular_velocity += (b1.vx - b1.vy) * 0.05
                    b2.angular_velocity += (b2.vx - b2.vy) * 0.05

    def animate(self):
        # Update physics
        self.rotation_angle += SPIN_SPEED * 0.016
        for ball in self.balls:
            # Apply gravity and friction
            ball.vy += GRAVITY
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Apply spin friction
            ball.angular_velocity *= 0.98

            # Check collisions
            self.check_wall_collisions(ball)
        self.check_ball_collisions()

        # Redraw
        self.canvas.delete("all")
        points = self.get_heptagon_points()
        self.canvas.create_polygon(points, fill="", outline="black", width=2)

        for ball in self.balls:
            # Draw ball with number
            self.canvas.create_oval(
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS,
                ball.y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )
            # Rotate number based on angular velocity
            angle = math.degrees(ball.angular_velocity)
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                angle=angle,
                font=("Arial", int(BALL_RADIUS)),
            )

        self.canvas.after(16, self.animate)


if __name__ == "__main__":
    root = tk.Tk()
    WIDTH = 800
    HEIGHT = 800
    app = HeptagonBounce(root)
    root.mainloop()
