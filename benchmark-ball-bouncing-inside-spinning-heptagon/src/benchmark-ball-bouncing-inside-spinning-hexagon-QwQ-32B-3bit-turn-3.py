import tkinter as tk
import math
from dataclasses import dataclass
import sys

# Configuration parameters
HEPTAGON_RADIUS = 300
BALL_RADIUS = 8
WIDTH, HEIGHT = 600, 600  # Canvas dimensions
GRAVITY = 0.3  # Acceleration downward (adjustable)
FRICTION = 0.05  # Damping factor (adjustable)
DT = 0.1  # Time step for physics calculations

# Ball colors provided in the problem statement
BALL_COLORS = [
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

# Heptagon rotation parameters
ROTATION_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    num: int
    color: str
    spin_angle: float = 0
    spin_speed: float = 0


class HeptagonSimulation:
    def __init__(self, canvas):
        self.canvas = canvas
        self.balls = []
        self.time = 0
        # Initialize balls with unique numbers and colors
        for i in range(20):
            ball = Ball(
                x=WIDTH / 2,
                y=HEIGHT / 2,
                vx=0,
                vy=GRAVITY * 2,
                num=i + 1,
                color=BALL_COLORS[i],
            )
            ball.spin_speed = 1 + i  # Vary spin speed based on number
            self.balls.append(ball)

    def closest_point_on_segment(self, a, b, p):
        ax, ay = a
        bx, by = b
        px, py = p

        ab_x = bx - ax
        ab_y = by - ay
        ap_x = px - ax
        ap_y = py - ay

        dot_ap_ab = ap_x * ab_x + ap_y * ab_y
        dot_ab_ab = ab_x**2 + ab_y**2

        if dot_ab_ab == 0:
            return a, math.hypot(ap_x, ap_y)

        t = dot_ap_ab / dot_ab_ab
        t_clamped = max(0, min(1, t))
        closest_x = ax + t_clamped * ab_x
        closest_y = ay + t_clamped * ab_y

        dx = px - closest_x
        dy = py - closest_y
        distance = math.hypot(dx, dy)
        return (closest_x, closest_y), distance

    def update_heptagon_vertices(self):
        angle = 2 * math.pi / 7  # 7 vertices for heptagon
        current_angle = self.time * ROTATION_SPEED
        vertices = []
        for k in range(7):
            theta = current_angle + k * angle
            x = WIDTH / 2 + HEPTAGON_RADIUS * math.cos(theta)
            y = HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(theta)
            vertices.append((x, y))
        return vertices

    def step(self):
        # Update each ball's position and velocity
        for ball in self.balls:
            # Update velocity with gravity and friction
            ball.vy += GRAVITY * DT
            ball.vy *= 1 - FRICTION  # Apply friction damping
            ball.vx *= 1 - FRICTION

            # Update position
            ball.x += ball.vx * DT
            ball.y += ball.vy * DT

            # Update spin angle based on speed
            speed = math.hypot(ball.vx, ball.vy)
            ball.spin_angle += speed * ball.spin_speed * DT

        # Handle collisions with heptagon edges
        vertices = self.update_heptagon_vertices()
        edges = [(vertices[i], vertices[(i + 1) % 7]) for i in range(7)]

        for ball in self.balls:
            for a, b in edges:
                closest_p, distance = self.closest_point_on_segment(
                    a, b, (ball.x, ball.y)
                )
                if distance < BALL_RADIUS:
                    ax, ay = a
                    bx, by = b
                    edge_dir_x = bx - ax
                    edge_dir_y = by - ay
                    edge_length_sq = edge_dir_x**2 + edge_dir_y**2
                    unit_norm_x = -(edge_dir_y) / math.hypot(edge_dir_x, edge_dir_y)
                    unit_norm_y = edge_dir_x / math.hypot(edge_dir_x, edge_dir_y)

                    # Compute normal component of velocity
                    v_dot_n = ball.vx * unit_norm_x + ball.vy * unit_norm_y
                    # Reflect the velocity
                    ball.vx -= 2 * v_dot_n * unit_norm_x
                    ball.vy -= 2 * v_dot_n * unit_norm_y

                    # Move ball out of collision
                    overlap = BALL_RADIUS - distance
                    ball.x += unit_norm_x * overlap
                    ball.y += unit_norm_y * overlap

        # Handle collisions between balls
        balls = self.balls
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                dx = balls[i].x - balls[j].x
                dy = balls[i].y - balls[j].y
                distance_sq = dx**2 + dy**2
                if distance_sq < (2 * BALL_RADIUS) ** 2:
                    distance = math.hypot(dx, dy)
                    if distance == 0:
                        continue
                    normal_x = dx / distance
                    normal_y = dy / distance

                    # Compute normal components of velocities
                    va_n = balls[i].vx * normal_x + balls[i].vy * normal_y
                    vb_n = balls[j].vx * normal_x + balls[j].vy * normal_y

                    # Update velocities after collision
                    va_new_n = vb_n
                    vb_new_n = va_n

                    # Calculate new velocities
                    balls[i].vx -= 2 * va_n * normal_x
                    balls[i].vy -= 2 * va_n * normal_y
                    balls[j].vx -= 2 * vb_n * normal_x
                    balls[j].vy -= 2 * vb_n * normal_y

                    # Separate the balls to prevent overlap
                    overlap = (
                        BALL_RADIUS - dx * normal_x - dy * normal_y
                    )  # Not perfectly accurate but a quick fix
                    balls[i].x -= normal_x * overlap / 2
                    balls[j].x += normal_x * overlap / 2
                    balls[i].y -= normal_y * overlap / 2
                    balls[j].y += normal_y * overlap / 2

        # Draw everything
        self.canvas.delete("all")
        # Draw rotating heptagon
        vertices = self.update_heptagon_vertices()
        for i in range(7):
            self.canvas.create_line(
                vertices[i][0],
                vertices[i][1],
                vertices[(i + 1) % 7][0],
                vertices[(i + 1) % 7][1],
                fill="white",
                width=2,
            )

        # Draw balls and their numbers rotated by their spin angle
        for ball in self.balls:
            x, y = int(ball.x), int(ball.y)
            self.canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
                width=1,
            )
            # Rotate text using a small image or rotation trick (limited by Tkinter)
            # Here we just rotate the text symbolically using angle
            angle_rad = ball.spin_angle % (2 * math.pi)
            if angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            # Draw number rotated by approximate angle
            self.canvas.create_text(
                x,
                y,
                text=str(ball.num),
                angle=int(angle_rad * 180 / math.pi),
                font=("Arial", 8),
            )

        self.time += DT
        self.canvas.after(16, self.step)


def main():
    root = tk.Tk()
    root.title("Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
    canvas.pack()
    sim = HeptagonSimulation(canvas)
    sim.step()
    root.mainloop()


if __name__ == "__main__":
    main()
