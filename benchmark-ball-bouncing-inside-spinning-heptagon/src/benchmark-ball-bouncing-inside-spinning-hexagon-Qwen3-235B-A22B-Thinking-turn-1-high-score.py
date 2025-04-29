import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
import random


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    omega: float  # angular velocity
    angle: float  # accumulated angle for rotation visual


@dataclass
class Wall:
    x1: float
    y1: float
    x2: float
    y2: float


# Constants
g = 1000  # gravity pixels/s²
friction = 0.01  # linear friction coefficient
rot_friction = 0.01  # rotational friction
e_wall = 0.9  # coefficient of restitution for walls
e_ball = 0.9  # coefficient for ball-ball collisions
dt = 1.0 / 60.0  # simulation time step
heptagon_radius = 300
ball_radius = 15
window_size = 800
heptagon_center_x = window_size // 2
heptagon_center_y = window_size // 2
heptagon_angular_velocity = 2 * math.pi / 5  # radians per second

colors = [
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


def compute_heptagon_vertices(cx, cy, R, angle):
    vertices = []
    for i in range(7):
        theta = angle + i * 2 * math.pi / 7
        x = cx + R * math.cos(theta)
        y = cy + R * math.sin(theta)
        vertices.append((x, y))
    return vertices


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=window_size, height=window_size)
    canvas.pack()
    heptagon_angle = 0.0

    # Initialize balls
    balls = []
    for i in range(20):
        vx = random.uniform(-50, 50)
        vy = random.uniform(-50, 50)
        omega = random.uniform(-5, 5)
        balls.append(
            Ball(
                x=heptagon_center_x,
                y=heptagon_center_y,
                vx=vx,
                vy=vy,
                radius=ball_radius,
                color=colors[i],
                number=i + 1,
                omega=omega,
                angle=0.0,
            )
        )

    def update_frame():
        nonlocal heptagon_angle

        # Update heptagon's rotation
        heptagon_angle += heptagon_angular_velocity * dt

        # Recompute heptagon walls
        vertices = compute_heptagon_vertices(
            heptagon_center_x, heptagon_center_y, heptagon_radius, heptagon_angle
        )
        walls = []
        for i in range(7):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 7]
            walls.append(Wall(x1, y1, x2, y2))

        # Update all balls' positions with physics
        for ball in balls:
            # Apply gravity
            ball.vy += g * dt
            # Apply friction
            ball.vx *= 1 - friction
            ball.vy *= 1 - friction
            ball.omega *= 1 - rot_friction
            # Update position
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            # Update rotation
            ball.angle += ball.omega * dt

        # Ball-wall collisions
        for ball in balls:
            for wall in walls:
                A = np.array([wall.x1, wall.y1])
                B = np.array([wall.x2, wall.y2])
                P = np.array([ball.x, ball.y])

                # Find closest point on wall segment AB to P
                ab = B - A
                ab_len2 = np.dot(ab, ab)
                if ab_len2 == 0:
                    closest = A.copy()
                else:
                    ap = P - A
                    t = np.dot(ap, ab) / ab_len2
                    t = np.clip(t, 0, 1)
                    closest = A + t * ab

                # Distance
                delta = P - closest
                dist = np.linalg.norm(delta)

                if dist < ball.radius * 0.999:  # 0.999 to handle floating points
                    # Compute normal
                    if dist == 0:
                        normal = np.array([1.0, 0.0], dtype=np.float64)
                    else:
                        normal = delta / dist

                    # Penetration depth
                    pen_depth = ball.radius - dist
                    # Resolve penetration
                    ball.x += normal[0] * pen_depth
                    ball.y += normal[1] * pen_depth

                    # Compute wall's velocity at closest point
                    ox, oy = heptagon_center_x, heptagon_center_y
                    rel_pos = closest - np.array([ox, oy])
                    wall_vel = np.array(
                        [
                            -heptagon_angular_velocity * rel_pos[1],
                            heptagon_angular_velocity * rel_pos[0],
                        ]
                    )

                    # Ball's velocity before
                    v_ball_before = np.array([ball.vx, ball.vy], dtype=np.float64)

                    # Relative velocity in normal direction
                    v_rel_dot = np.dot(v_ball_before - wall_vel, normal)

                    # Collision response
                    v_ball_after = v_ball_before - (e_wall + 1) * v_rel_dot * normal

                    ball.vx, ball.vy = v_ball_after[0], v_ball_after[1]

        # Ball-ball collisions
        num_balls = len(balls)
        for i in range(num_balls):
            for j in range(i + 1, num_balls):
                a = balls[i]
                b = balls[j]

                dx = a.x - b.x
                dy = a.y - b.y
                dist = math.hypot(dx, dy)
                min_dist = a.radius + b.radius

                if dist < min_dist * 0.999:
                    # Resolve penetration
                    if dist == 0:
                        sx, sy = 0.0, 0.0
                    else:
                        sx = dx / dist
                        sy = dy / dist
                    correction = (min_dist - dist) / 1.5  # Distribute correction
                    # Move apart
                    a.x += sx * correction
                    a.y += sy * correction
                    b.x -= sx * correction
                    b.y -= sy * correction

                    # Compute collision normal
                    normal = np.array([a.x - b.x, a.y - b.y], dtype=np.float64)
                    norm_len = np.linalg.norm(normal)
                    if norm_len == 0:
                        continue
                    normal /= norm_len

                    # Relative velocity
                    v_rel = np.array([a.vx - b.vx, a.vy - b.vy], dtype=np.float64)
                    v_dot = np.dot(v_rel, normal)

                    # Impulse
                    j = (e_ball + 1) * v_dot

                    # Update velocities
                    a.vx -= j * normal[0] * 0.5
                    a.vy -= j * normal[1] * 0.5
                    b.vx += j * normal[0] * 0.5
                    b.vy += j * normal[1] * 0.5

        # Redraw
        canvas.delete("all")

        # Draw heptagon
        current_vertices = compute_heptagon_vertices(
            heptagon_center_x, heptagon_center_y, heptagon_radius, heptagon_angle
        )
        poly_coords = []
        for x, y in current_vertices:
            poly_coords.extend((x, y))
        canvas.create_polygon(poly_coords, outline="black", fill="", width=2)

        # Draw balls
        for ball in balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")
            # Draw number
            text_x = ball.x + ball.radius * math.cos(ball.angle)
            text_y = ball.y + ball.radius * math.sin(ball.angle)
            canvas.create_text(
                text_x, text_y, text=str(ball.number), font=("Arial", 8), fill="black"
            )

        # Schedule next update
        root.after(int(dt * 1000), update_frame)

    # Start the update loop
    root.after(int(dt * 1000), update_frame)
    root.mainloop()


if __name__ == "__main__":
    main()
