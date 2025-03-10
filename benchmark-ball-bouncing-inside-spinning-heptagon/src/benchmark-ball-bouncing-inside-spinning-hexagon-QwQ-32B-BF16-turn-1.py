import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys
import time

WIDTH, HEIGHT = 800, 800
BALL_RADIUS = 10
HEPTAGON_RADIUS = 300
ANGULAR_SPEED = 2 * math.pi / 5  # radians per second
GRAVITY = 0.5  # acceleration due to gravity
FRICTION_COEFF = 0.1  # friction coefficient
RESTITUTION = 0.8  # restitution coefficient
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


@dataclass
class Ball:
    pos: np.ndarray  # [x, y]
    vel: np.ndarray  # velocity vector
    radius: float = BALL_RADIUS
    color: str = ""
    spin: int = 0
    new_pos: np.ndarray = None
    new_vel: np.ndarray = None


def get_outward_normal(A, B):
    A = np.array(A)
    B = np.array(B)
    AB = B - A
    normal_vector = np.array([AB[1], -AB[0]])
    length = np.linalg.norm(normal_vector)
    if length == 0:
        return np.zeros(2)
    return normal_vector / length


def get_heptagon_edges(rotation_angle):
    vertices = []
    for k in range(7):
        angle = rotation_angle + 2 * math.pi * k / 7
        x = HEPTAGON_RADIUS * math.cos(angle)
        y = HEPTAGON_RADIUS * math.sin(angle)
        vertices.append((x, y))
    edges = []
    for i in range(7):
        a = vertices[i]
        b = vertices[(i + 1) % 7]
        edges.append((a, b))
    return edges


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    balls = []
    for i in range(20):
        ball = Ball(
            pos=np.array([0.0, 0.0]),
            vel=np.array([0.0, 0.0]),
            color=colors[i],
            spin=i + 1,
        )
        balls.append(ball)

    current_rotation_angle = 0.0
    last_time = None

    def animate():
        nonlocal current_rotation_angle, last_time
        current_time = time.time()
        if last_time is None:
            last_time = current_time
            return
        delta_t = current_time - last_time
        last_time = current_time

        current_rotation_angle = (current_rotation_angle + ANGULAR_SPEED * delta_t) % (
            2 * math.pi
        )

        # Update each ball's new position and velocity
        for ball in balls:
            acceleration = np.array([0.0, GRAVITY]) - FRICTION_COEFF * ball.vel
            new_vel = ball.vel + acceleration * delta_t
            new_pos = ball.pos + ball.vel * delta_t
            ball.new_vel = new_vel
            ball.new_pos = new_pos

        # Process ball-ball collisions
        for i in range(len(balls)):
            ball1 = balls[i]
            for j in range(i + 1, len(balls)):
                ball2 = balls[j]
                dx = ball1.new_pos[0] - ball2.new_pos[0]
                dy = ball1.new_pos[1] - ball2.new_pos[1]
                dist_sq = dx**2 + dy**2
                if dist_sq < (2 * BALL_RADIUS) ** 2 and dist_sq > 1e-6:
                    norm = np.array([dx, dy], dtype=float)
                    length = np.hypot(dx, dy)
                    unit_n = norm / length
                    v_rel = ball2.new_vel - ball1.new_vel
                    v_rel_n = np.dot(v_rel, unit_n)
                    impulse = ((1 + RESTITUTION) * v_rel_n) / 2
                    ball1.new_vel += impulse * unit_n
                    ball2.new_vel -= impulse * unit_n

        # Process ball-wall collisions
        edges = get_heptagon_edges(current_rotation_angle)
        for ball in balls:
            new_pos = ball.new_pos.copy()
            closest_dist = float("inf")
            closest_signed = 0
            closest_normal = None
            closest_edge = None
            closest_point = None

            for edge in edges:
                A, B = edge
                A = np.array(A)
                B = np.array(B)
                AB = B - A
                AP = new_pos - A
                t = np.dot(AP, AB) / np.dot(AB, AB)
                if t < 0:
                    closest_p = A
                elif t > 1:
                    closest_p = B
                else:
                    closest_p = A + AB * t
                vec_to_p = new_pos - closest_p
                outward_normal = get_outward_normal(A, B)
                signed_distance = np.dot(vec_to_p, outward_normal)
                distance = abs(signed_distance)
                if signed_distance > 0 and distance < closest_dist:
                    closest_dist = distance
                    closest_signed = signed_distance
                    closest_normal = outward_normal
                    closest_point = closest_p
                    closest_edge = (A, B)

            if closest_dist < BALL_RADIUS and closest_signed > 0:
                delta = BALL_RADIUS - closest_signed
                inward_normal = -closest_normal
                correction = inward_normal * delta
                new_pos += correction
                v = ball.new_vel
                v_n = np.dot(v, closest_normal)
                v_n_new = -RESTITUTION * v_n
                v_t = v - v_n * closest_normal
                ball.new_vel = v_t + v_n_new * closest_normal
                ball.new_pos = new_pos

        # Update balls' positions and velocities
        for ball in balls:
            ball.pos = ball.new_pos
            ball.vel = ball.new_vel

        # Draw the scene
        canvas.delete("all")
        edges = get_heptagon_edges(current_rotation_angle)
        vertices = []
        for edge in edges:
            A, B = edge
            vertices.append(A)
        scaled_vertices = [(WIDTH / 2 + v[0], HEIGHT / 2 + v[1]) for v in vertices]
        canvas.create_polygon(scaled_vertices, outline="black", fill="")

        for ball in balls:
            x, y = ball.pos
            cx = WIDTH / 2 + x
            cy = HEIGHT / 2 + y
            canvas.create_oval(
                cx - BALL_RADIUS,
                cy - BALL_RADIUS,
                cx + BALL_RADIUS,
                cy + BALL_RADIUS,
                fill=ball.color,
            )
            canvas.create_text(cx, cy, text=str(ball.spin), fill="black")

        root.after(16, animate)

    root.after(0, animate)
    root.mainloop()


if __name__ == "__main__":
    main()
