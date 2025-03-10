import tkinter as tk
from dataclasses import dataclass
import math
import numpy as np
from typing import List, Tuple
import sys


@dataclass
class Ball:
    x: float  # center x position
    y: float  # center y position
    vx: float  # velocity x
    vy: float  # velocity y
    angle: float  # rotation angle
    angular_vel: float  # angular velocity
    number: int  # ball number
    color: str  # ball color


WINDOW_SIZE = 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
GRAVITY = 981.0  # pixels/s^2
FRICTION = 0.8  # friction coefficient
ANGULAR_FRICTION = 0.95
RESTITUTION = 0.7  # coefficient of restitution
DT = 1 / 60  # time step

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


class BouncingBallsApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in Spinning Heptagon")

        self.canvas = tk.Canvas(
            self.root, width=WINDOW_SIZE, height=WINDOW_SIZE, bg="black"
        )
        self.canvas.pack()

        self.heptagon_angle = 0
        self.balls = self.initialize_balls()

        self.root.after(0, self.update)

    def initialize_balls(self) -> List[Ball]:
        balls = []
        for i in range(20):
            angle = 2 * math.pi * i / 20
            # All balls start from center with slight offset
            balls.append(
                Ball(
                    x=WINDOW_SIZE / 2 + math.cos(angle) * 5,
                    y=WINDOW_SIZE / 2 + math.sin(angle) * 5,
                    vx=0,
                    vy=0,
                    angle=0,
                    angular_vel=0,
                    number=i + 1,
                    color=COLORS[i],
                )
            )
        return balls

    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        center_x = WINDOW_SIZE / 2
        center_y = WINDOW_SIZE / 2

        for i in range(7):
            angle = self.heptagon_angle + 2 * math.pi * i / 7
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def check_wall_collision(self, ball: Ball) -> None:
        vertices = self.get_heptagon_vertices()

        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]

            # Calculate wall normal
            wall_dx = v2[0] - v1[0]
            wall_dy = v2[1] - v1[1]
            wall_len = math.sqrt(wall_dx**2 + wall_dy**2)
            normal_x = -wall_dy / wall_len
            normal_y = wall_dx / wall_len

            # Calculate distance to wall
            dist = (ball.x - v1[0]) * normal_x + (ball.y - v1[1]) * normal_y

            if abs(dist) <= BALL_RADIUS:
                # Collision response
                dot_product = ball.vx * normal_x + ball.vy * normal_y

                ball.vx -= (1 + RESTITUTION) * dot_product * normal_x
                ball.vy -= (1 + RESTITUTION) * dot_product * normal_y

                # Apply friction to tangential velocity
                tangent_x = -normal_y
                tangent_y = normal_x
                tang_vel = ball.vx * tangent_x + ball.vy * tangent_y

                ball.vx -= FRICTION * tang_vel * tangent_x
                ball.vy -= FRICTION * tang_vel * tangent_y

                # Update angular velocity based on friction
                ball.angular_vel += tang_vel * FRICTION / BALL_RADIUS

    def check_ball_collision(self, b1: Ball, b2: Ball) -> None:
        dx = b2.x - b1.x
        dy = b2.y - b1.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 2 * BALL_RADIUS:
            # Normalize collision vector
            nx = dx / dist
            ny = dy / dist

            # Relative velocity
            rel_vx = b2.vx - b1.vx
            rel_vy = b2.vy - b1.vy

            # Impulse
            impulse = (1 + RESTITUTION) * (rel_vx * nx + rel_vy * ny) / 2

            # Update velocities
            b1.vx += impulse * nx
            b1.vy += impulse * ny
            b2.vx -= impulse * nx
            b2.vy -= impulse * ny

            # Update angular velocities
            rel_vel = math.sqrt(rel_vx**2 + rel_vy**2)
            b1.angular_vel += rel_vel * FRICTION / BALL_RADIUS
            b2.angular_vel -= rel_vel * FRICTION / BALL_RADIUS

    def update(self):
        # Update heptagon rotation
        self.heptagon_angle += 2 * math.pi * DT / 5  # 360 degrees per 5 seconds

        # Update ball positions and velocities
        for ball in self.balls:
            ball.vy += GRAVITY * DT
            ball.x += ball.vx * DT
            ball.y += ball.vy * DT
            ball.angle += ball.angular_vel * DT
            ball.angular_vel *= ANGULAR_FRICTION

            self.check_wall_collision(ball)

        # Check ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.check_ball_collision(self.balls[i], self.balls[j])

        self.draw()
        self.root.after(int(DT * 1000), self.update)

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon
        vertices = self.get_heptagon_vertices()
        self.canvas.create_polygon(
            *[coord for vertex in vertices for coord in vertex],
            outline="white",
            fill="",
            width=2
        )

        # Draw balls
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS,
                ball.y + BALL_RADIUS,
                fill=ball.color,
                outline="white",
            )

            # Draw number (rotated to show ball spin)
            text_x = ball.x + BALL_RADIUS * 0.5 * math.cos(ball.angle)
            text_y = ball.y + BALL_RADIUS * 0.5 * math.sin(ball.angle)
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(BALL_RADIUS * 0.8)),
            )


if __name__ == "__main__":
    app = BouncingBallsApp()
    app.root.mainloop()
