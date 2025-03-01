import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List
import sys


# Constants
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 600
HEPTAGON_RADIUS = 250
BALL_RADIUS = 10
NUM_BALLS = 20
GRAVITY = 0.05
FRICTION = 0.98
BOUNCE_REDUCTION = 0.6  # Bounce reduction factor
SPIN_FRICTION = 0.95
HEPTAGON_SPIN_SPEED = 360 / 5  # Degrees per second
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    spin: float  # Angular velocity of the ball
    number: int
    color: str
    radius: float

    def draw(self, canvas: tk.Canvas, angle_offset: float):
        """Draws the ball on the canvas, including its number."""
        x = self.x
        y = self.y
        # Rotate the number based on spin
        text_angle = self.spin * 10  # Adjust the scaling factor as needed
        canvas.create_oval(
            x - self.radius,
            y - self.radius,
            x + self.radius,
            y + self.radius,
            fill=self.color,
            outline="black",
        )
        canvas.create_text(
            x,
            y,
            text=str(self.number),
            font=("Arial", 8),
            fill="black",
            angle=text_angle % 360,
        )

    def update(self, balls: List["Ball"], angle_offset: float):
        """Updates the ball's position, velocity, and spin."""
        # Apply gravity
        self.vy += GRAVITY

        # Update position
        self.x += self.vx
        self.y += self.vy

        # Apply spin friction
        self.spin *= SPIN_FRICTION

        # Apply overall friction to the ball
        self.vx *= FRICTION
        self.vy *= FRICTION

        # Heptagon wall collision and ball-ball collision check happens inside here.
        self.handle_collisions(balls, angle_offset)

    def handle_collisions(self, balls: List["Ball"], angle_offset: float):
        """Handles collisions with the heptagon walls and other balls."""
        # Heptagon Wall Collision
        for i in range(7):
            angle1 = math.radians(angle_offset + (i * 360 / 7))
            angle2 = math.radians(angle_offset + ((i + 1) % 7 * 360 / 7))
            x1 = CANVAS_WIDTH / 2 + HEPTAGON_RADIUS * math.cos(angle1)
            y1 = CANVAS_HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(angle1)
            x2 = CANVAS_WIDTH / 2 + HEPTAGON_RADIUS * math.cos(angle2)
            y2 = CANVAS_HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(angle2)

            # Calculate distance from ball center to the line segment (wall)
            dx = self.x - x1
            dy = self.y - y1
            wall_dx = x2 - x1
            wall_dy = y2 - y1

            if wall_dx == 0 and wall_dy == 0:  # Edge case where wall segment is a point
                continue

            t = (dx * wall_dx + dy * wall_dy) / (wall_dx * wall_dx + wall_dy * wall_dy)
            if t < 0:
                closest_x = x1
                closest_y = y1
            elif t > 1:
                closest_x = x2
                closest_y = y2
            else:
                closest_x = x1 + t * wall_dx
                closest_y = y1 + t * wall_dy

            dist_to_wall = math.sqrt(
                (self.x - closest_x) ** 2 + (self.y - closest_y) ** 2
            )

            if dist_to_wall < self.radius:
                # Collision detected with heptagon wall
                normal_x = self.x - closest_x
                normal_y = self.y - closest_y
                norm_magnitude = math.sqrt(normal_x**2 + normal_y**2)
                if norm_magnitude > 0:  # Avoid division by zero
                    normal_x /= norm_magnitude
                    normal_y /= norm_magnitude

                    # Calculate relative velocity
                    rel_vx = self.vx
                    rel_vy = self.vy
                    # Calculate the dot product of relative velocity and normal
                    dot_product = rel_vx * normal_x + rel_vy * normal_y

                    # Apply reflection. The bounce height should not exceed the radius of the heptagon.
                    if dot_product > 0:  # Ensure the ball is moving towards the wall
                        return  # or continue to next wall check

                    self.vx -= 2 * dot_product * normal_x * BOUNCE_REDUCTION
                    self.vy -= 2 * dot_product * normal_y * BOUNCE_REDUCTION

                    # Adjust ball position to prevent being stuck inside the wall
                    overlap = self.radius - dist_to_wall
                    self.x += normal_x * overlap
                    self.y += normal_y * overlap

                    # Add spin to the ball upon collision
                    self.spin += (
                        self.vx * normal_y - self.vy * normal_x
                    ) * 0.1  # Adjust spin factor as desired

        # Ball-Ball Collision
        for other_ball in balls:
            if self is other_ball:
                continue

            dx = other_ball.x - self.x
            dy = other_ball.y - self.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.radius + other_ball.radius:
                # Collision detected
                # Calculate normal and penetration depth
                normal_x = dx / distance
                normal_y = dy / distance
                penetration_depth = (self.radius + other_ball.radius) - distance

                # Separate the balls
                self.x -= 0.5 * penetration_depth * normal_x
                self.y -= 0.5 * penetration_depth * normal_y
                other_ball.x += 0.5 * penetration_depth * normal_x
                other_ball.y += 0.5 * penetration_depth * normal_y

                # Calculate relative velocity
                rel_vx = other_ball.vx - self.vx
                rel_vy = other_ball.vy - self.vy

                # Calculate the dot product of relative velocity and normal
                dot_product = rel_vx * normal_x + rel_vy * normal_y

                # Apply impulse (reflection) only if balls are moving towards each other
                if dot_product > 0:  # Ensure balls are moving towards each other
                    return

                impulse = 2 * dot_product / (1 + 1)  # Assume equal mass

                self.vx += impulse * normal_x
                self.vy += impulse * normal_y
                other_ball.vx -= impulse * normal_x
                other_ball.vy -= impulse * normal_y

                # Spin transfer
                self.spin += (other_ball.spin) * 0.5
                other_ball.spin += (self.spin) * 0.5


class SpinningHeptagonApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Spinning Heptagon with Bouncing Balls")
        self.canvas = tk.Canvas(
            root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white"
        )
        self.canvas.pack()
        self.balls: List[Ball] = []
        self.angle_offset = 0
        self.heptagon_coords = self.calculate_heptagon_coordinates()

        self.create_balls()
        self.draw_heptagon()  # Initial draw
        self.update()

    def calculate_heptagon_coordinates(self):
        """Calculates the coordinates of the heptagon's vertices."""
        coords = []
        for i in range(7):
            angle = math.radians(i * 360 / 7)
            x = CANVAS_WIDTH / 2 + HEPTAGON_RADIUS * math.cos(angle)
            y = CANVAS_HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(angle)
            coords.append((x, y))
        return coords

    def draw_heptagon(self):
        """Draws the spinning heptagon."""
        self.heptagon_coords = self.calculate_heptagon_coordinates()
        rotated_coords = []
        for i in range(7):
            angle = math.radians(self.angle_offset + i * 360 / 7)
            x = CANVAS_WIDTH / 2 + HEPTAGON_RADIUS * math.cos(angle)
            y = CANVAS_HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(angle)
            rotated_coords.append((x, y))

        # Clear and redraw the heptagon
        self.canvas.delete("heptagon")
        for i in range(7):
            x1, y1 = rotated_coords[i]
            x2, y2 = rotated_coords[(i + 1) % 7]
            self.canvas.create_line(
                x1, y1, x2, y2, fill="black", width=2, tags="heptagon"
            )

    def create_balls(self):
        """Creates the balls and places them at the center."""
        for i in range(NUM_BALLS):
            color = BALL_COLORS[i % len(BALL_COLORS)]
            self.balls.append(
                Ball(
                    x=CANVAS_WIDTH // 2,
                    y=CANVAS_HEIGHT // 2,
                    vx=0,
                    vy=0,
                    spin=0,
                    number=i + 1,
                    color=color,
                    radius=BALL_RADIUS,
                )
            )

    def update(self):
        """Updates the game state and redraws the canvas."""
        self.angle_offset = (
            self.angle_offset + HEPTAGON_SPIN_SPEED / 30
        ) % 360  # Spin speed adjusted for frame rate
        self.draw_heptagon()
        self.canvas.delete("ball")  # Clear previous ball drawings

        for ball in self.balls:
            ball.update(self.balls, self.angle_offset)
            ball.draw(self.canvas, self.angle_offset)  # Draw after update

        self.root.after(30, self.update)  # Adjust the delay for desired frame rate


if __name__ == "__main__":
    root = tk.Tk()
    app = SpinningHeptagonApp(root)
    root.mainloop()
