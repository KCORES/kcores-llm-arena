import tkinter as tk
import math
import numpy as np
import dataclasses
import typing
import sys


@dataclasses.dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    rotation: float  # Angle of rotation in radians
    rotation_speed: float


@dataclasses.dataclass
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    rotation: float  # Angle of rotation in radians
    rotation_speed: float


def calculate_heptagon_vertex(center_x, center_y, heptagon_radius, angle):
    """Calculates the x and y coordinates of a heptagon vertex."""
    x = center_x + heptagon_radius * math.cos(angle)
    y = center_y + heptagon_radius * math.sin(angle)
    return x, y


def distance(x1, y1, x2, y2):
    """Calculates the distance between two points."""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def check_collision(ball1, ball2):
    """Checks for collision between two balls."""
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    return dist <= ball1.radius + ball2.radius


def resolve_collision(ball1, ball2):
    """Resolves collision between two balls."""
    # Calculate the normal vector
    nx = ball2.x - ball1.x
    ny = ball2.y - ball1.y
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    nx /= dist
    ny /= dist

    # Calculate the relative velocity
    dvx = ball1.vx - ball2.vx
    dvy = ball1.vy - ball2.vy

    # Calculate the dot product of the relative velocity and the normal vector
    dot_product = dvx * nx + dvy * ny

    # If the balls are moving away from each other, no collision response is needed
    if dot_product > 0:
        return

    # Calculate the impulse
    impulse = (2 * dot_product) / (1 / 1 + 1 / 1)  # Assuming mass = 1 for both balls

    # Apply the impulse to the velocities
    ball1.vx -= impulse * nx
    ball1.vy -= impulse * ny
    ball2.vx += impulse * nx
    ball2.vy += impulse * ny


def bounce_off_heptagon(ball, heptagon):
    """Handles the bouncing of a ball off the heptagon walls."""
    for i in range(7):
        angle = 2 * math.pi * i / 7
        x, y = calculate_heptagon_vertex(
            heptagon.center_x, heptagon.center_y, heptagon.radius, angle
        )
        x2, y2 = calculate_heptagon_vertex(
            heptagon.center_x,
            heptagon.center_y,
            heptagon.radius,
            (i + 1) * 2 * math.pi / 7,
        )

        # Calculate distance from ball to the line segment
        dx = x2 - x
        dy = y2 - y
        t = ((ball.x - x) * dx + (ball.y - y) * dy) / (dx * dx + dy * dy)

        if 0 <= t <= 1:
            closest_x = x + t * dx
            closest_y = y + t * dy
            dist = distance(ball.x, ball.y, closest_x, closest_y)
            if dist <= ball.radius:
                # Normalize the vector from the wall to the ball
                nx = ball.x - closest_x
                ny = ball.y - closest_y
                norm = distance(0, 0, nx, ny)
                nx /= norm
                ny /= norm

                # Calculate the relative velocity along the normal
                dot_product = ball.vx * nx + ball.vy * ny

                # Apply the bounce (reverse the normal component of the velocity)
                ball.vx -= 2 * dot_product * nx
                ball.vy -= 2 * dot_product * ny

                # Friction
                friction = 0.8
                ball.vx *= friction
                ball.vy *= friction
                break


class BallSimulation:
    def __init__(self, master, width=800, height=600):
        self.master = master
        master.title("Bouncing Balls in a Spinning Heptagon")
        self.width = width
        self.height = height
        self.canvas = tk.Canvas(
            master, width=self.width, height=self.height, bg="black"
        )
        self.canvas.pack()

        self.colors = [
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
        self.balls = []
        self.heptagon = Heptagon(
            self.width / 2,
            self.height / 2,
            min(self.width, self.height) / 3,
            0,
            360 / (5 * 60),
        )  # 360 degrees in 5 seconds

        self.create_balls(20)
        self.animate()

    def create_balls(self, num_balls):
        """Creates the balls with initial conditions."""
        for i in range(num_balls):
            radius = 15
            x = self.width / 2
            y = self.height / 2
            vx = np.random.uniform(-5, 5)
            vy = np.random.uniform(-5, 5)
            color = self.colors[i % len(self.colors)]
            rotation_speed = np.random.uniform(-1, 1)

            ball = Ball(x, y, vx, vy, radius, color, i + 1, 0, rotation_speed)
            self.balls.append(ball)

    def animate(self):
        """Animates the bouncing balls and rotating heptagon."""
        self.canvas.delete("all")

        # Update heptagon rotation
        self.heptagon.rotation += self.heptagon.rotation_speed / 60  # 60 frames

        # Draw heptagon
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.heptagon.rotation
            x, y = calculate_heptagon_vertex(
                self.heptagon.center_x,
                self.heptagon.center_y,
                self.heptagon.radius,
                angle,
            )
            x2, y2 = calculate_heptagon_vertex(
                self.heptagon.center_x,
                self.heptagon.center_y,
                self.heptagon.radius,
                (i + 1) * 2 * math.pi / 7 + self.heptagon.rotation,
            )
            self.canvas.create_line(x, y, x2, y2, fill="white", width=2)

        # Update ball positions
        for ball in self.balls:
            # Apply gravity
            gravity = 0.5
            ball.vy += gravity

            # Apply friction
            friction = 0.02
            ball.vx *= 1 - friction
            ball.vy *= 1 - friction

            # Update ball position
            ball.x += ball.vx
            ball.y += ball.vy

            # Update ball rotation
            ball.rotation += ball.rotation_speed / 60

            # Bounce off walls
            if ball.x - ball.radius < 0 or ball.x + ball.radius > self.width:
                ball.vx *= -1
            if ball.y - ball.radius < 0 or ball.y + ball.radius > self.height:
                ball.vy *= -1

            # Bounce off heptagon
            bounce_off_heptagon(ball, self.heptagon)

        # Check collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                if check_collision(self.balls[i], self.balls[j]):
                    resolve_collision(self.balls[i], self.balls[j])

        # Draw balls
        for ball in self.balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            self.canvas.create_oval(x0, y0, x1, y1, fill=ball.color)
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), fill="black", font=("Arial", 10)
            )

        self.master.after(16, self.animate)  # Approximately 60 frames per second


if __name__ == "__main__":
    root = tk.Tk()
    simulation = BallSimulation(root)
    root.mainloop()
