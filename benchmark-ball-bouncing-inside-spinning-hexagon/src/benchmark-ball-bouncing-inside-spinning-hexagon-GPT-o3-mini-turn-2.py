import tkinter as tk
import math
import numpy as np
import random
from dataclasses import dataclass
from typing import List

# ---------------------
# Simulation Parameters
# ---------------------
WIDTH = 800  # Canvas width in pixels
HEIGHT = 800  # Canvas height in pixels
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
HEPTAGON_RADIUS = 300  # Radius of the circumscribed circle of the heptagon
BALL_RADIUS = 15  # Radius of each ball
NUM_BALLS = 20  # Total number of balls
GRAVITY = np.array([0, 400])  # Gravity vector (pixels/s^2)
DT = 0.02  # Time step in seconds
HEPTAGON_ANGULAR_VELOCITY = (
    2 * math.pi / 5
)  # Angular speed of heptagon in radians per second

# List of ball colors (each ball has a unique color)
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

# Coefficients of restitution
WALL_RESTITUTION = 0.7  # Bounce restitution for wall collisions
BALL_RESTITUTION = 0.9  # Bounce restitution for ball-ball collisions


# ---------------------
# Ball Data Class
# ---------------------
@dataclass
class Ball:
    num: int  # Ball number (1 to 20)
    position: np.ndarray  # 2D position vector
    velocity: np.ndarray  # 2D velocity vector
    radius: float  # Ball radius
    color: str  # Fill color (hex code)
    angle: float  # Spin angle (for drawing spin indicator)
    angular_velocity: float  # Spin rate (radians per second)


# ---------------------
# Simulation Class
# ---------------------
class Simulation:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.time = 0.0
        self.heptagon_angle = 0.0  # Current rotation angle of the heptagon
        self.balls: List[Ball] = []
        # Initialize balls at the heptagon center with a slight random velocity and spin
        for i in range(NUM_BALLS):
            # Small random jitter to break perfect symmetry
            vel = np.array([random.uniform(-30, 30), random.uniform(-30, 30)])
            ang_vel = random.uniform(-5, 5)
            b = Ball(
                num=i + 1,
                position=CENTER.copy(),
                velocity=vel,
                radius=BALL_RADIUS,
                color=COLORS[i],
                angle=0.0,
                angular_velocity=ang_vel,
            )
            self.balls.append(b)

    def update(self):
        # Update simulation time and heptagon rotation
        self.time += DT
        self.heptagon_angle = (self.heptagon_angle + HEPTAGON_ANGULAR_VELOCITY * DT) % (
            2 * math.pi
        )

        # Update each ball's kinematics
        for ball in self.balls:
            # Apply gravity
            ball.velocity += GRAVITY * DT
            # Update position
            ball.position += ball.velocity * DT
            # Update spin angle
            ball.angle += ball.angular_velocity * DT
            # Apply angular friction
            ball.angular_velocity *= 0.99
            # Apply slight damping to translational motion (air resistance)
            ball.velocity *= 0.999

        # Handle collisions with heptagon walls
        self.collide_with_walls()
        # Handle collisions between balls
        self.collide_between_balls()

        # Redraw the simulation
        self.draw()
        # Schedule next update
        self.canvas.after(int(DT * 1000), self.update)

    def get_heptagon_vertices(self):
        """
        Compute the vertices of the rotating regular heptagon.
        """
        vertices = []
        # Vertices are computed based on the current rotation angle.
        for i in range(7):
            theta = self.heptagon_angle + (2 * math.pi * i) / 7
            x = CENTER[0] + HEPTAGON_RADIUS * math.cos(theta)
            y = CENTER[1] + HEPTAGON_RADIUS * math.sin(theta)
            vertices.append(np.array([x, y]))
        return vertices

    def collide_with_walls(self):
        """
        Check and resolve collisions between balls and the heptagon walls.
        """
        vertices = self.get_heptagon_vertices()
        for ball in self.balls:
            for i in range(len(vertices)):
                A = vertices[i]
                B = vertices[(i + 1) % len(vertices)]
                # Compute the closest point Q on edge AB to the ball's position.
                AB = B - A
                AB_length_sq = np.dot(AB, AB)
                if AB_length_sq == 0:
                    continue
                t = np.dot(ball.position - A, AB) / AB_length_sq
                t = max(0, min(1, t))
                Q = A + t * AB
                diff = ball.position - Q
                distance = np.linalg.norm(diff)
                if distance < ball.radius:
                    # Collision detected with the wall segment
                    if distance == 0:
                        normal = np.array([1, 0])
                    else:
                        normal = diff / distance
                    # Compute the wall's velocity at contact point Q due to heptagon rotation.
                    r_vec = Q - CENTER
                    # In 2D, the tangential velocity is given by ω cross r = ω * (-r_y, r_x)
                    wall_velocity = HEPTAGON_ANGULAR_VELOCITY * np.array(
                        [-r_vec[1], r_vec[0]]
                    )
                    relative_velocity = ball.velocity - wall_velocity
                    vn = np.dot(relative_velocity, normal)
                    if vn < 0:
                        # Reflect the ball's velocity relative to the moving wall.
                        ball.velocity = (
                            ball.velocity - (1 + WALL_RESTITUTION) * vn * normal
                        )
                        # Correct the ball's position to eliminate penetration.
                        penetration = ball.radius - distance
                        ball.position += normal * penetration

    def collide_between_balls(self):
        """
        Check and resolve collisions between balls using simple elastic collision response.
        """
        n = len(self.balls)
        for i in range(n):
            for j in range(i + 1, n):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                delta = ball1.position - ball2.position
                dist = np.linalg.norm(delta)
                min_dist = ball1.radius + ball2.radius
                if dist < min_dist and dist != 0:
                    # Collision detected: normalize the collision normal vector.
                    normal = delta / dist
                    # Compute relative velocity in the direction of the normal.
                    relative_velocity = ball1.velocity - ball2.velocity
                    vn = np.dot(relative_velocity, normal)
                    if vn < 0:
                        # Apply impulse to both balls (equal mass assumed).
                        impulse = -(1 + BALL_RESTITUTION) * vn / 2
                        ball1.velocity += impulse * normal
                        ball2.velocity -= impulse * normal
                    # Positional correction to resolve overlap (separate balls equally).
                    overlap = min_dist - dist
                    correction = normal * (overlap / 2)
                    ball1.position += correction
                    ball2.position -= correction

    def draw(self):
        """
        Redraw the entire simulation: heptagon and all balls.
        """
        self.canvas.delete("all")
        # Draw the heptagon
        vertices = self.get_heptagon_vertices()
        poly_coords = []
        for v in vertices:
            poly_coords.extend(v.tolist())
        self.canvas.create_polygon(poly_coords, outline="black", fill="", width=2)

        # Draw each ball
        for ball in self.balls:
            x, y = ball.position.tolist()
            r = ball.radius
            # Draw the ball as a circle with a black outline.
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
            )
            # Draw the ball number centered on the ball.
            self.canvas.create_text(
                x, y, text=str(ball.num), fill="white", font=("Helvetica", 12, "bold")
            )
            # Draw a line from the center to indicate the ball's current spin direction.
            line_length = r
            dx = line_length * math.cos(ball.angle)
            dy = line_length * math.sin(ball.angle)
            self.canvas.create_line(x, y, x + dx, y + dy, fill="white", width=2)


# ---------------------
# Main function
# ---------------------
def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()
    sim = Simulation(canvas)
    sim.update()
    root.mainloop()


if __name__ == "__main__":
    main()
