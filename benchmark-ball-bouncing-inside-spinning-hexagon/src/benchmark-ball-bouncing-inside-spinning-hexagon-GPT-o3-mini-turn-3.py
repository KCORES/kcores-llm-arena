# Bouncing Balls inside a Spinning Heptagon Simulation
# This simulation uses tkinter for visualization, numpy for numerical operations,
# and implements collisions between balls, collisions with rotating heptagon walls,
# along with gravity and friction effects.

import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List


# Ball dataclass to store properties of each ball
@dataclass
class Ball:
    id: int  # Ball number (1 to 20)
    pos: np.ndarray  # 2D position vector
    vel: np.ndarray  # 2D velocity vector
    angle: float  # Orientation angle (radians)
    angular_velocity: float  # Angular velocity (radians per second)
    color: str  # Fill color
    radius: float  # Ball radius


# Main simulation class
class BouncingBallsSim:
    def __init__(self):
        # Setup window dimensions and canvas center
        self.width = 800
        self.height = 600
        self.center = np.array([self.width / 2, self.height / 2])

        # Initialize tkinter window and canvas
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(
            self.root, width=self.width, height=self.height, bg="black"
        )
        self.canvas.pack()

        # Simulation parameters
        self.ball_radius = 15
        self.heptagon_radius = (
            250  # Radius of the heptagon (distance from center to vertices)
        )
        self.heptagon_angle = 0.0  # Current rotation angle of the heptagon (radians)
        self.heptagon_angular_velocity = (
            2 * math.pi / 5
        )  # Rotation speed: 360° per 5 seconds

        self.gravity = np.array([0, 500])  # Gravity vector (pixels/s²)
        self.friction = 0.999  # Linear friction factor per frame
        self.angular_friction = 0.99  # Rotational friction factor per frame

        self.wall_restitution = 0.7  # Restitution coefficient for wall collisions
        self.ball_restitution = 0.9  # Restitution coefficient for ball collisions

        self.dt = 0.02  # Time step for simulation (in seconds)

        # Colors for the 20 balls, as provided
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

        # Initialize balls at the center with small random initial velocities and spins
        self.balls: List[Ball] = []
        for i in range(20):
            pos = np.copy(self.center)
            # Small random initial velocity in both x and y directions
            velocity = np.random.uniform(-50, 50, size=2)
            # Initial orientation angle is 0, with a small random angular velocity
            angle = 0.0
            angular_velocity = np.random.uniform(-10, 10)
            ball = Ball(
                id=i + 1,
                pos=pos,
                vel=velocity,
                angle=angle,
                angular_velocity=angular_velocity,
                color=self.colors[i],
                radius=self.ball_radius,
            )
            self.balls.append(ball)

        # Start the simulation loop
        self.run_simulation()

    def compute_heptagon_vertices(self) -> List[np.ndarray]:
        """
        Compute the current vertices of the rotating heptagon.
        """
        vertices = []
        for i in range(7):
            angle = self.heptagon_angle + i * (2 * math.pi / 7)
            x = self.center[0] + self.heptagon_radius * math.cos(angle)
            y = self.center[1] + self.heptagon_radius * math.sin(angle)
            vertices.append(np.array([x, y]))
        return vertices

    def update(self):
        """
        Update the state of the simulation.
        """
        # Update heptagon rotation angle
        self.heptagon_angle += self.heptagon_angular_velocity * self.dt
        vertices = self.compute_heptagon_vertices()

        # Update each ball's state
        for ball in self.balls:
            # Apply gravity to change velocity
            ball.vel += self.gravity * self.dt
            # Update position based on velocity
            ball.pos += ball.vel * self.dt
            # Update rotation angle based on angular velocity
            ball.angle += ball.angular_velocity * self.dt

            # Apply friction to gradually reduce velocities
            ball.vel *= self.friction
            ball.angular_velocity *= self.angular_friction

            # Handle collisions with the heptagon walls
            for i in range(len(vertices)):
                A = vertices[i]
                B = vertices[(i + 1) % len(vertices)]
                # Compute projection of ball center onto the wall segment AB
                AB = B - A
                AB_length_sq = np.dot(AB, AB)
                if AB_length_sq == 0:
                    continue
                t = np.dot(ball.pos - A, AB) / AB_length_sq
                t = max(0, min(1, t))
                closest = A + t * AB
                diff = ball.pos - closest
                dist = np.linalg.norm(diff)
                if dist < ball.radius:
                    # Collision detected with wall segment
                    if dist == 0:
                        # If exactly on the wall, choose a perpendicular normal
                        normal = np.array([-AB[1], AB[0]])
                        normal /= np.linalg.norm(normal)
                    else:
                        normal = diff / dist
                    penetration = ball.radius - dist
                    # Resolve penetration by moving ball out of wall collision
                    ball.pos += normal * penetration

                    # Compute wall velocity at the collision point (due to heptagon rotation)
                    r = closest - self.center
                    wall_vel = self.heptagon_angular_velocity * np.array([-r[1], r[0]])

                    # Calculate relative velocity between ball and wall
                    rel_vel = ball.vel - wall_vel
                    rel_speed_normal = np.dot(rel_vel, normal)
                    if rel_speed_normal < 0:
                        # Reflect the normal component of the velocity
                        impulse = -(1 + self.wall_restitution) * rel_speed_normal
                        ball.vel += impulse * normal
                        # Optionally, modify angular velocity due to friction from collision
                        tangential = rel_vel - rel_speed_normal * normal
                        if np.linalg.norm(tangential) != 0:
                            ball.angular_velocity += (
                                np.linalg.norm(tangential)
                                * 0.1
                                * (
                                    -np.sign(ball.angular_velocity)
                                    if ball.angular_velocity != 0
                                    else 0
                                )
                            )

        # Handle collisions between balls
        n = len(self.balls)
        for i in range(n):
            for j in range(i + 1, n):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                delta = ball1.pos - ball2.pos
                dist = np.linalg.norm(delta)
                if dist < 2 * self.ball_radius and dist != 0:
                    # Compute collision normal and penetration depth
                    normal = delta / dist
                    penetration = 2 * self.ball_radius - dist
                    # Separate overlapping balls
                    ball1.pos += normal * (penetration / 2)
                    ball2.pos -= normal * (penetration / 2)

                    # Adjust velocities based on collision (elastic collision with restitution)
                    rel_vel = ball1.vel - ball2.vel
                    rel_speed = np.dot(rel_vel, normal)
                    if rel_speed < 0:
                        impulse = -(1 + self.ball_restitution) * rel_speed / 2
                        ball1.vel += impulse * normal
                        ball2.vel -= impulse * normal

        # Redraw simulation
        self.draw(vertices)
        self.root.after(int(self.dt * 1000), self.update)

    def draw(self, vertices: List[np.ndarray]):
        """
        Draw the heptagon and all balls on the canvas.
        """
        self.canvas.delete("all")

        # Draw the rotating heptagon
        points = []
        for v in vertices:
            points.extend(v.tolist())
        self.canvas.create_polygon(points, outline="white", fill="", width=2)

        # Draw each ball
        for ball in self.balls:
            x, y = ball.pos
            r = ball.radius
            # Draw ball circle
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
            )
            # Draw ball number at the center
            self.canvas.create_text(
                x, y, text=str(ball.id), fill="black", font=("Helvetica", 10, "bold")
            )
            # Draw a line from the center to the edge to indicate the ball's rotation (spin)
            line_length = r
            lx = x + line_length * math.cos(ball.angle)
            ly = y + line_length * math.sin(ball.angle)
            self.canvas.create_line(x, y, lx, ly, fill="black", width=2)

    def run_simulation(self):
        """
        Start the simulation loop.
        """
        self.update()
        self.root.mainloop()


if __name__ == "__main__":
    sim = BouncingBallsSim()
