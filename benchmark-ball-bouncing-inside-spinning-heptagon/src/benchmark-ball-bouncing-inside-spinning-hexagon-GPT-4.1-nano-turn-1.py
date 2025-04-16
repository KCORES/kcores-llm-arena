import tkinter as tk
import math
import time
import numpy as np
from dataclasses import dataclass
from typing import List

# ==================== Configuration ====================

WIDTH, HEIGHT = 800, 800

BALL_RADIUS = 10
NUM_BALLS = 20

# Colors in hex
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

# Gravity and physics
GRAVITY = 980  # pixels/sec^2, approximate gravity
FRICTION_COEFF = 0.9  # energy loss on bounce
AIR_RESISTANCE = 0.99  # per frame air resistance

# Heptagon parameters
HEPTAGON_SIZE = 300  # radius of the circumscribed circle
SPIN_SPEED_DEG_PER_SEC = 72  # 360 degrees / 5 seconds
FRAME_RATE = 60  # frames per second

# Calculate spin per frame
SPIN_SPEED_RAD_PER_SEC = math.radians(SPIN_SPEED_DEG_PER_SEC)
SPIN_SPEED_RAD_PER_FRAME = SPIN_SPEED_RAD_PER_SEC / FRAME_RATE

# ==================== Data Classes ====================


@dataclass
class Ball:
    position: np.ndarray  # [x, y]
    velocity: np.ndarray  # [vx, vy]
    color: str
    number: int
    spin: float  # in radians, representing rotation angle
    spin_velocity: float  # radians/sec
    radius: float = BALL_RADIUS


@dataclass
class Wall:
    start: np.ndarray
    end: np.ndarray
    normal: np.ndarray  # Unit normal vector pointing inward


# ==================== Helper Functions ====================


def create_heptagon(center, size):
    vertices = []
    for i in range(7):
        angle_deg = 360 / 7 * i
        angle_rad = math.radians(angle_deg)
        x = center[0] + size * math.cos(angle_rad)
        y = center[1] + size * math.sin(angle_rad)
        vertices.append(np.array([x, y]))
    return vertices


def compute_walls(vertices):
    walls = []
    n = len(vertices)
    for i in range(n):
        start = vertices[i]
        end = vertices[(i + 1) % n]
        # Wall direction vector
        wall_dir = end - start
        # Inward normal (perpendicular to wall)
        normal = np.array([-wall_dir[1], wall_dir[0]])
        normal = normal / np.linalg.norm(normal)
        # For a convex polygon, normal points inward if cross product with edge points inward
        # Check direction by cross product with vector from start to center
        center = np.mean(vertices, axis=0)
        to_center = center - start
        if np.cross(wall_dir, to_center) < 0:
            normal = -normal
        walls.append(Wall(start=start, end=end, normal=normal))
    return walls


def point_in_polygon(point, vertices):
    # Ray casting algorithm
    count = 0
    x, y = point
    for i in range(len(vertices)):
        v1 = vertices[i]
        v2 = vertices[(i + 1) % len(vertices)]
        if ((v1[1] > y) != (v2[1] > y)) and (
            x < (v2[0] - v1[0]) * (y - v1[1]) / (v2[1] - v1[1] + 1e-10) + v1[0]
        ):
            count += 1
    return count % 2 == 1


def closest_point_on_segment(point, start, end):
    # Returns the closest point on segment to point
    seg = end - start
    t = np.dot(point - start, seg) / np.dot(seg, seg)
    t = max(0, min(1, t))
    return start + t * seg


def rotate_point(point, angle_rad, center):
    # Rotate point around center by angle_rad
    translated = point - center
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    rotated = np.array(
        [
            translated[0] * cos_a - translated[1] * sin_a,
            translated[0] * sin_a + translated[1] * cos_a,
        ]
    )
    return rotated + center


# ==================== Main Application ====================


class BouncingBallsApp:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.center = np.array([WIDTH / 2, HEIGHT / 2])
        self.vertices = create_heptagon(self.center, HEPTAGON_SIZE)
        self.walls = compute_walls(self.vertices)

        # Initialize balls
        self.balls: List[Ball] = []
        for i in range(NUM_BALLS):
            color = COLORS[i % len(COLORS)]
            number = i + 1
            # Drop from center with zero initial velocity
            position = np.array(self.center)
            velocity = np.array([0.0, 0.0])
            spin = 0.0
            spin_velocity = 0.0
            self.balls.append(
                Ball(position, velocity, color, number, spin, spin_velocity)
            )

        self.last_time = None
        self.running = True

        # Start animation
        self.animate()

    def animate(self):
        current_time = time.time()
        if self.last_time is None:
            dt = 1 / FRAME_RATE
        else:
            dt = current_time - self.last_time
        self.last_time = current_time

        self.update_balls(dt)
        self.draw()
        if self.running:
            self.master.after(int(1000 / FRAME_RATE), self.animate)

    def update_balls(self, dt):
        for ball in self.balls:
            # Apply gravity
            ball.velocity[1] += GRAVITY * dt

            # Apply air resistance
            ball.velocity *= AIR_RESISTANCE

            # Update position
            ball.position += ball.velocity * dt

            # Update spin angle
            ball.spin += ball.spin_velocity * dt

            # Constraints: check collision with walls
            for wall in self.walls:
                self.resolve_wall_collision(ball, wall, dt)

            # Check collision with other balls
            for other in self.balls:
                if other is not ball:
                    self.resolve_ball_collision(ball, other, dt)

    def resolve_wall_collision(self, ball: Ball, wall: Wall, dt):
        # Find closest point on wall segment to ball
        closest = closest_point_on_segment(ball.position, wall.start, wall.end)
        dist_vector = ball.position - closest
        dist = np.linalg.norm(dist_vector)
        if dist < ball.radius:
            # Compute penetration depth
            penetration = ball.radius - dist
            contact_normal = dist_vector / (dist + 1e-10)
            # Move ball outside
            ball.position += contact_normal * penetration

            # Reflect velocity
            v_normal = np.dot(ball.velocity, contact_normal) * contact_normal
            v_tangent = ball.velocity - v_normal
            ball.velocity = v_tangent - v_normal * FRICTION_COEFF
            # Apply friction to spin (simulate frictional torque)
            ball.spin_velocity *= FRICTION_COEFF

            # Add some bounce effect based on impact
            # For realistic bounce not exceeding radius height
            impact_speed = np.dot(ball.velocity, contact_normal)
            if impact_speed < 0:
                ball.velocity -= 2 * impact_speed * contact_normal
                ball.velocity *= FRICTION_COEFF

    def resolve_ball_collision(self, ball1: Ball, ball2: Ball, dt):
        delta_pos = ball2.position - ball1.position
        dist = np.linalg.norm(delta_pos)
        min_dist = ball1.radius + ball2.radius
        if dist < min_dist:
            # Overlap correction
            overlap = min_dist - dist
            if dist > 1e-10:
                correction = delta_pos / dist * (overlap / 2)
                ball1.position -= correction
                ball2.position += correction
            else:
                # Exactly at same position, push apart arbitrarily
                correction = np.array([ball1.radius, 0])
                ball1.position -= correction
                ball2.position += correction
                delta_pos = ball2.position - ball1.position
                dist = np.linalg.norm(delta_pos)

            # Compute relative velocity
            rel_vel = ball2.velocity - ball1.velocity
            normal = delta_pos / (dist + 1e-10)
            vel_along_normal = np.dot(rel_vel, normal)

            # Only resolve if balls are moving towards each other
            if vel_along_normal < 0:
                # Coefficient of restitution (elastic collision)
                restitution = 0.9
                impulse_mag = -(1 + restitution) * vel_along_normal
                # Assuming equal mass
                impulse = impulse_mag * normal / 2
                ball1.velocity += impulse
                ball2.velocity -= impulse

                # Spin exchange (rough approximation)
                # If spinning influences collision response
                # (Optional: can be more complex)

    def draw(self):
        self.canvas.delete("all")
        # Draw heptagon
        self.draw_heptagon()

        # Draw balls
        for ball in self.balls:
            # Draw ball with number
            x, y = ball.position
            r = ball.radius
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
            )
            self.canvas.create_text(
                x,
                y,
                text=str(ball.number),
                fill="black",
                font=("Arial", int(r), "bold"),
            )

    def draw_heptagon(self):
        # Draw spinning heptagon (for visualization)
        points = []
        for vertex in self.vertices:
            # Rotate vertices about center by current spin angle
            rotated_point = rotate_point(vertex, 0, self.center)
            points.extend(rotated_point)
        # Draw polygon
        coords = []
        for v in points:
            coords.extend(v)
        self.canvas.create_polygon(coords, outline="black", fill="", width=2)

        # Draw the rotating lines for effect
        # For more realism, you can add animated edges or a background pattern
        # For simplicity, only the outline is drawn

        # Update the vertices for next frame based on spinning
        # Since the heptagon rotates with a fixed speed
        angle = SPIN_SPEED_RAD_PER_FRAME
        new_vertices = []
        for v in self.vertices:
            rotated_v = rotate_point(v, angle, self.center)
            new_vertices.append(rotated_v)
        self.vertices = new_vertices
        self.walls = compute_walls(self.vertices)

    def run(self):
        self.master.mainloop()


# ==================== Run the Application ====================

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Balls Bouncing Inside Spinning Heptagon")
    app = BouncingBallsApp(root)
    app.run()
