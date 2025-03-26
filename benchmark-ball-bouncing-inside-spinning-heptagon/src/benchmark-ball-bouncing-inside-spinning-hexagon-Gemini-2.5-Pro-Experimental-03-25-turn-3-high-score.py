import tkinter as tk
import math
import numpy as np
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

# --- Constants ---
WIDTH, HEIGHT = 800, 800
NUM_BALLS = 20
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Distance from center to vertex
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
GRAVITY = np.array([0, 981.0])  # Pixels/s^2 (Adjust scale as needed)
TIME_STEP = (
    1 / 100
)  # Simulation time step in seconds (smaller is more accurate but slower)
UPDATE_MS = int(TIME_STEP * 1000)  # Update interval for tkinter in milliseconds

# Heptagon spin: 360 degrees in 5 seconds = 72 degrees/s = 1.2566 rad/s
HEPTAGON_ANGULAR_VELOCITY = math.radians(360 / 5.0)

# Collision properties
COEFF_RESTITUTION = 0.7  # Bounciness (0 = no bounce, 1 = perfect bounce)
# Ensure bounce height constraint: max < HEPTAGON_RADIUS, min > BALL_RADIUS
# Derived from e = sqrt(bounce_height / drop_height). Our 'e' limits the velocity rebound.
# A value of 0.7 implies a bounce height roughly half the drop height (relative to impact).
# Given gravity and initial drop from center, this should satisfy the constraint.

COEFF_FRICTION_BALL_WALL = 0.3  # Friction between ball and wall
COEFF_FRICTION_BALL_BALL = 0.2  # Friction between balls
AIR_FRICTION_LINEAR = 0.01  # Simple linear drag factor
AIR_FRICTION_ANGULAR = 0.05  # Simple angular drag factor


# --- Utility Functions ---
def rotate_point(point: np.ndarray, angle: float, center: np.ndarray) -> np.ndarray:
    """Rotates a point around a center."""
    relative_point = point - center
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    rotated_x = relative_point[0] * cos_a - relative_point[1] * sin_a
    rotated_y = relative_point[0] * sin_a + relative_point[1] * cos_a
    return np.array([rotated_x, rotated_y]) + center


def point_segment_distance(
    p: np.ndarray, a: np.ndarray, b: np.ndarray
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Calculates the shortest distance from point p to line segment ab.
    Returns distance, closest point on the infinite line, closest point on the segment.
    """
    ap = p - a
    ab = b - a
    ab_squared = np.dot(ab, ab)

    if ab_squared == 0.0:  # Segment is a point
        closest_on_line = a
        closest_on_segment = a
        dist_sq = np.dot(ap, ap)
    else:
        # Project p onto the line containing the segment
        t = np.dot(ap, ab) / ab_squared
        closest_on_line = a + t * ab

        # Check if the projection falls onto the segment
        if t < 0.0:
            closest_on_segment = a  # Closest to endpoint a
        elif t > 1.0:
            closest_on_segment = b  # Closest to endpoint b
        else:
            closest_on_segment = closest_on_line  # Projection is on the segment

        dist_vec = p - closest_on_segment
        dist_sq = np.dot(dist_vec, dist_vec)

    distance = (
        math.sqrt(dist_sq) if dist_sq > 1e-12 else 0.0
    )  # Avoid sqrt(negative) due to precision
    return distance, closest_on_line, closest_on_segment


def normalize(v: np.ndarray) -> np.ndarray:
    """Normalizes a numpy array (vector). Returns zero vector if magnitude is zero."""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


# --- Data Classes ---
@dataclass
class Ball:
    id: int
    radius: float
    color: str
    mass: float = field(init=False)
    inertia: float = field(init=False)  # Moment of inertia (for rotation)
    position: np.ndarray = field(
        default_factory=lambda: np.array([WIDTH / 2, HEIGHT / 2], dtype=float)
    )
    velocity: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0], dtype=float)
    )
    angle: float = 0.0  # Radians
    angular_velocity: float = 0.0  # Radians per second
    canvas_item: int = None  # Store canvas object ID for ball
    text_item: int = None  # Store canvas object ID for text

    def __post_init__(self):
        # Assume uniform density sphere/disk
        self.mass = math.pi * self.radius**2  # Area proportional for 2D
        # Moment of inertia for a solid disk: 0.5 * m * r^2
        self.inertia = 0.5 * self.mass * self.radius**2

    def update(self, dt: float):
        """Update ball position and angle based on velocity."""
        # Apply gravity
        self.velocity += GRAVITY * dt

        # Apply simple air friction (drag)
        self.velocity *= 1 - AIR_FRICTION_LINEAR * dt
        self.angular_velocity *= 1 - AIR_FRICTION_ANGULAR * dt

        # Update position and angle
        self.position += self.velocity * dt
        self.angle += self.angular_velocity * dt

    def draw(self, canvas: tk.Canvas):
        """Draw the ball and its number on the canvas."""
        x0 = self.position[0] - self.radius
        y0 = self.position[1] - self.radius
        x1 = self.position[0] + self.radius
        y1 = self.position[1] + self.radius

        if self.canvas_item is None:
            self.canvas_item = canvas.create_oval(
                x0, y0, x1, y1, fill=self.color, outline="black"
            )
            self.text_item = canvas.create_text(
                self.position[0],
                self.position[1],
                text=str(self.id),
                fill="black",
                font=("Arial", int(self.radius * 0.8), "bold"),
            )
        else:
            canvas.coords(self.canvas_item, x0, y0, x1, y1)
            canvas.coords(self.text_item, self.position[0], self.position[1])
            # Basic rotation indication: move text slightly based on angle
            # (Proper text rotation isn't easily available in tkinter)
            text_offset_scale = self.radius * 0.3
            text_offset_x = text_offset_scale * math.cos(
                self.angle + math.pi / 2
            )  # Offset perpendicular to 'up' direction of number
            text_offset_y = text_offset_scale * math.sin(self.angle + math.pi / 2)
            canvas.coords(
                self.text_item,
                self.position[0] + text_offset_x,
                self.position[1] + text_offset_y,
            )
            # Or simply keep text centered if rotation looks bad:
            # canvas.coords(self.text_item, self.position[0], self.position[1])


@dataclass
class Heptagon:
    center: np.ndarray
    radius: float  # Center to vertex distance
    angle: float = 0.0  # Current rotation angle in radians
    angular_velocity: float = HEPTAGON_ANGULAR_VELOCITY  # Radians per second
    canvas_item: int = None

    def get_vertices(self) -> List[np.ndarray]:
        """Calculate the current vertices of the rotated heptagon."""
        vertices = []
        num_sides = 7
        for i in range(num_sides):
            angle = self.angle + 2 * math.pi * i / num_sides
            x = self.center[0] + self.radius * math.cos(angle)
            y = self.center[1] + self.radius * math.sin(angle)
            vertices.append(np.array([x, y]))
        return vertices

    def get_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Return pairs of vertices representing the edges."""
        vertices = self.get_vertices()
        edges = []
        for i in range(len(vertices)):
            edges.append((vertices[i], vertices[(i + 1) % len(vertices)]))
        return edges

    def update(self, dt: float):
        """Update heptagon rotation."""
        self.angle += self.angular_velocity * dt
        # Keep angle within 0 to 2*pi for clarity, though not strictly necessary for math
        self.angle %= 2 * math.pi

    def draw(self, canvas: tk.Canvas):
        """Draw the heptagon on the canvas."""
        vertices_coords = []
        for v in self.get_vertices():
            vertices_coords.extend(v.tolist())

        if self.canvas_item is None:
            self.canvas_item = canvas.create_polygon(
                vertices_coords, fill="", outline="blue", width=2  # Transparent fill
            )
        else:
            canvas.coords(self.canvas_item, vertices_coords)


# --- Simulation Class ---
class Simulation:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.balls: List[Ball] = []
        self.heptagon = Heptagon(
            center=np.array([WIDTH / 2, HEIGHT / 2]), radius=HEPTAGON_RADIUS
        )
        self._create_balls()
        self.paused = False
        self.canvas.bind("<space>", self.toggle_pause)  # Bind spacebar to pause/unpause

    def toggle_pause(self, event=None):
        self.paused = not self.paused
        if not self.paused:
            print("Resuming simulation")
            self.run()  # Restart the loop if unpausing
        else:
            print("Simulation paused")

    def _create_balls(self):
        """Initialize the balls."""
        start_pos = self.heptagon.center.copy()
        for i in range(NUM_BALLS):
            # Slightly offset initial positions to avoid perfect overlap issues?
            # Or let gravity separate them. Starting exactly at center as requested.
            self.balls.append(
                Ball(
                    id=i + 1,
                    radius=BALL_RADIUS,
                    color=COLORS[i % len(COLORS)],
                    position=start_pos.copy(),
                    # Initial velocity is zero
                )
            )

    def _handle_collisions(self, dt: float):
        """Detect and resolve collisions between balls and with walls."""
        # Ball-Wall Collisions
        edges = self.heptagon.get_edges()
        vertices = self.heptagon.get_vertices()  # Get current vertices

        for ball in self.balls:
            collided_this_step = False
            # Check collision with each edge (wall)
            for i, (v1, v2) in enumerate(edges):
                dist, _, closest_point_on_segment = point_segment_distance(
                    ball.position, v1, v2
                )

                if dist < ball.radius:
                    collided_this_step = True
                    # --- Collision Response ---
                    # 1. Calculate Collision Normal
                    # Vector from closest point on segment to ball center
                    collision_normal = normalize(
                        ball.position - closest_point_on_segment
                    )
                    if (
                        np.linalg.norm(collision_normal) == 0
                    ):  # Avoid zero normal if perfectly centered
                        # Use the normal of the wall segment itself
                        edge_vec = v2 - v1
                        normal_unnorm = np.array(
                            [-edge_vec[1], edge_vec[0]]
                        )  # Perpendicular
                        # Ensure normal points outwards from polygon center
                        if np.dot(normal_unnorm, v1 - self.heptagon.center) < 0:
                            normal_unnorm *= -1
                        collision_normal = normalize(normal_unnorm)
                        if np.linalg.norm(collision_normal) == 0:
                            continue  # Should not happen for non-zero edge

                    # 2. Separate Overlapping Objects
                    overlap = ball.radius - dist
                    if overlap > 0:
                        # Move ball slightly out of the wall
                        ball.position += collision_normal * (
                            overlap + 1e-6
                        )  # Epsilon to prevent re-collision

                    # 3. Calculate Relative Velocity
                    # Wall velocity at collision point P = closest_point_on_segment
                    # v_wall = omega x r = (-omega * ry, omega * rx) where r = P - Center
                    r = closest_point_on_segment - self.heptagon.center
                    wall_velocity = np.array(
                        [
                            -self.heptagon.angular_velocity * r[1],
                            self.heptagon.angular_velocity * r[0],
                        ]
                    )
                    relative_velocity = ball.velocity - wall_velocity

                    # 4. Calculate Impulse (Normal Component - Restitution)
                    vel_along_normal = np.dot(relative_velocity, collision_normal)

                    # Only apply impulse if objects are moving towards each other
                    if vel_along_normal < 0:
                        impulse_j = -(1 + COEFF_RESTITUTION) * vel_along_normal
                        impulse_j /= 1 / ball.mass  # Wall mass is infinite

                        # Apply impulse to ball's velocity
                        ball.velocity += (impulse_j / ball.mass) * collision_normal

                        # 5. Calculate Impulse (Tangential Component - Friction)
                        tangent = np.array([-collision_normal[1], collision_normal[0]])
                        relative_tangential_vel_mag = np.dot(relative_velocity, tangent)

                        # Calculate velocity of ball surface at contact point due to rotation
                        ball_surface_vel_tangential = (
                            -ball.angular_velocity * ball.radius
                        )

                        # Total relative tangential velocity (sliding speed)
                        sliding_velocity = (
                            relative_tangential_vel_mag + ball_surface_vel_tangential
                        )

                        # Calculate friction impulse magnitude (limited by normal impulse)
                        max_friction_impulse = abs(COEFF_FRICTION_BALL_WALL * impulse_j)
                        friction_impulse_magnitude = -sliding_velocity
                        # Adjust denominator if considering inertia: 1/m + r^2/I
                        friction_denominator = (1 / ball.mass) + (
                            ball.radius**2 / ball.inertia
                        )
                        if abs(friction_denominator) > 1e-9:
                            friction_impulse_magnitude /= friction_denominator
                        else:
                            friction_impulse_magnitude = 0

                        # Clamp friction impulse
                        friction_impulse_magnitude = max(
                            -max_friction_impulse,
                            min(max_friction_impulse, friction_impulse_magnitude),
                        )

                        friction_impulse_vector = friction_impulse_magnitude * tangent

                        # Apply friction impulse to linear and angular velocity
                        ball.velocity += friction_impulse_vector / ball.mass
                        torque = (
                            -ball.radius * friction_impulse_magnitude
                        )  # Torque = r x F (magnitude in 2D)
                        ball.angular_velocity += torque / ball.inertia

                    # Optimization: A ball likely only collides with one wall segment per step
                    # We could break here, but checking all allows handling corner cases slightly better
                    # break # Optional: Assume only one wall collision per step

        # Ball-Ball Collisions
        ball_pairs = [
            (self.balls[i], self.balls[j])
            for i in range(NUM_BALLS)
            for j in range(i + 1, NUM_BALLS)
        ]

        for ball1, ball2 in ball_pairs:
            dist_vec = ball2.position - ball1.position
            dist = np.linalg.norm(dist_vec)
            min_dist = ball1.radius + ball2.radius

            if dist < min_dist:
                # --- Collision Response ---
                # 1. Calculate Collision Normal (from ball1 to ball2)
                if dist < 1e-9:  # Avoid division by zero if balls perfectly overlap
                    collision_normal = normalize(
                        np.random.rand(2) - 0.5
                    )  # Random direction
                    dist = 1e-9
                else:
                    collision_normal = dist_vec / dist

                # 2. Separate Overlapping Objects
                overlap = min_dist - dist
                separation_vec = collision_normal * (
                    overlap * 0.5 + 1e-6
                )  # Distribute separation
                ball1.position -= separation_vec
                ball2.position += separation_vec

                # 3. Calculate Relative Velocity
                relative_velocity = ball2.velocity - ball1.velocity

                # 4. Calculate Impulse (Normal Component - Restitution)
                vel_along_normal = np.dot(relative_velocity, collision_normal)

                # Only apply impulse if objects are moving towards each other
                if vel_along_normal < 0:
                    e = COEFF_RESTITUTION
                    m1, m2 = ball1.mass, ball2.mass
                    impulse_j = -(1 + e) * vel_along_normal
                    impulse_j /= 1 / m1 + 1 / m2

                    # Apply impulse
                    impulse_vec = impulse_j * collision_normal
                    ball1.velocity -= impulse_vec / m1
                    ball2.velocity += impulse_vec / m2

                    # 5. Calculate Impulse (Tangential Component - Friction)
                    tangent = np.array([-collision_normal[1], collision_normal[0]])
                    relative_tangential_vel_mag = np.dot(relative_velocity, tangent)

                    # Surface velocities due to rotation at contact point
                    # Point relative to ball1 center is R1 * normal
                    # Point relative to ball2 center is -R2 * normal
                    # Velocity due to rotation = w x r
                    # For ball1: w1 * k x (R1 * normal) = w1 * R1 * (k x normal) = w1 * R1 * tangent (if normal is (nx, ny), tangent is (-ny, nx))
                    # For ball2: w2 * k x (-R2 * normal) = -w2 * R2 * (k x normal) = -w2 * R2 * tangent
                    # Surface vel ball1 = ball1.angular_velocity * ball1.radius * tangent (vector) -> magnitude wr
                    # Surface vel ball2 = -ball2.angular_velocity * ball2.radius * tangent (vector) -> magnitude -wr
                    # But we use scalar angular velocity w. Tangential speed at contact points:
                    surf_speed1 = (
                        -ball1.angular_velocity * ball1.radius
                    )  # Negative sign convention might depend
                    surf_speed2 = -ball2.angular_velocity * ball2.radius

                    # Sliding velocity = relative linear tangential velocity - relative angular surface velocity
                    # sliding_velocity = (v2_tang - v1_tang) - (surf_speed2 - surf_speed1) -- check this carefully
                    # Let's use a simpler model based on relative tangential velocity of centers plus surface speeds
                    # Total relative velocity component along tangent
                    sliding_velocity = (
                        relative_tangential_vel_mag + surf_speed1 - surf_speed2
                    )

                    # Calculate friction impulse magnitude (limited by normal impulse)
                    max_friction_impulse = abs(COEFF_FRICTION_BALL_BALL * impulse_j)

                    # Effective mass/inertia term for tangential impulse
                    I1, I2 = ball1.inertia, ball2.inertia
                    R1, R2 = ball1.radius, ball2.radius
                    friction_denominator = 1 / m1 + 1 / m2 + R1**2 / I1 + R2**2 / I2

                    if abs(friction_denominator) > 1e-9:
                        friction_impulse_magnitude = (
                            -sliding_velocity / friction_denominator
                        )
                    else:
                        friction_impulse_magnitude = 0.0

                    # Clamp friction impulse
                    friction_impulse_magnitude = max(
                        -max_friction_impulse,
                        min(max_friction_impulse, friction_impulse_magnitude),
                    )

                    friction_impulse_vector = friction_impulse_magnitude * tangent

                    # Apply friction impulse to linear and angular velocities
                    ball1.velocity -= friction_impulse_vector / m1
                    ball2.velocity += friction_impulse_vector / m2

                    # Torque = r x F. For ball 1, r = R1 * normal, F = -friction_impulse_vector
                    # Torque_1_mag = | R1 * normal x (-mag * tangent) | = R1 * mag * |normal x tangent| = R1 * mag
                    # Sign: Friction opposes sliding. If sliding_velocity > 0 (relative motion in tangent dir), friction acts opposite.
                    # Torque direction depends on cross product rules / sign conventions.
                    # Torque on ball 1 = r1 x F_friction = (R1*normal) x (friction_mag * tangent) -> check sign
                    # Let's use the scalar torque = +/- R * F_tangential
                    # Torque = Radius * TangentialForceComponent
                    torque1 = (
                        ball1.radius * friction_impulse_magnitude
                    )  # Check sign convention
                    torque2 = (
                        ball2.radius * friction_impulse_magnitude
                    )  # Check sign convention

                    ball1.angular_velocity += torque1 / ball1.inertia
                    ball2.angular_velocity -= (
                        torque2 / ball2.inertia
                    )  # Force on ball2 is opposite

    def update(self):
        """Update the simulation state by one time step."""
        if self.paused:
            # Need to redraw even when paused if window is resized etc.
            self.draw()
            # Schedule next check without advancing physics
            self.canvas.after(UPDATE_MS, self.update)
            return

        # 1. Update Heptagon
        self.heptagon.update(TIME_STEP)

        # 2. Update Balls (apply forces, move)
        for ball in self.balls:
            ball.update(TIME_STEP)

        # 3. Handle Collisions (detect and respond)
        # Multiple collision passes can improve stability but cost performance
        num_collision_passes = 2  # Iterate collision resolution
        for _ in range(num_collision_passes):
            self._handle_collisions(TIME_STEP / num_collision_passes)

        # 4. Draw everything
        self.draw()

        # 5. Schedule next update
        self.canvas.after(UPDATE_MS, self.update)

    def draw(self):
        """Clear canvas and draw all elements."""
        # It's often faster to move existing items than to delete and recreate
        # self.canvas.delete("all") # Avoid if possible

        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas)

    def run(self):
        """Start the simulation loop."""
        self.update()


# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")

    # Handle high-DPI displays for better scaling if needed
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)  # Windows specific
        # Adjust WIDTH, HEIGHT, BALL_RADIUS, HEPTAGON_RADIUS based on DPI if needed
    except ImportError:
        pass  # Not on Windows or ctypes not available
    except AttributeError:
        pass  # Older Windows version

    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="lightblue")
    canvas.pack()

    # Add instructions
    info_label = tk.Label(root, text="Spacebar to Pause/Resume", font=("Arial", 10))
    info_label.pack(side=tk.BOTTOM)

    # Set a higher recursion depth limit if complex collisions cause issues,
    # although ideally the simulation logic should avoid deep recursion.
    # sys.setrecursionlimit(2000)

    simulation = Simulation(canvas)

    # Ensure window closing stops the simulation loop gracefully
    def on_closing():
        print("Closing window...")
        # Stop the simulation loop by setting pause and not rescheduling
        simulation.paused = True
        # Optionally cancel the last scheduled 'after' job if possible,
        # but just pausing and letting the window close usually works.
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    simulation.run()
    root.mainloop()
