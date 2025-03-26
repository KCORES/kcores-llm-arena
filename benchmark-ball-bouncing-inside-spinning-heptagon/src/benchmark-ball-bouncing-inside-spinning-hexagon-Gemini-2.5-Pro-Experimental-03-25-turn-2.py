import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass, field
import typing
import time
import sys

# --- Constants ---
WIDTH, HEIGHT = 800, 800
NUM_BALLS = 20
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Ensure it's large enough
CENTER_X, CENTER_Y = WIDTH / 2, HEIGHT / 2
CENTER = np.array([CENTER_X, CENTER_Y])

# Physics constants
GRAVITY = np.array([0.0, 400.0])  # Pixels/s^2 (Y increases downwards)
DT = 0.016  # Time step (approx 60 FPS)
COEFF_RESTITUTION = 0.7  # Bounce factor (sqrt(ball_radius / HEPTAGON_RADIUS) < e < 1) -> sqrt(15/300) ~ 0.22. Let's use 0.7 for lively bounce.
FRICTION_COEFF = 0.3  # Tangential friction during collision
AIR_FRICTION_COEFF = 0.01  # Simple velocity damping

# Heptagon rotation
HEPTAGON_SIDES = 7
SPIN_SPEED_DPS = 360 / 5.0  # Degrees per second
SPIN_SPEED_RPS = math.radians(SPIN_SPEED_DPS)  # Radians per second

# Colors
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

# Type alias for vectors
Vector = np.ndarray


# --- Vector Helper Functions ---
def normalize(v: Vector) -> Vector:
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def rotate_vector(v: Vector, angle: float) -> Vector:
    """Rotates a 2D vector by a given angle in radians."""
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return np.array([v[0] * cos_a - v[1] * sin_a, v[0] * sin_a + v[1] * cos_a])


# --- Ball Class ---
@dataclass
class Ball:
    id: int
    pos: Vector
    vel: Vector = field(default_factory=lambda: np.zeros(2, dtype=float))
    acc: Vector = field(default_factory=lambda: np.zeros(2, dtype=float))
    radius: float = BALL_RADIUS
    mass: float = 1.0  # Assume uniform mass
    inv_mass: float = field(init=False)
    moment_of_inertia: float = field(init=False)  # For solid sphere: 2/5 * m * r^2
    inv_moment_of_inertia: float = field(init=False)
    angle: float = 0.0  # Radians
    angular_vel: float = 0.0  # Radians per second
    color: str = "blue"
    restitution: float = COEFF_RESTITUTION
    friction: float = FRICTION_COEFF
    tk_id_shape: typing.Optional[int] = None
    tk_id_text: typing.Optional[int] = None

    def __post_init__(self):
        self.inv_mass = 1.0 / self.mass if self.mass > 0 else 0
        self.moment_of_inertia = (
            (2 / 5) * self.mass * self.radius**2 if self.mass > 0 else 0
        )
        self.inv_moment_of_inertia = (
            1.0 / self.moment_of_inertia if self.moment_of_inertia > 0 else 0
        )
        # Small initial offset to prevent perfect overlap at center
        self.pos += np.random.rand(2) * 0.1

    def apply_force(self, force: Vector):
        self.acc += force * self.inv_mass

    def apply_impulse(self, impulse: Vector, contact_point: Vector):
        self.vel += impulse * self.inv_mass
        # Torque = r x F (cross product). In 2D: r.x * F.y - r.y * F.x
        contact_vec = contact_point - self.pos
        torque = contact_vec[0] * impulse[1] - contact_vec[1] * impulse[0]
        self.angular_vel += torque * self.inv_moment_of_inertia

    def update(self, dt: float):
        # Apply gravity and air friction
        self.apply_force(GRAVITY * self.mass)
        self.apply_force(-self.vel * AIR_FRICTION_COEFF)  # Simple air damping

        # Update velocity and position (Euler integration)
        self.vel += self.acc * dt
        self.pos += self.vel * dt

        # Update angle
        self.angle += self.angular_vel * dt
        self.angle %= 2 * math.pi  # Keep angle in [0, 2pi)

        # Reset acceleration
        self.acc = np.zeros(2)

    def draw(self, canvas: tk.Canvas):
        x0, y0 = self.pos - self.radius
        x1, y1 = self.pos + self.radius

        if self.tk_id_shape:
            canvas.coords(self.tk_id_shape, x0, y0, x1, y1)
            canvas.itemconfig(self.tk_id_shape, fill=self.color)
        else:
            self.tk_id_shape = canvas.create_oval(
                x0, y0, x1, y1, fill=self.color, outline="black"
            )

        text_angle_deg = -math.degrees(self.angle)  # Tkinter uses degrees, clockwise
        if self.tk_id_text:
            canvas.coords(self.tk_id_text, self.pos[0], self.pos[1])
            # Update angle directly using itemconfig
            canvas.itemconfig(self.tk_id_text, angle=text_angle_deg)
        else:
            self.tk_id_text = canvas.create_text(
                self.pos[0],
                self.pos[1],
                text=str(self.id),
                angle=text_angle_deg,  # Tkinter supports text rotation!
                font=("Arial", int(self.radius * 0.8), "bold"),
                fill="black",
            )


# --- Heptagon Class ---
@dataclass
class Heptagon:
    center: Vector
    radius: float
    num_sides: int = HEPTAGON_SIDES
    angle: float = 0.0  # Current rotation angle in radians
    spin_speed: float = SPIN_SPEED_RPS  # Radians per second
    vertices: typing.List[Vector] = field(default_factory=list)
    edges: typing.List[typing.Tuple[Vector, Vector]] = field(default_factory=list)
    tk_id: typing.Optional[int] = None

    def __post_init__(self):
        self.update_geometry()

    def update_geometry(self):
        self.vertices = []
        angle_step = 2 * math.pi / self.num_sides
        for i in range(self.num_sides):
            vertex_angle = self.angle + i * angle_step
            # Start from top vertex for regularity if desired
            # vertex_angle = self.angle + i * angle_step - math.pi / 2
            x = self.center[0] + self.radius * math.cos(vertex_angle)
            y = self.center[1] + self.radius * math.sin(vertex_angle)
            self.vertices.append(np.array([x, y]))

        self.edges = []
        for i in range(self.num_sides):
            p1 = self.vertices[i]
            p2 = self.vertices[(i + 1) % self.num_sides]
            self.edges.append((p1, p2))

    def update(self, dt: float):
        self.angle += self.spin_speed * dt
        self.angle %= 2 * math.pi
        self.update_geometry()

    def draw(self, canvas: tk.Canvas):
        coords = [coord for vertex in self.vertices for coord in vertex]
        if self.tk_id:
            canvas.coords(self.tk_id, *coords)
        else:
            self.tk_id = canvas.create_polygon(
                *coords, fill="", outline="white", width=2
            )

    def get_wall_velocity(self, point_on_wall: Vector) -> Vector:
        """Calculates the velocity of a point on the spinning wall."""
        relative_pos = point_on_wall - self.center
        # Velocity (v) = angular_velocity (omega) x radius (r)
        # In 2D: v_x = -omega * r_y, v_y = omega * r_x
        omega = self.spin_speed
        vel_x = -omega * relative_pos[1]
        vel_y = omega * relative_pos[0]
        # Need to adjust for sign convention? Let's test.
        # If spin_speed is positive (CCW), and r = (R, 0), vel should be (0, omega*R)
        # vel_x = -omega * 0 = 0
        # vel_y = omega * R --> Correct.
        # If r = (0, R), vel should be (-omega*R, 0)
        # vel_x = -omega * R --> Correct
        # vel_y = omega * 0 = 0
        return np.array([vel_x, vel_y])


# --- Collision Detection and Response ---


def collide_ball_wall(ball: Ball, heptagon: Heptagon):
    """Detects and resolves collisions between a ball and the heptagon walls."""
    min_overlap = float("inf")
    collision_edge = None
    collision_normal = None
    closest_point_on_edge = None

    for p1, p2 in heptagon.edges:
        edge_vec = p2 - p1
        edge_len_sq = np.dot(edge_vec, edge_vec)
        if edge_len_sq == 0:
            continue  # Should not happen for heptagon

        # Vector from p1 to ball center
        ball_vec = ball.pos - p1

        # Project ball_vec onto edge_vec (find parameter t)
        t = np.dot(ball_vec, edge_vec) / edge_len_sq

        # Find closest point on the infinite line containing the edge
        if t < 0:
            closest_point = p1
        elif t > 1:
            closest_point = p2
        else:
            closest_point = p1 + t * edge_vec

        # Distance from ball center to closest point
        dist_vec = ball.pos - closest_point
        dist_sq = np.dot(dist_vec, dist_vec)

        if dist_sq < ball.radius**2:
            dist = math.sqrt(dist_sq)
            overlap = ball.radius - dist
            if (
                overlap > 0
            ):  # Ensure it's a real overlap, avoid floating point issues near zero
                # Use edge normal pointing inwards
                edge_normal = np.array([edge_vec[1], -edge_vec[0]])  # Rotate 90 deg CCW
                edge_normal = normalize(edge_normal)

                # Ensure normal points towards the heptagon center (roughly)
                center_to_p1 = p1 - heptagon.center
                if np.dot(edge_normal, center_to_p1) > 0:
                    edge_normal = -edge_normal  # Flip if pointing outwards

                # Prioritize the collision with the largest overlap in case of corner hits
                # This isn't perfect for corners but simpler than vertex checks
                if (
                    overlap > min_overlap
                ):  # Use > because we want the *shallowest* penetration to resolve first typically. Or maybe largest? Let's try largest.
                    # Actually, maybe need smallest dist? Let's stick with largest overlap for now.
                    min_overlap = overlap
                    collision_edge = (p1, p2)
                    collision_normal = edge_normal
                    closest_point_on_edge = closest_point

    if collision_normal is not None:
        # --- Positional Correction ---
        # Move ball out of penetration along the normal
        correction = collision_normal * min_overlap
        ball.pos += correction

        # --- Collision Response ---
        wall_vel_at_contact = heptagon.get_wall_velocity(closest_point_on_edge)
        relative_vel = ball.vel - wall_vel_at_contact

        # Velocity component along the normal
        vel_along_normal = np.dot(relative_vel, collision_normal)

        # Only apply impulse if moving towards the wall
        if vel_along_normal < 0:
            # Calculate restitution impulse (normal direction)
            e = ball.restitution
            j_n = -(1 + e) * vel_along_normal * ball.mass  # Wall mass assumed infinite
            impulse_n = j_n * collision_normal

            # Calculate friction impulse (tangent direction)
            tangent = np.array(
                [-collision_normal[1], collision_normal[0]]
            )  # Perpendicular to normal
            vel_along_tangent = np.dot(relative_vel, tangent)

            # Relative tangential velocity at contact point (includes spin)
            # Ball surface velocity = -angular_vel * radius (sign depends on convention)
            # Assume positive angular_vel is CCW. Tangent is 90deg CCW from normal.
            # Contact point relative to ball center = -radius * collision_normal
            # Surface vel = angular_vel (k) x (-radius * normal) = -angular_vel * radius * (k x normal)
            # k x normal = k x (nx i + ny j) = nx (k x i) + ny (k x j) = nx (j) + ny (-i) = -ny i + nx j
            # Surface vel = -angular_vel * radius * (-normal.y i + normal.x j)
            # Surface vel = angular_vel * radius * (normal.y i - normal.x j)
            # The tangential component of this surface velocity is its projection onto the tangent vector
            # tangent = (-normal.y i + normal.x j)
            # Surface vel dot tangent = angular_vel * radius * (normal.y * -normal.y + (-normal.x * normal.x))
            # = -angular_vel * radius * (normal.y^2 + normal.x^2) = -angular_vel * radius
            # relative_tangent_vel = vel_along_tangent + ball_surface_vel_tangent
            relative_tangent_vel = (
                vel_along_tangent - ball.angular_vel * ball.radius
            )  # Check sign convention carefully! If angular_vel is positive (CCW), it opposes positive tangent velocity?

            # Max friction impulse based on Coulomb friction (mu * normal_impulse)
            max_friction_impulse = abs(j_n * ball.friction)

            # Impulse needed to stop relative tangential motion (simplified)
            # Note: More accurate would involve moments of inertia etc.
            # Let's use simplified impulse proportional to relative tangent velocity
            j_t = -relative_tangent_vel * ball.mass  # Impulse needed to stop sliding
            j_t = max(
                -max_friction_impulse, min(j_t, max_friction_impulse)
            )  # Clamp friction

            impulse_t = j_t * tangent

            # Apply combined impulse
            total_impulse = impulse_n + impulse_t
            contact_point_relative = closest_point_on_edge  # Contact point on ball surface approx closest point

            ball.apply_impulse(total_impulse, contact_point_relative)


def collide_ball_ball(ball1: Ball, ball2: Ball):
    """Detects and resolves collisions between two balls."""
    delta_pos = ball2.pos - ball1.pos
    dist_sq = np.dot(delta_pos, delta_pos)
    min_dist = ball1.radius + ball2.radius

    if (
        dist_sq < min_dist**2 and dist_sq > 1e-6
    ):  # Avoid collision with self or zero distance
        dist = math.sqrt(dist_sq)
        overlap = min_dist - dist
        normal = delta_pos / dist  # Normal from ball1 to ball2

        # --- Positional Correction ---
        # Separate balls along the normal, weighted by inverse mass
        total_inv_mass = ball1.inv_mass + ball2.inv_mass
        if total_inv_mass > 0:
            correction = normal * (overlap / total_inv_mass)
            ball1.pos -= correction * ball1.inv_mass
            ball2.pos += correction * ball2.inv_mass

        # --- Collision Response ---
        relative_vel = ball1.vel - ball2.vel
        vel_along_normal = np.dot(relative_vel, normal)

        # Only apply impulse if moving towards each other
        if vel_along_normal < 0:
            e = min(ball1.restitution, ball2.restitution)  # Use minimum restitution

            # Calculate normal impulse magnitude
            j_n = -(1 + e) * vel_along_normal / total_inv_mass
            impulse_n = j_n * normal

            # Calculate friction impulse (tangential) - simplified version
            # Effective mass for tangential collision can differ, but let's reuse total_inv_mass
            tangent = np.array([-normal[1], normal[0]])
            relative_vel_tangent = np.dot(relative_vel, tangent)

            # Relative surface velocity due to spin
            surface_vel1 = (
                -ball1.angular_vel * ball1.radius * tangent
            )  # Vel of ball1 surface at contact point
            surface_vel2 = (
                -ball2.angular_vel * ball2.radius * tangent
            )  # Vel of ball2 surface at contact point
            # relative_surface_vel_tangent = relative_vel_tangent + (surface_vel1 - surface_vel2) projected onto tangent.
            # Since surface_vels are already tangential:
            relative_surface_vel_tangent = (
                relative_vel_tangent
                - ball1.angular_vel * ball1.radius
                + ball2.angular_vel * ball2.radius
            )  # Sign check needed!

            mu = (ball1.friction + ball2.friction) / 2  # Average friction

            # Impulse needed to stop relative tangential motion
            j_t = (
                -relative_surface_vel_tangent / total_inv_mass
            )  # Simplified effective mass
            max_friction_impulse = abs(j_n * mu)
            j_t = max(-max_friction_impulse, min(j_t, max_friction_impulse))  # Clamp

            impulse_t = j_t * tangent

            # Apply impulses
            total_impulse = impulse_n + impulse_t

            # Contact points relative to centers
            contact1 = ball1.pos + normal * ball1.radius
            contact2 = ball2.pos - normal * ball2.radius  # Normal points from 1 to 2

            ball1.apply_impulse(-total_impulse, contact1)
            ball2.apply_impulse(total_impulse, contact2)


# --- Simulation Class ---
class Simulation:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="#202030")
        self.canvas.pack()

        self.heptagon = Heptagon(center=CENTER, radius=HEPTAGON_RADIUS)
        self.balls = self._create_balls()

        self.last_time = time.monotonic()
        self.running = True

        # Bind escape key to close
        self.master.bind("<Escape>", self.quit)

        self.master.after(10, self.step)  # Start the loop

    def _create_balls(self) -> typing.List[Ball]:
        balls = []
        for i in range(NUM_BALLS):
            ball = Ball(
                id=i + 1,
                pos=CENTER.copy(),  # Start at center
                color=COLORS[i % len(COLORS)],
            )
            balls.append(ball)
        return balls

    def step(self):
        if not self.running:
            return

        current_time = time.monotonic()
        # Use a fixed timestep for stability, accumulating real time
        # Or just use actual elapsed time (simpler but less stable?)
        # dt = current_time - self.last_time
        dt = DT  # Using fixed timestep
        self.last_time = current_time

        # 1. Update Heptagon
        self.heptagon.update(dt)

        # 2. Update Balls (Physics)
        for ball in self.balls:
            ball.update(dt)

        # 3. Handle Collisions (multiple passes can improve stability)
        num_passes = 3  # Iteratively resolve collisions/overlaps
        for _ in range(num_passes):
            # Ball-Wall Collisions
            for ball in self.balls:
                collide_ball_wall(ball, self.heptagon)

            # Ball-Ball Collisions
            for i in range(NUM_BALLS):
                for j in range(i + 1, NUM_BALLS):
                    collide_ball_ball(self.balls[i], self.balls[j])

        # 4. Draw everything
        self.canvas.delete("all")  # Clear previous frame
        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas)  # Draw balls on top

        # 5. Schedule next step
        # Calculate time taken and adjust next call time if using fixed DT
        # elapsed = time.monotonic() - current_time
        # wait_time = max(1, int((DT - elapsed) * 1000)) # Wait at least 1ms
        wait_time = int(DT * 1000)
        self.master.after(wait_time, self.step)

    def quit(self, event=None):
        self.running = False
        self.master.quit()


# --- Main Execution ---
if __name__ == "__main__":
    # Check if numpy is installed
    if "numpy" not in sys.modules:
        print("Error: NumPy library is required but not found.")
        print("Please install it using: pip install numpy")
        sys.exit(1)

    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    # Set window position (optional)
    # root.geometry(f"{WIDTH}x{HEIGHT}+100+50")
    app = Simulation(root)
    root.mainloop()
