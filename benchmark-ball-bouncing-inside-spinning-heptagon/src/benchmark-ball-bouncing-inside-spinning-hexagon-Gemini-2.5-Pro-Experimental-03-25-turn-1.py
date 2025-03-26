import tkinter as tk
import math
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple

# --- Constants ---
WIDTH, HEIGHT = 800, 800
BALL_COUNT = 20
BALL_RADIUS = 15
HEPTAGON_RADIUS = 350  # Ensure it's large enough
COLORS = [
    # "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51",
    # "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800", # Duplicate colors removed for clarity
    # "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61",
    # "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ee7800",
    "#eb6238",  # First 10 unique
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
    "#db8449",
    "#d66a35",
    "#c95c2f",
    "#c24e29",  # Added 2 more unique ones for 20
] * (
    BALL_COUNT // 20 + 1
)  # Ensure enough colors if BALL_COUNT > 20

GRAVITY = np.array([0.0, 350.0])  # Pixels/s^2 (positive y is down)
TIME_STEP = 1 / 60  # Simulation time step in seconds
HEPTAGON_SPIN_SPEED = 360 / 5  # Degrees per second
HEPTAGON_SPIN_RAD_PER_SEC = math.radians(HEPTAGON_SPIN_SPEED)

# Physics properties
COEFF_RESTITUTION = 0.7  # Bounciness (0=inelastic, 1=perfectly elastic)
# Bounce height constraint: r_ball < bounce_height < R_heptagon
# This implies e should be such that e^2 * h is within this range.
# For a drop from center to edge (~R_heptagon), bounce height = e^2 * R_heptagon.
# We need BALL_RADIUS < e^2 * R_heptagon < R_heptagon.
# e.g. 15 < e^2 * 350 < 350 => 0.04 < e^2 < 1 => 0.2 < e < 1. Our 0.7 is fine.
FRICTION_COEFF_BALL_WALL = 0.3  # Tangential friction coefficient
FRICTION_COEFF_BALL_BALL = 0.2
AIR_RESISTANCE_FACTOR = 0.005  # Simple linear drag F = -kv

# --- Vector Math ---
# Using numpy arrays for vectors


# --- Data Structures ---
@dataclass
class Ball:
    id: int
    pos: np.ndarray
    vel: np.ndarray
    radius: float
    mass: float
    color: str
    angle: float = 0.0  # Rotation angle in radians
    angular_vel: float = 0.0  # Radians per second
    moment_of_inertia: float = 0.0  # Will be calculated
    tk_id_ball: int = -1  # Canvas item ID for the oval
    tk_id_text: int = -1  # Canvas item ID for the number

    def __post_init__(self):
        # Moment of inertia for a solid sphere: (2/5) * m * r^2
        # Since we are in 2D, let's approximate with a disk: (1/2) * m * r^2
        # Or just use sphere's I, as it relates torque to angular accel
        self.moment_of_inertia = 0.4 * self.mass * self.radius**2  # (2/5)mr^2


# --- Simulation Class ---
class BouncingBallsSim:
    def __init__(self, master):
        self.master = master
        self.master.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.heptagon_center = np.array([WIDTH / 2, HEIGHT / 2])
        self.heptagon_radius = HEPTAGON_RADIUS
        self.heptagon_angle = 0.0  # Current angle in radians
        self.tk_id_heptagon = -1

        self.balls: List[Ball] = []
        self._create_balls()
        self._create_heptagon()  # Draw initial heptagon

        self.last_time = time.perf_counter()
        self.running = True
        self.master.protocol("WM_DELETE_WINDOW", self.on_close)
        self.update()

    def _create_balls(self):
        for i in range(BALL_COUNT):
            # Start slightly offset to avoid immediate perfect overlap issues
            offset = np.random.rand(2) * 0.1 - 0.05
            start_pos = self.heptagon_center + offset

            ball = Ball(
                id=i + 1,
                pos=start_pos.copy(),
                vel=np.zeros(2, dtype=float),
                radius=BALL_RADIUS,
                mass=math.pi
                * BALL_RADIUS**2,  # Mass proportional to area (constant density)
                color=COLORS[i % len(COLORS)],
            )
            # Create canvas items (oval and text)
            x0, y0 = ball.pos - ball.radius
            x1, y1 = ball.pos + ball.radius
            ball.tk_id_ball = self.canvas.create_oval(
                x0, y0, x1, y1, fill=ball.color, outline="white", width=1
            )
            ball.tk_id_text = self.canvas.create_text(
                ball.pos[0],
                ball.pos[1],
                text=str(ball.id),
                fill="black",
                font=("Arial", int(ball.radius * 0.8), "bold"),
            )
            self.balls.append(ball)

    def _get_heptagon_vertices(self, angle_rad):
        vertices = []
        num_sides = 7
        for i in range(num_sides):
            theta = angle_rad + (2 * math.pi * i / num_sides)
            x = self.heptagon_center[0] + self.heptagon_radius * math.cos(theta)
            y = self.heptagon_center[1] + self.heptagon_radius * math.sin(theta)
            vertices.append(np.array([x, y]))
        return vertices

    def _create_heptagon(self):
        vertices = self._get_heptagon_vertices(self.heptagon_angle)
        flat_vertices = [coord for v in vertices for coord in v]
        self.tk_id_heptagon = self.canvas.create_polygon(
            *flat_vertices, outline="cyan", fill="", width=2  # No fill
        )

    def _update_heptagon_drawing(self):
        vertices = self._get_heptagon_vertices(self.heptagon_angle)
        flat_vertices = [coord for v in vertices for coord in v]
        self.canvas.coords(self.tk_id_heptagon, *flat_vertices)

    def on_close(self):
        self.running = False
        self.master.destroy()

    def update(self):
        if not self.running:
            return

        current_time = time.perf_counter()
        # dt = current_time - self.last_time # Variable timestep (can be unstable)
        dt = TIME_STEP  # Fixed timestep
        self.last_time = current_time

        # Update heptagon rotation
        self.heptagon_angle += HEPTAGON_SPIN_RAD_PER_SEC * dt
        self.heptagon_angle %= 2 * math.pi  # Keep angle within [0, 2pi)

        # Physics Update
        self.apply_forces(dt)
        self.integrate(dt)
        self.handle_collisions(dt)

        # Drawing Update
        self.draw()

        # Schedule next update
        self.master.after(int(TIME_STEP * 1000), self.update)

    def apply_forces(self, dt):
        for ball in self.balls:
            # Gravity
            force = GRAVITY * ball.mass

            # Air resistance (simple linear drag)
            force -= AIR_RESISTANCE_FACTOR * ball.vel

            # Update velocity based on force F = ma => a = F/m => dv = a*dt = (F/m)*dt
            ball.vel += (force / ball.mass) * dt

            # Update angular velocity (damping only for now, friction adds more later)
            # Simple angular damping
            ball.angular_vel *= 1.0 - 0.5 * dt

    def integrate(self, dt):
        for ball in self.balls:
            ball.pos += ball.vel * dt
            ball.angle += ball.angular_vel * dt
            ball.angle %= 2 * math.pi

    def handle_collisions(self, dt):
        # Ball-Wall Collisions
        vertices = self._get_heptagon_vertices(self.heptagon_angle)
        num_vertices = len(vertices)
        omega = HEPTAGON_SPIN_RAD_PER_SEC

        for i in range(num_vertices):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % num_vertices]
            wall_vec = p2 - p1
            wall_len = np.linalg.norm(wall_vec)
            if wall_len < 1e-6:
                continue  # Avoid division by zero
            wall_normal = (
                np.array([-wall_vec[1], wall_vec[0]]) / wall_len
            )  # Outward normal

            for ball in self.balls:
                # Vector from wall start (p1) to ball center
                vec_p1_ball = ball.pos - p1

                # Project ball center onto the wall line
                proj_dist = np.dot(vec_p1_ball, wall_vec) / wall_len
                proj_dist_clamped = np.clip(proj_dist, 0, wall_len)

                # Closest point on the infinite line containing the wall segment
                # closest_point_line = p1 + (wall_vec / wall_len) * np.dot(vec_p1_ball, wall_vec / wall_len)
                # More direct: distance from line = |(Ax + By + C)| / sqrt(A^2 + B^2)
                # Normal form: wall_normal.x * x + wall_normal.y * y - np.dot(wall_normal, p1) = 0
                dist_from_line = np.dot(ball.pos - p1, wall_normal)

                # Closest point on the wall *segment*
                if proj_dist_clamped == 0:  # Closest to p1
                    closest_point_seg = p1
                    dist_to_closest = np.linalg.norm(ball.pos - p1)
                    collision_normal = (
                        (ball.pos - p1) / dist_to_closest
                        if dist_to_closest > 1e-6
                        else wall_normal
                    )  # Treat corner like edge for simplicity
                elif proj_dist_clamped == wall_len:  # Closest to p2
                    closest_point_seg = p2
                    dist_to_closest = np.linalg.norm(ball.pos - p2)
                    collision_normal = (
                        (ball.pos - p2) / dist_to_closest
                        if dist_to_closest > 1e-6
                        else wall_normal
                    )  # Treat corner like edge
                else:  # Closest point is on the segment interior
                    closest_point_seg = p1 + (wall_vec / wall_len) * proj_dist_clamped
                    dist_to_closest = abs(
                        dist_from_line
                    )  # Distance is perpendicular distance
                    collision_normal = wall_normal

                # --- Collision Detection ---
                overlap = ball.radius - dist_to_closest

                if overlap >= -0.1:  # Collision detected (allow tiny overlap tolerance)
                    # --- Positional Correction ---
                    # Move ball slightly away along the normal to prevent sinking
                    correction_amount = max(
                        0, overlap + 0.1
                    )  # Ensure positive correction
                    ball.pos += collision_normal * correction_amount

                    # --- Collision Response ---
                    # Calculate relative velocity between ball and wall point
                    # Wall velocity at contact point: v_wall = omega x r_contact (cross product)
                    # r_contact = closest_point_seg - self.heptagon_center
                    r_contact = closest_point_seg - self.heptagon_center
                    # In 2D: omega = (0, 0, omega_z), r = (rx, ry, 0)
                    # v_wall = (-omega_z * ry, omega_z * rx, 0)
                    v_wall = np.array([-omega * r_contact[1], omega * r_contact[0]])

                    v_rel = ball.vel - v_wall
                    v_rel_normal_comp = np.dot(v_rel, collision_normal)

                    # Only process collision if ball is moving towards the wall
                    if v_rel_normal_comp < 0:
                        # --- Normal Impulse (Bounce) ---
                        e = COEFF_RESTITUTION
                        j_normal = -(1 + e) * v_rel_normal_comp * ball.mass
                        impulse_normal = j_normal * collision_normal
                        ball.vel += impulse_normal / ball.mass

                        # --- Frictional Impulse (Spin and Slowdown) ---
                        v_rel_tangent = v_rel - v_rel_normal_comp * collision_normal
                        tangent_vec = v_rel_tangent / (
                            np.linalg.norm(v_rel_tangent) + 1e-9
                        )  # Avoid div by zero

                        # Relative velocity at contact point including ball's spin
                        # v_surface_ball = ball.angular_vel * ball.radius * (-tangent_vec[1], tangent_vec[0]) # Incorrect sign/direction?
                        # v_contact_relative = v_rel_tangent - omega_cross_r (where r is radius vector)
                        # Simplified: Use center's relative tangent velocity
                        v_rel_tangent_speed = np.linalg.norm(v_rel_tangent)

                        # Max friction impulse magnitude (Coulomb friction)
                        j_friction_max = FRICTION_COEFF_BALL_WALL * abs(j_normal)

                        # Friction impulse magnitude (tries to stop relative tangent motion)
                        # Impulse needed to stop relative tangent motion: mass * v_rel_tangent_speed
                        # We also need to consider moment of inertia for rotation change.
                        # Simplified approach: Impulse proportional to relative speed, capped by max friction.
                        # Let's use simpler impulse: j_friction = min(mass * v_rel_tangent_speed, j_friction_max) # Not quite right for rotation
                        # Alternative: Apply impulse that opposes relative tangential motion
                        j_friction = -min(
                            ball.mass * v_rel_tangent_speed * 0.8, j_friction_max
                        )  # Damp tangential motion, factor < 1 for stability

                        impulse_friction = j_friction * tangent_vec
                        ball.vel += impulse_friction / ball.mass

                        # --- Torque from Friction ---
                        # Torque = r x F_friction = radius * (-collision_normal) x impulse_friction
                        # Torque_mag = ball.radius * ||impulse_friction|| * sin(angle between -normal and tangent) = ball.radius * ||impulse_friction||
                        # Torque direction is perpendicular to plane (z-axis)
                        # Torque = r cross F => In 2D: R * F_tangential
                        # Here impulse acts tangentially, so torque mag is R * j_friction_mag
                        torque = ball.radius * (
                            -j_friction
                        )  # Positive j_friction opposes motion, positive torque spins counter-clockwise if tangent is left relative to normal?
                        # Let's align torque with friction impulse direction relative to radius
                        # Torque = r x F. r points from center to contact (-R*normal). F is along tangent.
                        # Torque_z = (-R*normal_x)*F_y - (-R*normal_y)*F_x
                        # Torque_z = -R * (normal_x * F_y - normal_y * F_x) = -R * (normal x F)_z
                        # If F = j_friction * tangent, Torque_z = -R * j_friction * (normal x tangent)_z
                        # If tangent is 90 deg CCW from normal, (normal x tangent)_z = 1. Torque = -R * j_friction
                        delta_angular_vel = torque / ball.moment_of_inertia
                        ball.angular_vel += delta_angular_vel

        # Ball-Ball Collisions
        for i in range(BALL_COUNT):
            for j in range(i + 1, BALL_COUNT):
                ball_a = self.balls[i]
                ball_b = self.balls[j]

                vec_ab = ball_b.pos - ball_a.pos
                dist_sq = np.dot(vec_ab, vec_ab)
                min_dist = ball_a.radius + ball_b.radius

                if (
                    dist_sq < min_dist**2 and dist_sq > 1e-9
                ):  # Collision detected and not exactly same point
                    dist = math.sqrt(dist_sq)
                    collision_normal = vec_ab / dist  # Normal from A to B
                    overlap = min_dist - dist

                    # --- Positional Correction ---
                    # Move balls apart along the normal, proportionally to mass (or just 50/50 if masses equal)
                    total_mass = ball_a.mass + ball_b.mass
                    correction = (
                        overlap * 0.6
                    )  # Slightly more than needed to avoid sticking
                    ball_a.pos -= (
                        collision_normal * correction * (ball_b.mass / total_mass)
                    )
                    ball_b.pos += (
                        collision_normal * correction * (ball_a.mass / total_mass)
                    )

                    # --- Collision Response ---
                    # Relative velocity
                    v_rel = ball_b.vel - ball_a.vel
                    v_rel_normal_comp = np.dot(v_rel, collision_normal)

                    # Only process if balls are moving towards each other
                    if v_rel_normal_comp < 0:
                        e = COEFF_RESTITUTION
                        inv_mass_sum = 1.0 / ball_a.mass + 1.0 / ball_b.mass

                        # --- Normal Impulse ---
                        j_normal = -(1 + e) * v_rel_normal_comp / inv_mass_sum
                        impulse_normal = j_normal * collision_normal

                        ball_a.vel -= impulse_normal / ball_a.mass
                        ball_b.vel += impulse_normal / ball_b.mass

                        # --- Frictional Impulse (Simplified) ---
                        # Calculate relative velocity tangent to the collision point
                        v_rel_tangent = v_rel - v_rel_normal_comp * collision_normal
                        tangent_vec = v_rel_tangent / (
                            np.linalg.norm(v_rel_tangent) + 1e-9
                        )

                        # Relative surface speed (simplified: only consider center vels + spin)
                        # v_rel_surf_a = -ball_a.angular_vel * ball_a.radius # speed at contact point relative to center A
                        # v_rel_surf_b = -ball_b.angular_vel * ball_b.radius # speed at contact point relative to center B
                        # Total relative tangent speed includes these spins, it's complex.
                        # Simplified: Use relative velocity of centers tangentially
                        v_rel_tangent_speed = np.linalg.norm(v_rel_tangent)

                        # Max friction impulse
                        j_friction_max = FRICTION_COEFF_BALL_BALL * abs(j_normal)

                        # Simplified friction impulse magnitude (damps relative tangent motion)
                        # Impulse needed to stop tangential motion (centers): ~ v_rel_tangent_speed / inv_mass_sum
                        j_friction_needed = (
                            v_rel_tangent_speed / inv_mass_sum
                        )  # Approx impulse needed
                        j_friction = -min(
                            j_friction_needed, j_friction_max
                        )  # Actual friction impulse magnitude (negative: opposes motion)

                        impulse_friction = j_friction * tangent_vec

                        # Apply linear friction impulse
                        ball_a.vel -= impulse_friction / ball_a.mass
                        ball_b.vel += impulse_friction / ball_b.mass

                        # Apply torque due to friction
                        # Torque on A = r_A x (-impulse_friction) = (-R * collision_normal) x (-impulse_friction)
                        # Torque on B = r_B x (+impulse_friction) = (+R * collision_normal) x (+impulse_friction)
                        # Torque magnitude for both is R * |j_friction|
                        torque_mag = ball_a.radius * abs(j_friction)

                        # Determine direction using cross product logic (normal x tangent)
                        # For ball A, force is -impulse_friction (-j_friction * tangent)
                        # Torque_A_z = R * (-j_friction) * (normal x tangent)_z
                        # For ball B, force is +impulse_friction (+j_friction * tangent)
                        # Torque_B_z = -R * (+j_friction) * (normal x tangent)_z
                        # Assume tangent is 90 deg CCW from normal => (normal x tangent)_z = +1
                        torque_sign = np.sign(j_friction)  # Friction opposes motion

                        delta_angular_vel_A = (
                            torque_sign * torque_mag / ball_a.moment_of_inertia
                        )
                        delta_angular_vel_B = (
                            torque_sign * torque_mag / ball_b.moment_of_inertia
                        )  # Should be opposite? Torque = r x F. r_B = R*normal. F_B = impulse_friction. Torque_B = R*(normal x F_B)_z

                        # Let's rethink torque direction simply:
                        # Friction opposes relative tangential motion at contact point.
                        # If ball B surface moves faster tangentially than A's surface, friction slows B, speeds up A.
                        # This induces torque. If friction force on A is in direction T, torque is R x T.
                        # If friction force on B is -T, torque is R x (-T).
                        # Let's use the simplified sign from wall collision: torque = R * (-j_friction) applied to both? Needs care.

                        # Simpler: Friction causes torque opposing the spin difference relative to linear motion.
                        # Let's stick to the impulse j_friction opposing v_rel_tangent.
                        # Torque on A: radius_vec_A x (-impulse_friction) => (-R*normal) x (-j_friction*tangent) = R*j_friction*(normal x tangent)_z
                        # Torque on B: radius_vec_B x (+impulse_friction) => (+R*normal) x ( j_friction*tangent) = R*j_friction*(normal x tangent)_z
                        # If (normal x tangent)_z = 1, delta_omega = R * j_friction / I
                        # j_friction is negative, so this causes negative delta_omega (CW spin). Is this right?

                        # Let's use the magnitude and decide sign based on relative spin vs linear motion. Too complex for now.
                        # Use the wall-collision torque logic for simplicity, applied to both:
                        torque = ball_a.radius * (
                            -j_friction
                        )  # Same magnitude, direction needs checking
                        # This torque might need opposite signs for the two balls.
                        # If friction force on A is F, torque on A = rA x F. If force on B is -F, torque on B = rB x (-F).
                        # rA = -R*normal, rB = R*normal.
                        # T_A = (-R*normal) x F
                        # T_B = (R*normal) x (-F) = - R*(normal x F) = T_A
                        # So the torques should be the same! (If F is the force ON A). Let F = impulse_friction on B
                        # Force ON A is -impulse_friction. Torque on A = (-R*normal) x (-impulse_friction) = R * (normal x impulse_friction)_z
                        # Force ON B is +impulse_friction. Torque on B = (R*normal) x (+impulse_friction) = R * (normal x impulse_friction)_z
                        # Okay, torques are the same direction.
                        delta_angular_vel = (
                            torque / ball_a.moment_of_inertia
                        )  # Use ball A's radius/MoI - assumes same radius
                        ball_a.angular_vel += delta_angular_vel
                        ball_b.angular_vel += delta_angular_vel * (
                            ball_a.moment_of_inertia / ball_b.moment_of_inertia
                            if ball_b.moment_of_inertia > 1e-9
                            else 0
                        )  # scale by MoI ratio

    def draw(self):
        # Update heptagon
        self._update_heptagon_drawing()

        # Update balls
        for ball in self.balls:
            x0 = ball.pos[0] - ball.radius
            y0 = ball.pos[1] - ball.radius
            # Move ball oval
            self.canvas.coords(
                ball.tk_id_ball, x0, y0, x0 + 2 * ball.radius, y0 + 2 * ball.radius
            )

            # Move and rotate text
            self.canvas.coords(ball.tk_id_text, ball.pos[0], ball.pos[1])
            # Convert radians to degrees for tkinter text angle (positive is counter-clockwise)
            tk_angle_deg = -math.degrees(ball.angle)
            self.canvas.itemconfig(ball.tk_id_text, angle=tk_angle_deg)

        # Bring text items to front (optional, but helps visibility)
        for ball in self.balls:
            self.canvas.tag_raise(ball.tk_id_text)

        self.canvas.update_idletasks()  # Process pending tk events


# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    sim = BouncingBallsSim(root)
    root.mainloop()
