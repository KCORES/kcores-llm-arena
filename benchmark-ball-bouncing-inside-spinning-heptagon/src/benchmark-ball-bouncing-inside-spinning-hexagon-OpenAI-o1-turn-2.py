import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constant definitions (English)
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

# Time step for the animation (seconds)
DT = 0.02

# Gravity (pixels / s^2) - tuned for visual effect
GRAVITY = 30.0

# Velocity damping factor each frame (friction-like)
VELOCITY_DAMPING = 0.99

# Coefficient of restitution for collisions (with walls/balls)
ELASTICITY = 0.9

# Heptagon settings
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 300  # large enough for 20 balls
# Spinning speed = 360 deg per 5 seconds = 72 deg per second
HEPTAGON_SPIN_SPEED = 72.0  # deg/second

# Ball settings
NUM_BALLS = 20
BALL_RADIUS = 15

# Ball colors
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    omega: float  # spin angular velocity (for demonstration)
    radius: float
    color: str
    number: int


class SpinningHeptagonBalls:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("Spinning Heptagon with Bouncing Balls")
        self.canvas = tk.Canvas(
            self.master, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white"
        )
        self.canvas.pack()

        # Heptagon center
        self.cx = WINDOW_WIDTH / 2
        self.cy = WINDOW_HEIGHT / 2

        # Current rotation angle of the heptagon (in degrees)
        self.heptagon_angle = 0.0

        # Precompute the base (unrotated) heptagon vertices in local space
        self.heptagon_vertices = self._compute_heptagon_points(HEPTAGON_RADIUS)

        # Initialize balls (all starting near center)
        self.balls: List[Ball] = []
        for i in range(NUM_BALLS):
            self.balls.append(
                Ball(
                    x=self.cx,
                    y=self.cy,
                    vx=0.0,
                    vy=0.0,
                    omega=0.0,
                    radius=BALL_RADIUS,
                    color=COLORS[i % len(COLORS)],
                    number=i + 1,
                )
            )

        self._animate()

    def _compute_heptagon_points(self, radius: float):
        """Compute heptagon vertices in local coordinates (center = (0,0))."""
        points = []
        for i in range(HEPTAGON_SIDES):
            angle = 2.0 * math.pi * i / HEPTAGON_SIDES
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append((x, y))
        return points

    def _rotate_point(self, x, y, angle_deg):
        """Rotate point (x, y) around (0,0) by angle_deg degrees, return (xr, yr)."""
        theta = math.radians(angle_deg)
        xr = x * math.cos(theta) - y * math.sin(theta)
        yr = x * math.sin(theta) + y * math.cos(theta)
        return xr, yr

    def _is_outside_local_heptagon(self, x, y, radius):
        """
        Check if a circle center (x,y) with given radius
        is outside the local (unrotated) heptagon.
        If outside, return the index of the nearest edge for collision response.
        Otherwise return -1 if inside.
        """
        # Using winding or any polygon point-in test with offset for the ball's radius is more precise.
        # For simplicity, we check if the center is outside any edge by > radius distance from inside.
        # We'll do a half-plane check: For each edge, if the point is outside, we track it.
        # Return first outside edge or -1 if inside.

        n = len(self.heptagon_vertices)
        for i in range(n):
            x1, y1 = self.heptagon_vertices[i]
            x2, y2 = self.heptagon_vertices[(i + 1) % n]

            # Edge normal pointing "outwards" for a convex polygon
            # Vector from x1,y1 to x2,y2 => (dx,dy)
            dx = x2 - x1
            dy = y2 - y1
            # Normal is ( dy, -dx ) or ( -dy, dx ). For a normal pointing outward,
            # let's ensure it's outward by checking cross product sign with center?
            # However, for a regular polygon oriented with vertices CCW,
            # outward normal can be (dy, -dx).

            # We transform the circle center to a reference relative to x1,y1:
            rx = x - x1
            ry = y - y1

            # Dot product with outward normal:
            # normal = (dy, -dx)
            ndot = (dy * rx) + (-dx * ry)

            # If ndot < -radius => definitely outside the edge
            # but we want to check if distance from line is > radius
            # distance = ndot / edge_length
            edge_len = math.sqrt(dx * dx + dy * dy)
            dist = ndot / edge_len

            if dist < -radius:
                return i  # outside this edge
        return -1

    def _reflect_ball_on_edge(self, ball: Ball, edge_index: int, local_x, local_y):
        """
        Reflect ball velocity in local coordinates on the given edge index.
        local_x, local_y is the current ball center in local space.
        We'll reflect (vx, vy) about the outward normal for that edge.
        """
        x1, y1 = self.heptagon_vertices[edge_index]
        x2, y2 = self.heptagon_vertices[(edge_index + 1) % len(self.heptagon_vertices)]

        dx = x2 - x1
        dy = y2 - y1
        # outward normal for a CCW polygon
        nx = dy
        ny = -dx

        # Normalize
        length = math.sqrt(nx * nx + ny * ny)
        nx /= length
        ny /= length

        # Current velocity in local coords
        # We need to rotate ball.vx, ball.vy from global to local as well,
        # but we've already placed the ball in local space. We still need to do the same rotation
        # for velocity. Let's do that in a separate helper so we keep code simpler.
        lvx, lvy = self._rotate_vector_to_local(ball.vx, ball.vy)

        # Dot product with normal
        vdot = lvx * nx + lvy * ny

        # If velocity is outward, reflect
        # reflection: v' = v - 2*(v dot n)*n
        rx = lvx - 2.0 * vdot * nx
        ry = lvy - 2.0 * vdot * ny

        # Apply elasticity
        rx *= ELASTICITY
        ry *= ELASTICITY

        # Rotate back to global
        gvx, gvy = self._rotate_vector_to_global(rx, ry)
        ball.vx = gvx
        ball.vy = gvy

        # Small positional correction: push the ball slightly inside to avoid
        # repeated collisions in the next frame
        # We'll push the ball along the normal direction
        pen = abs(vdot) * DT * 2.0  # approximate penetration
        px = nx * (pen if vdot < 0 else -pen)
        py = ny * (pen if vdot < 0 else -pen)

        # Rotate this correction back to global as well
        # Actually px, py is still local; convert to global
        gpx, gpy = self._rotate_vector_to_global(px, py)
        ball.x += gpx
        ball.y += gpy

        # Increase spin a bit based on collision (demonstration)
        # Toward random direction
        ball.omega += 0.1 * vdot

    def _rotate_vector_to_local(self, vx, vy):
        """Rotate a velocity (vx, vy) by -self.heptagon_angle to the local frame."""
        return self._rotate_point(vx, vy, -self.heptagon_angle)

    def _rotate_vector_to_global(self, lvx, lvy):
        """Rotate a local velocity (lvx, lvy) by +self.heptagon_angle to global frame."""
        return self._rotate_point(lvx, lvy, self.heptagon_angle)

    def _update_balls(self):
        # Update polygon rotation
        self.heptagon_angle += HEPTAGON_SPIN_SPEED * DT
        self.heptagon_angle %= 360.0  # keep in [0, 360)

        # Update each ball
        for i in range(NUM_BALLS):
            ball = self.balls[i]

            # Apply gravity
            ball.vy += GRAVITY * DT

            # Apply velocity damping (simple friction)
            ball.vx *= VELOCITY_DAMPING
            ball.vy *= VELOCITY_DAMPING

            # Update position
            ball.x += ball.vx * DT
            ball.y += ball.vy * DT

            # Collision with rotating heptagon walls
            # 1) Convert ball center to local coords
            local_x, local_y = self._rotate_point(
                ball.x - self.cx, ball.y - self.cy, -self.heptagon_angle
            )
            # 2) Check if outside
            edge_index = self._is_outside_local_heptagon(local_x, local_y, ball.radius)
            if edge_index != -1:  # outside => reflect
                self._reflect_ball_on_edge(ball, edge_index, local_x, local_y)

            # Collision with other balls
            for j in range(i + 1, NUM_BALLS):
                other = self.balls[j]
                dx = other.x - ball.x
                dy = other.y - ball.y
                dist_sq = dx * dx + dy * dy
                if dist_sq < (ball.radius + other.radius) ** 2:
                    dist = math.sqrt(dist_sq)
                    if dist == 0:
                        # small random push
                        dx = 0.001
                        dy = 0.0
                        dist = 0.001
                    nx = dx / dist
                    ny = dy / dist

                    # Relative velocity
                    rvx = other.vx - ball.vx
                    rvy = other.vy - ball.vy
                    rel_dot = rvx * nx + rvy * ny
                    if rel_dot < 0:
                        # collision impulse for equal mass
                        jval = -(1 + ELASTICITY) * rel_dot / 2
                        # apply impulse
                        impulsex = jval * nx
                        impulsey = jval * ny

                        ball.vx -= impulsex
                        ball.vy -= impulsey
                        other.vx += impulsex
                        other.vy += impulsey

                        # simple correction to avoid sinking in
                        overlap = (ball.radius + other.radius) - dist
                        ball.x -= nx * overlap * 0.5
                        ball.y -= ny * overlap * 0.5
                        other.x += nx * overlap * 0.5
                        other.y += ny * overlap * 0.5

                        # spin correction
                        ball.omega += 0.1 * jval
                        other.omega -= 0.1 * jval

            # Update spin (damp spin too)
            ball.omega *= 0.995

    def _draw_scene(self):
        self.canvas.delete("all")

        # Draw the rotated heptagon
        rotated_points = []
        for lx, ly in self.heptagon_vertices:
            gx, gy = self._rotate_point(lx, ly, self.heptagon_angle)
            rotated_points.append((gx + self.cx, gy + self.cy))

        # Flatten
        poly_coords = []
        for p in rotated_points:
            poly_coords.append(p[0])
            poly_coords.append(p[1])

        self.canvas.create_polygon(poly_coords, fill="", outline="black", width=2)

        # Draw balls
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")

            # Draw ball number (no true text rotation in default tkinter)
            # We'll just place text to show spin value in parentheses for reference
            spin_str = f"{ball.number}\n(ω={ball.omega:.1f})"
            self.canvas.create_text(ball.x, ball.y, text=spin_str, fill="black")

    def _animate(self):
        self._update_balls()
        self._draw_scene()
        self.master.after(int(DT * 1000), self._animate)


def main():
    root = tk.Tk()
    app = SpinningHeptagonBalls(root)
    root.mainloop()


if __name__ == "__main__":
    main()
