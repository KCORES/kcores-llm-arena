import tkinter as tk
import math
from dataclasses import dataclass


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    mass: float
    omega: float  # angular velocity (radians/sec)
    angle: float  # current rotation angle
    color: str
    number: int


@dataclass
class Heptagon:
    cx: float
    cy: float
    radius: float
    angular_velocity: float
    theta: float = 0.0  # current rotation angle


def compute_heptagon_vertices(heptagon):
    """Returns the (x, y) coordinates of the heptagon vertices."""
    vertices = []
    angle_increment = 2 * math.pi / 7
    for i in range(7):
        theta = heptagon.theta + i * angle_increment
        x = heptagon.cx + heptagon.radius * math.cos(theta)
        y = heptagon.cy + heptagon.radius * math.sin(theta)
        vertices.append((x, y))
    return vertices


def closest_point_on_segment(px, py, seg_p1, seg_p2):
    """Finds the closest point on a segment to a given point."""
    seg_dx = seg_p2[0] - seg_p1[0]
    seg_dy = seg_p2[1] - seg_p1[1]
    seg_len_sq = seg_dx**2 + seg_dy**2
    if seg_len_sq == 0:
        return seg_p1
    t = ((px - seg_p1[0]) * seg_dx + (py - seg_p1[1]) * seg_dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = seg_p1[0] + t * seg_dx
    proj_y = seg_p1[1] + t * seg_dy
    return (proj_x, proj_y)


def handle_ball_wall_collisions(balls, heptagon, dt):
    """Detect and resolve collisions between balls and the rotating heptagon walls."""
    vertices = compute_heptagon_vertices(heptagon)
    edges = []
    for i in range(len(vertices)):
        edges.append((vertices[i], vertices[(i + 1) % len(vertices)]))

    e_wall = 0.8  # restitution coefficient for walls
    for ball in balls:
        for p1, p2 in edges:
            cp = closest_point_on_segment(ball.x, ball.y, p1, p2)
            dx = ball.x - cp[0]
            dy = ball.y - cp[1]
            dist = math.hypot(dx, dy)
            if dist >= ball.radius:
                continue

            # Compute inward-pointing normal
            ex = p2[0] - p1[0]
            ey = p2[1] - p1[1]
            nx = -ey
            ny = ex
            norm = math.hypot(nx, ny)
            if norm == 0:
                continue
            nx /= norm
            ny /= norm

            # Resolve penetration
            penetration = ball.radius - dist
            ball.x += nx * penetration
            ball.y += ny * penetration

            # Wall velocity at collision point
            rel_x = cp[0] - heptagon.cx
            rel_y = cp[1] - heptagon.cy
            wall_vx = -heptagon.angular_velocity * rel_y
            wall_vy = heptagon.angular_velocity * rel_x

            # Relative velocity
            rel_vel_x = ball.vx - wall_vx
            rel_vel_y = ball.vy - wall_vy
            vn = rel_vel_x * nx + rel_vel_y * ny
            if vn > 0:
                continue

            # Impulse-based collision response
            j = -(1 + e_wall) * vn
            ball.vx += j * nx
            ball.vy += j * ny


def handle_ball_ball_collisions(balls, dt):
    """Detect and resolve collisions between balls."""
    e = 0.8  # restitution coefficient
    n = len(balls)
    for i in range(n):
        a = balls[i]
        for j in range(i + 1, n):
            b = balls[j]
            dx = b.x - a.x
            dy = b.y - a.y
            dist_sq = dx * dx + dy * dy
            min_dist = a.radius + b.radius
            if dist_sq >= min_dist * min_dist:
                continue

            dist = math.hypot(dx, dy)
            if dist == 0:
                dx, dy, dist = 0.1, 0.1, 0.1414

            penetration = (min_dist - dist) / 2
            nx = dx / dist
            ny = dy / dist

            a.x -= penetration * nx
            a.y -= penetration * ny
            b.x += penetration * nx
            b.y += penetration * ny

            dvx = a.vx - b.vx
            dvy = a.vy - b.vy
            vn = dvx * nx + dvy * ny
            if vn > 0:
                continue

            impulse = (1 + e) * vn / 2
            a.vx -= impulse * nx
            a.vy -= impulse * ny
            b.vx += impulse * nx
            b.vy += impulse * ny


def update_physics(dt_sec):
    """Update ball positions and handle collisions."""
    global heptagon, balls
    heptagon.theta += heptagon.angular_velocity * dt_sec
    for ball in balls:
        g = 9.8 * 100  # gravity (scaled for pixels)
        ball.vy += g * dt_sec
        drag = 0.01
        ball.vx *= 1 - drag * dt_sec
        ball.vy *= 1 - drag * dt_sec
        angular_drag = 0.01
        ball.omega *= 1 - angular_drag * dt_sec
        ball.angle += ball.omega * dt_sec
        ball.x += ball.vx * dt_sec
        ball.y += ball.vy * dt_sec
    handle_ball_wall_collisions(balls, heptagon, dt_sec)
    handle_ball_ball_collisions(balls, dt_sec)


def render():
    """Draw the scene on the canvas."""
    canvas.delete("all")
    vertices = compute_heptagon_vertices(heptagon)
    flat_coords = []
    for x, y in vertices:
        flat_coords.extend([x, y])
    canvas.create_polygon(flat_coords, outline="black", fill="", width=2)
    for ball in balls:
        r = ball.radius
        x, y = ball.x, ball.y
        canvas.create_oval(x - r, y - r, x + r, y + r, fill=ball.color, outline="black")
        line_len = r * 0.75
        lx = x + line_len * math.cos(ball.angle)
        ly = y + line_len * math.sin(ball.angle)
        canvas.create_line(x, y, lx, ly, fill="black", width=2)
        canvas.create_text(x, y, text=str(ball.number), fill="black")


def main():
    global canvas, root, heptagon, balls
    root = tk.Tk()
    width, height = 800, 800
    canvas = tk.Canvas(root, width=width, height=height)
    canvas.pack()

    heptagon = Heptagon(cx=400, cy=400, radius=300, angular_velocity=2 * math.pi / 5)

    colors = [
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

    ball_radius = 15
    balls = []
    num_balls = 20
    angle_step = 2 * math.pi / num_balls
    for i in range(num_balls):
        angle = i * angle_step
        x = heptagon.cx + (ball_radius * 2) * math.cos(angle)
        y = heptagon.cy + (ball_radius * 2) * math.sin(angle)
        balls.append(
            Ball(
                x=x,
                y=y,
                vx=0.0,
                vy=0.0,
                radius=ball_radius,
                mass=1.0,
                omega=0.0,
                angle=0.0,
                color=colors[i],
                number=i + 1,
            )
        )

    def _update():
        update_physics(0.01)
        render()
        root.after(10, _update)

    _update()
    root.mainloop()


if __name__ == "__main__":
    main()
