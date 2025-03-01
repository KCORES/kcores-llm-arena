import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# --- Constants ---
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
BALL_RADIUS = 15
NUM_BALLS = 20
HEPTAGON_CENTER_X = 300
HEPTAGON_CENTER_Y = 300
HEPTAGON_RADIUS = 200
HEPTAGON_ROTATION_SPEED = 360 / 5  # degrees per second
GRAVITY = 0.5
FRICTION = 0.01
BALL_FRICTION = 0.02
RESTITUTION = 0.6  # Adjust for bounce height
TIME_STEP = 0.02  # Time step for physics updates, smaller is more accurate but slower


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    spin_angle: float
    spin_speed: float


def create_heptagon_vertices(center_x, center_y, radius, angle_offset=0):
    vertices = []
    for i in range(7):
        angle_rad = math.radians(angle_offset + i * 360 / 7)
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        vertices.append((x, y))
    return vertices


def rotate_polygon(points, center_x, center_y, angle_degrees):
    rotated_points = []
    angle_rad = math.radians(angle_degrees)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    for x, y in points:
        rel_x = x - center_x
        rel_y = y - center_y
        rotated_x = rel_x * cos_angle - rel_y * sin_angle + center_x
        rotated_y = rel_x * sin_angle + rel_y * cos_angle + center_y
        rotated_points.append((rotated_x, rotated_y))
    return rotated_points


def draw_heptagon(canvas, vertices, color="black"):
    canvas.create_polygon(vertices, outline=color, fill="")


def draw_ball(canvas, ball):
    x0 = ball.x - ball.radius
    y0 = ball.y - ball.radius
    x1 = ball.x + ball.radius
    y1 = ball.y + ball.radius
    canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")

    # Draw number with rotation approximation (shifting position based on spin)
    angle_rad = ball.spin_angle
    text_x = ball.x + ball.radius * 0.3 * math.cos(angle_rad)
    text_y = ball.y + ball.radius * 0.3 * math.sin(angle_rad)
    canvas.create_text(
        text_x,
        text_y,
        text=str(ball.number),
        fill="white",
        font=("Arial", int(ball.radius * 0.7)),
        angle=math.degrees(ball.spin_angle),
    )


def point_segment_distance(point, segment_p1, segment_p2):
    """Calculates the shortest distance between a point and a line segment."""
    px, py = point
    p1x, p1y = segment_p1
    p2x, p2y = segment_p2

    dx = p2x - p1x
    dy = p2y - p1y

    if dx == 0 and dy == 0:  # Segment is a point
        return math.sqrt((px - p1x) ** 2 + (py - p1y) ** 2)

    t = ((px - p1x) * dx + (py - p1y) * dy) / (dx**2 + dy**2)
    t = max(0, min(1, t))  # Clamp t to be within the segment [0, 1]

    closest_x = p1x + t * dx
    closest_y = p1y + t * dy

    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2), (
        closest_x,
        closest_y,
    )


def reflect_velocity(
    ball_x, ball_y, ball_vx, ball_vy, segment_p1, segment_p2, restitution
):
    """Reflects velocity vector based on collision with a line segment."""
    p1x, p1y = segment_p1
    p2x, p2y = segment_p2

    segment_dx = p2x - p1x
    segment_dy = p2y - p1y

    # Calculate normal vector to the segment
    normal_x = -segment_dy
    normal_y = segment_dx

    # Normalize normal vector
    normal_magnitude = math.sqrt(normal_x**2 + normal_y**2)
    if normal_magnitude == 0:
        return ball_vx, ball_vy  # Avoid division by zero if segment is a point
    normal_x /= normal_magnitude
    normal_y /= normal_magnitude

    # Calculate dot product of velocity and normal vector
    dot_product = ball_vx * normal_x + ball_vy * normal_y

    # Reflection formula: v_reflected = v - 2 * (v dot n) * n
    vx_reflected = ball_vx - 2 * dot_product * normal_x
    vy_reflected = ball_vy - 2 * dot_product * normal_y

    return vx_reflected * restitution, vy_reflected * restitution


def check_wall_collision(ball, vertices, restitution):
    """Checks for collision with heptagon walls and reflects ball velocity."""
    ball_x, ball_y = ball.x, ball.y
    ball_radius = ball.radius
    ball_vx, ball_vy = ball.vx, ball.vy
    collision = False

    for i in range(len(vertices)):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % len(vertices)]
        distance, closest_point = point_segment_distance((ball_x, ball_y), p1, p2)

        if distance <= ball_radius:
            collision = True
            ball_vx, ball_vy = reflect_velocity(
                ball_x, ball_y, ball_vx, ball_vy, p1, p2, restitution
            )
            # Slightly move ball out of collision to prevent sticking
            overlap = ball_radius - distance
            direction_x = ball_x - closest_point[0]
            direction_y = ball_y - closest_point[1]
            direction_magnitude = math.sqrt(direction_x**2 + direction_y**2)
            if direction_magnitude != 0:
                ball.x += (
                    direction_x / direction_magnitude * overlap * 1.01
                )  # A bit more to push out
                ball.y += direction_y / direction_magnitude * overlap * 1.01

    return collision, ball_vx, ball_vy


def check_ball_collision(ball1, ball2, restitution):
    """Checks for collision between two balls and updates their velocities (simplified elastic collision)."""
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    distance_sq = dx**2 + dy**2
    min_distance = ball1.radius + ball2.radius

    if distance_sq < min_distance**2:
        distance = math.sqrt(distance_sq)
        if distance == 0:
            distance = 0.0001  # Avoid division by zero
        nx = dx / distance
        ny = dy / distance

        # Relative velocity
        rvx = ball1.vx - ball2.vx
        rvy = ball1.vy - ball2.vy

        # Velocity along the normal
        vn = rvx * nx + rvy * ny

        if vn < 0:  # Balls are approaching
            # Impulse scalar
            impulse = (
                -(1 + restitution) * vn / (1 / 1 + 1 / 1)
            )  # Assuming same mass (simplified)

            # Impulse vector
            impulse_x = impulse * nx
            impulse_y = impulse * ny

            # Update velocities - equal and opposite impulses
            ball1.vx -= impulse_x * 0.5  # Simplified mass assumption
            ball1.vy -= impulse_y * 0.5
            ball2.vx += impulse_x * 0.5
            ball2.vy += impulse_y * 0.5

            # Separate balls to prevent sticking - basic separation
            overlap = (min_distance - distance) * 0.5
            ball1.x -= nx * overlap * 1.01
            ball1.y -= ny * overlap * 1.01
            ball2.x += nx * overlap * 1.01
            ball2.y += ny * overlap * 1.01
            return True
    return False


def update_frame():
    global heptagon_vertices, heptagon_rotation_angle, balls

    canvas.delete("all")

    # Rotate heptagon
    heptagon_rotation_angle += HEPTAGON_ROTATION_SPEED * TIME_STEP
    rotated_heptagon_vertices = rotate_polygon(
        heptagon_vertices, HEPTAGON_CENTER_X, HEPTAGON_CENTER_Y, heptagon_rotation_angle
    )
    draw_heptagon(canvas, rotated_heptagon_vertices)

    for ball in balls:
        # Gravity
        ball.vy += GRAVITY * TIME_STEP

        # Friction (linear motion)
        ball.vx *= 1 - FRICTION * TIME_STEP
        ball.vy *= 1 - FRICTION * TIME_STEP

        # Friction (rotational spin)
        ball.spin_speed *= 1 - BALL_FRICTION * TIME_STEP
        ball.spin_angle += ball.spin_speed * TIME_STEP

        # Wall collision
        collision_wall, ball_vx, ball_vy = check_wall_collision(
            ball, rotated_heptagon_vertices, RESTITUTION
        )
        if collision_wall:
            ball.vx = ball_vx
            ball.vy = ball_vy

        # Ball-ball collisions
        for other_ball in balls:
            if ball != other_ball:
                check_ball_collision(ball, other_ball, RESTITUTION)

        # Update position
        ball.x += ball.vx * TIME_STEP
        ball.y += ball.vy * TIME_STEP

        draw_ball(canvas, ball)

    root.after(int(TIME_STEP * 1000), update_frame)  # Convert to milliseconds


# --- Initialization ---
root = tk.Tk()
root.title("Bouncing Balls in Heptagon")
canvas_width = 600
canvas_height = 600
canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
canvas.pack()

balls: List[Ball] = []
for i in range(NUM_BALLS):
    balls.append(
        Ball(
            x=HEPTAGON_CENTER_X,
            y=HEPTAGON_CENTER_Y,
            vx=0,
            vy=0,
            radius=BALL_RADIUS,
            color=COLORS[i % len(COLORS)],
            number=i + 1,
            spin_angle=0,
            spin_speed=np.random.uniform(-1, 1) * 2.0,  # Initial random spin speed
        )
    )

heptagon_vertices = create_heptagon_vertices(
    HEPTAGON_CENTER_X, HEPTAGON_CENTER_Y, HEPTAGON_RADIUS
)
heptagon_rotation_angle = 0

update_frame()  # Start animation loop

root.mainloop()
