import math
import tkinter as tk
from dataclasses import dataclass
from typing import List, Tuple

WIDTH = 800
HEIGHT = 800
CENTER = (WIDTH / 2, HEIGHT / 2)
BALL_RADIUS = 10
HEPTAGON_RADIUS = 300  # Adjusted to ensure containment
GRAVITY = 2000  # Adjusted for visible effect
DT = 1 / 60
ROTATION_SPEED = math.radians(72)  # 72 degrees per second
FRICTION = 0.99  # Air resistance
RESTITUTION_WALL = 0.7
RESTITUTION_BALL = 0.6
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
    id: int
    color: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    angular_velocity: float
    angle: float = 0.0


def closest_point_on_segment(
    A: Tuple[float, float], B: Tuple[float, float], P: Tuple[float, float]
) -> Tuple[float, float]:
    Ax, Ay = A
    Bx, By = B
    Px, Py = P

    ax = Bx - Ax
    ay = By - Ay
    length_sq = ax**2 + ay**2
    if length_sq == 0:
        return A

    t = ((Px - Ax) * ax + (Py - Ay) * ay) / length_sq
    t = max(0.0, min(1.0, t))

    return (Ax + t * ax, Ay + t * ay)


def update_heptagon(angle: float) -> List[Tuple[float, float]]:
    vertices = []
    for i in range(7):
        theta = angle + 2 * math.pi * i / 7
        x = CENTER[0] + HEPTAGON_RADIUS * math.cos(theta)
        y = CENTER[1] + HEPTAGON_RADIUS * math.sin(theta)
        vertices.append((x, y))
    return vertices


def check_ball_wall_collisions(ball: Ball, vertices: List[Tuple[float, float]]):
    edges = list(zip(vertices, vertices[1:] + [vertices[0]]))
    for A, B in edges:
        C = closest_point_on_segment(A, B, (ball.x, ball.y))
        Cx, Cy = C
        dx = ball.x - Cx
        dy = ball.y - Cy
        distance_sq = dx**2 + dy**2
        if distance_sq >= ball.radius**2:
            continue

        distance = math.sqrt(distance_sq)
        if distance == 0:
            continue

        nx = dx / distance
        ny = dy / distance
        overlap = ball.radius - distance

        ball.x += overlap * nx
        ball.y += overlap * ny

        v_dot_n = ball.vx * nx + ball.vy * ny
        if v_dot_n > 0:
            ball.vx -= (1 + RESTITUTION_WALL) * v_dot_n * nx
            ball.vy -= (1 + RESTITUTION_WALL) * v_dot_n * ny

        ball.vx *= FRICTION
        ball.vy *= FRICTION


def check_ball_ball_collisions(balls: List[Ball]):
    for i in range(len(balls)):
        b1 = balls[i]
        for j in range(i + 1, len(balls)):
            b2 = balls[j]
            dx = b1.x - b2.x
            dy = b1.y - b2.y
            distance_sq = dx**2 + dy**2
            if distance_sq >= (2 * BALL_RADIUS) ** 2:
                continue

            distance = math.sqrt(distance_sq)
            if distance == 0:
                continue

            nx = dx / distance
            ny = dy / distance
            overlap = 2 * BALL_RADIUS - distance

            b1.x += 0.5 * overlap * nx
            b1.y += 0.5 * overlap * ny
            b2.x -= 0.5 * overlap * nx
            b2.y -= 0.5 * overlap * ny

            v_rel_x = b1.vx - b2.vx
            v_rel_y = b1.vy - b2.vy
            v_dot_n = v_rel_x * nx + v_rel_y * ny
            if v_dot_n <= 0:
                continue

            e = RESTITUTION_BALL
            j = -(1 + e) * v_dot_n
            j /= 2  # Both masses are 1

            imp_x = j * nx
            imp_y = j * ny

            b1.vx += imp_x
            b1.vy += imp_y
            b2.vx -= imp_x
            b2.vy -= imp_y

            b1.vx *= FRICTION
            b1.vy *= FRICTION
            b2.vx *= FRICTION
            b2.vy *= FRICTION


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
    canvas.pack()

    balls = []
    for i in range(20):
        balls.append(
            Ball(
                id=i + 1,
                color=COLORS[i],
                x=CENTER[0],
                y=CENTER[1],
                vx=0.0,
                vy=0.0,
                radius=BALL_RADIUS,
                angular_velocity=0.0,
            )
        )

    angle = 0.0

    def update():
        nonlocal angle
        angle += ROTATION_SPEED * DT
        angle %= 2 * math.pi

        vertices = update_heptagon(angle)

        for ball in balls:
            ball.vy += GRAVITY * DT
            new_x = ball.x + ball.vx * DT
            new_y = ball.y + ball.vy * DT
            ball.x, ball.y = new_x, new_y

            check_ball_wall_collisions(ball, vertices)

            ball.angle += ball.angular_velocity * DT

        check_ball_ball_collisions(balls)

        for ball in balls:
            ball.vx *= FRICTION
            ball.vy *= FRICTION

        canvas.delete("all")

        coords = []
        for x, y in vertices:
            coords.extend([x, y])
        canvas.create_polygon(coords, outline="black", fill="", width=2)

        for ball in balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")
            text_angle = math.degrees(ball.angle) % 360
            canvas.create_text(
                ball.x, ball.y, text=str(ball.id), angle=text_angle, fill="black"
            )

        root.after(int(DT * 1000), update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
