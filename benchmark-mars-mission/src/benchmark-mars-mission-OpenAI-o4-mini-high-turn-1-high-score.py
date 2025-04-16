import numpy as np
import plotly.graph_objects as go

# ----------------------------
# 1) Define physical constants
# ----------------------------
# Astronomical Unit in km
AU_km = 1.495978707e8
# Standard gravitational parameter of Sun in km^3/s^2
mu_sun = 1.32712440018e11
# Convert mu to AU^3 / day^2 for convenience
mu_AU_d2 = mu_sun * (86400.0**2) / AU_km**3

# ----------------------------
# 2) Define orbits & periods
# ----------------------------
r_e = 1.0  # Earth's orbital radius in AU
r_m = 1.524  # Mars  orbital radius in AU

# Kepler's 3rd law (T in days for circular orbit of radius r in AU):
T_e = 2 * np.pi * np.sqrt(r_e**3 / mu_AU_d2)
T_m = 2 * np.pi * np.sqrt(r_m**3 / mu_AU_d2)

# ----------------------------
# 3) Hohmann transfer to Mars
# ----------------------------
a_trans = 0.5 * (r_e + r_m)  # semi-major axis of transfer ellipse
e_trans = (r_m - r_e) / (r_m + r_e)  # eccentricity
t_trans = np.pi * np.sqrt(a_trans**3 / mu_AU_d2)  # time of flight (days)

# synodic period between Earth and Mars (days)
synodic = 2 * np.pi / abs(2 * np.pi / T_e - 2 * np.pi / T_m)

# We will “wait” on Mars synodic days before returning
t_wait = synodic

# To line up departure and arrival, we offset Mars's initial phase so that
# at t = t_trans it will be roughly at the aphelion point of the transfer ellipse
phi_m0 = np.pi - (2 * np.pi / T_m) * t_trans

# ----------------------------
# 4) Build time arrays
# ----------------------------
# outbound, stay, inbound
N1, Nw, N2 = 100, 50, 100
t1 = np.linspace(0, t_trans, N1)  # Earth→Mars
tw = np.linspace(t_trans, t_trans + t_wait, Nw)  # coast/stay at Mars
t2 = np.linspace(t_trans + t_wait, 2 * t_trans + t_wait, N2)  # Mars→Earth
t = np.hstack((t1, tw, t2))

# ----------------------------
# 5) Compute spacecraft trajectory
# ----------------------------
# Outbound true anomalies
f1 = np.pi * (t1 / t_trans)
r1 = a_trans * (1 - e_trans**2) / (1 + e_trans * np.cos(f1))
x1 = r1 * np.cos(f1)
y1 = r1 * np.sin(f1)

# Parked at Mars (just follow Mars's circular orbit during stay)
xw = r_m * np.cos(2 * np.pi / T_m * tw + phi_m0)
yw = r_m * np.sin(2 * np.pi / T_m * tw + phi_m0)

# Inbound: start at Mars (f=0 at t=t_trans+t_wait) going to Earth (f=π)
f2 = np.pi * ((t2 - (t_trans + t_wait)) / t_trans)
r2 = a_trans * (1 - e_trans**2) / (1 + e_trans * np.cos(f2))
# rotate incoming ellipse by π so it goes from Mars back inward
x2 = r2 * np.cos(f2 + np.pi)
y2 = r2 * np.sin(f2 + np.pi)

# full spacecraft trajectory
xs = np.hstack((x1, xw, x2))
ys = np.hstack((y1, yw, y2))
zs = np.zeros_like(xs)


# ----------------------------
# 6) Compute Earth & Mars positions
# ----------------------------
def planet_pos(r, T, times, phase0=0.0):
    """Return (x,y,z) of a circular planet orbit."""
    ang = 2 * np.pi / T * times + phase0
    return r * np.cos(ang), r * np.sin(ang), np.zeros_like(ang)


xe, ye, ze = planet_pos(r_e, T_e, t)
xm, ym, zm = planet_pos(r_m, T_m, t, phase0=phi_m0)

# ----------------------------
# 7) Precompute static orbit‐traces
# ----------------------------
theta = np.linspace(0, 2 * np.pi, 200)
orbit_e = dict(x=r_e * np.cos(theta), y=r_e * np.sin(theta), z=np.zeros_like(theta))
orbit_m = dict(x=r_m * np.cos(theta), y=r_m * np.sin(theta), z=np.zeros_like(theta))
# combined transfer ellipse (for reference)
f_full = np.linspace(0, np.pi, 200)
r_full = a_trans * (1 - e_trans**2) / (1 + e_trans * np.cos(f_full))
trans_out = dict(
    x=r_full * np.cos(f_full), y=r_full * np.sin(f_full), z=np.zeros_like(f_full)
)
trans_in = dict(
    x=r_full * np.cos(f_full + np.pi),
    y=r_full * np.sin(f_full + np.pi),
    z=np.zeros_like(f_full),
)

# ----------------------------
# 8) Build Plotly frames
# ----------------------------
frames = []
for k in range(len(t)):
    frames.append(
        go.Frame(
            data=[
                # Earth
                go.Scatter3d(
                    x=[xe[k]],
                    y=[ye[k]],
                    z=[ze[k]],
                    mode="markers",
                    marker=dict(size=5, color="blue"),
                    name="Earth",
                ),
                # Mars
                go.Scatter3d(
                    x=[xm[k]],
                    y=[ym[k]],
                    z=[zm[k]],
                    mode="markers",
                    marker=dict(size=4, color="red"),
                    name="Mars",
                ),
                # Spacecraft
                go.Scatter3d(
                    x=[xs[k]],
                    y=[ys[k]],
                    z=[zs[k]],
                    mode="markers",
                    marker=dict(size=3, color="green"),
                    name="Spacecraft",
                ),
            ],
            name=f"frame{k}",
        )
    )

# ----------------------------
# 9) Initial (frame=0) data + static orbits
# ----------------------------
data = []
# Static orbits
data.append(
    go.Scatter3d(
        x=orbit_e["x"],
        y=orbit_e["y"],
        z=orbit_e["z"],
        mode="lines",
        line=dict(color="blue", width=1),
        name="Earth Orbit",
    )
)
data.append(
    go.Scatter3d(
        x=orbit_m["x"],
        y=orbit_m["y"],
        z=orbit_m["z"],
        mode="lines",
        line=dict(color="red", width=1),
        name="Mars Orbit",
    )
)
data.append(
    go.Scatter3d(
        x=trans_out["x"],
        y=trans_out["y"],
        z=trans_out["z"],
        mode="lines",
        line=dict(color="green", width=1, dash="dash"),
        name="Transfer Inbound",
    )
)
data.append(
    go.Scatter3d(
        x=trans_in["x"],
        y=trans_in["y"],
        z=trans_in["z"],
        mode="lines",
        line=dict(color="green", width=1, dash="dot"),
        name="Transfer Outbound",
    )
)
# dynamic points at t=0
data.append(
    go.Scatter3d(
        x=[xe[0]],
        y=[ye[0]],
        z=[ze[0]],
        mode="markers",
        marker=dict(size=5, color="blue"),
        name="Earth",
    )
)
data.append(
    go.Scatter3d(
        x=[xm[0]],
        y=[ym[0]],
        z=[zm[0]],
        mode="markers",
        marker=dict(size=4, color="red"),
        name="Mars",
    )
)
data.append(
    go.Scatter3d(
        x=[xs[0]],
        y=[ys[0]],
        z=[zs[0]],
        mode="markers",
        marker=dict(size=3, color="green"),
        name="Spacecraft",
    )
)

# ----------------------------
# 10) Layout & Animation Control
# ----------------------------
layout = go.Layout(
    scene=dict(
        xaxis=dict(title="X [AU]", range=[-2, 2]),
        yaxis=dict(title="Y [AU]", range=[-2, 2]),
        zaxis=dict(title="Z [AU]", range=[-1, 1]),
        aspectmode="data",
    ),
    title="Earth→Mars→Earth Transfer Animation",
    updatemenus=[
        {
            "type": "buttons",
            "buttons": [
                {
                    "label": "Play",
                    "method": "animate",
                    "args": [
                        None,
                        {
                            "frame": {"duration": 50, "redraw": True},
                            "fromcurrent": True,
                            "transition": {"duration": 0},
                        },
                    ],
                }
            ],
        }
    ],
)

fig = go.Figure(data=data, frames=frames, layout=layout)
fig.write_html("earth_mars_return.html")
print("Animation written to earth_mars_return.html")
