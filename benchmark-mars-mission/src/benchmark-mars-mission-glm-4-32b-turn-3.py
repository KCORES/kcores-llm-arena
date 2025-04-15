import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.patches import Patch

# Astronomical constants (in AU and days)
AU = 149597870.7  # 1 AU in kilometers
EARTH_ORBIT = 1.0  # AU
MARS_ORBIT = 1.524  # AU
EARTH_PERIOD = 365.25  # days
MARS_PERIOD = 686.98  # days
EARTH_INCLINATION = 0.0  # degrees
MARS_INCLINATION = 1.85  # degrees

# Mission parameters
FRAMES = 500
TRANSFER_TIME_TO_MARS = 259  # days (Hohmann transfer time)
STAY_TIME_ON_MARS = 458  # days (waiting for next launch window)
TRANSFER_TIME_TO_EARTH = 259  # days

class SpaceMission:
    def __init__(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
    def setup_plot(self):
        """Initialize the 3D plot settings"""
        self.ax.set_xlabel('X (AU)')
        self.ax.set_ylabel('Y (AU)')
        self.ax.set_zlabel('Z (AU)')
        self.ax.set_title('Earth-Mars Mission Trajectory')
        
        # Set axis limits
        limit = 2
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(-limit/2, limit/2)
        
        # Add legend
        legend_elements = [
            Patch(facecolor='blue', label='Earth'),
            Patch(facecolor='red', label='Mars'),
            Patch(facecolor='yellow', label='Sun'),
            Patch(facecolor='gray', label='Spacecraft')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')

    def calculate_planet_position(self, t, semi_major_axis, period, inclination):
        """Calculate planet position at time t"""
        angle = (2 * np.pi * t / period) % (2 * np.pi)
        x = semi_major_axis * np.cos(angle)
        y = semi_major_axis * np.sin(angle)
        z = semi_major_axis * np.sin(angle) * np.sin(np.radians(inclination))
        return x, y, z

    def calculate_transfer_orbit(self, start_pos, end_pos, progress):
        """Calculate transfer orbit position"""
        # Simple elliptical interpolation
        return [start_pos[i] + (end_pos[i] - start_pos[i]) * progress for i in range(3)]

    def animate(self, frame):
        """Animation function"""
        self.ax.clear()
        self.setup_plot()
        
        t = frame * (TRANSFER_TIME_TO_MARS + STAY_TIME_ON_MARS + TRANSFER_TIME_TO_EARTH) / FRAMES
        
        # Calculate planet positions
        earth_pos = self.calculate_planet_position(t, EARTH_ORBIT, EARTH_PERIOD, EARTH_INCLINATION)
        mars_pos = self.calculate_planet_position(t, MARS_ORBIT, MARS_PERIOD, MARS_INCLINATION)
        
        # Draw orbits
        theta = np.linspace(0, 2*np.pi, 100)
        
        # Earth's orbit
        earth_orbit_x = EARTH_ORBIT * np.cos(theta)
        earth_orbit_y = EARTH_ORBIT * np.sin(theta)
        earth_orbit_z = np.zeros_like(theta)
        self.ax.plot(earth_orbit_x, earth_orbit_y, earth_orbit_z, 'b--', alpha=0.3)
        
        # Mars' orbit
        mars_orbit_x = MARS_ORBIT * np.cos(theta)
        mars_orbit_y = MARS_ORBIT * np.sin(theta)
        mars_orbit_z = MARS_ORBIT * np.sin(theta) * np.sin(np.radians(MARS_INCLINATION))
        self.ax.plot(mars_orbit_x, mars_orbit_y, mars_orbit_z, 'r--', alpha=0.3)
        
        # Draw Sun
        self.ax.scatter([0], [0], [0], color='yellow', s=100)
        
        # Draw Earth
        self.ax.scatter(earth_pos[0], earth_pos[1], earth_pos[2], color='blue', s=50)
        
        # Draw Mars
        self.ax.scatter(mars_pos[0], mars_pos[1], mars_pos[2], color='red', s=40)
        
        # Calculate spacecraft position
        if t <= TRANSFER_TIME_TO_MARS:
            # Outbound journey
            progress = t / TRANSFER_TIME_TO_MARS
            start_pos = self.calculate_planet_position(0, EARTH_ORBIT, EARTH_PERIOD, EARTH_INCLINATION)
            end_pos = self.calculate_planet_position(TRANSFER_TIME_TO_MARS, MARS_ORBIT, MARS_PERIOD, MARS_INCLINATION)
            spacecraft_pos = self.calculate_transfer_orbit(start_pos, end_pos, progress)
            
        elif t <= TRANSFER_TIME_TO_MARS + STAY_TIME_ON_MARS:
            # Staying on Mars
            spacecraft_pos = mars_pos
            
        else:
            # Return journey
            progress = (t - TRANSFER_TIME_TO_MARS - STAY_TIME_ON_MARS) / TRANSFER_TIME_TO_EARTH
            start_pos = self.calculate_planet_position(TRANSFER_TIME_TO_MARS + STAY_TIME_ON_MARS, 
                                                     MARS_ORBIT, MARS_PERIOD, MARS_INCLINATION)
            end_pos = self.calculate_planet_position(TRANSFER_TIME_TO_MARS + STAY_TIME_ON_MARS + TRANSFER_TIME_TO_EARTH,
                                                   EARTH_ORBIT, EARTH_PERIOD, EARTH_INCLINATION)
            spacecraft_pos = self.calculate_transfer_orbit(start_pos, end_pos, progress)
        
        # Draw spacecraft
        self.ax.scatter(spacecraft_pos[0], spacecraft_pos[1], spacecraft_pos[2], color='gray', s=20)
        
        # Add time indicator
        self.ax.text2D(0.02, 0.95, f'Mission Time: {int(t)} days', transform=self.ax.transAxes)
        
        # Add mission phase indicator
        if t <= TRANSFER_TIME_TO_MARS:
            phase = "Outbound Journey to Mars"
        elif t <= TRANSFER_TIME_TO_MARS + STAY_TIME_ON_MARS:
            phase = "Stay on Mars"
        else:
            phase = "Return Journey to Earth"
        self.ax.text2D(0.02, 0.90, phase, transform=self.ax.transAxes)

def main():
    mission = SpaceMission()
    anim = animation.FuncAnimation(mission.fig, mission.animate, frames=FRAMES,
                                  interval=50, blit=False)
    plt.show()

if __name__ == "__main__":
    main()