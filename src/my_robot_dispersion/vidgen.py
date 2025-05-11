import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from matplotlib.gridspec import GridSpec

# Parameters
lower_threshold = 7  # Minimum distance to freeze
upper_threshold = 8  # Maximum allowed distance
robot_speed = 0.1    # Movement speed
turn_angle = 45      # Degrees (45° increments)
max_steps = 150      # Increased maximum animation frames

class Robot:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.angle = np.random.uniform(0, 360)
        self.frozen = False
        self.path = [(x, y)]
        self.target_angle = None  # For the third robot's navigation
        
    def move_forward(self, distance):
        rad = math.radians(self.angle)
        self.x += distance * math.cos(rad)
        self.y += distance * math.sin(rad)
        self.path.append((self.x, self.y))
        
    def turn(self, degrees):
        self.angle = (self.angle + degrees) % 360
        
    def distance_to(self, other_robot):
        return math.sqrt((self.x - other_robot.x)**2 + (self.y - other_robot.y)**2)
    
    def get_rssi(self, other_robot):
        # RSSI decreases with distance (simplified)
        distance = self.distance_to(other_robot)
        return 1 / (distance + 0.001)

def create_simulation(ax):
    # Initialize robots with random positions
    robots = [
        Robot(random.uniform(-2, 2), random.uniform(-2, 2), 'red'),
        Robot(random.uniform(-2, 2), random.uniform(-2, 2), 'blue'),
        Robot(random.uniform(-2, 2), random.uniform(-2, 2), 'green')
    ]
    
    def update(frame):
        ax.clear()
        
        # Check distances between all pairs
        distances = {}
        frozen_pairs = set()
        for i in range(len(robots)):
            for j in range(i+1, len(robots)):
                dist = robots[i].distance_to(robots[j])
                distances[(i,j)] = dist
                if dist >= lower_threshold:
                    frozen_pairs.add(i)
                    frozen_pairs.add(j)
        
        # Freeze robots that have reached threshold with at least one other
        active_robots = [i for i, r in enumerate(robots) if not r.frozen]
        for i in frozen_pairs:
            if len(active_robots) > 1:  # Don't freeze if it would leave no active robots
                robots[i].frozen = True
        
        # Update robot positions (only for active robots)
        for i, robot in enumerate(robots):
            if robot.frozen:
                continue
                
            # Check if we're the last active robot
            is_last_active = len([r for r in robots if not r.frozen]) == 1
            
            if is_last_active:
                frozen_robots = [r for r in robots if r.frozen]
                dists = [robot.distance_to(r) for r in frozen_robots]
                
                # Check if we're in the target zone with both frozen robots
                if all(lower_threshold <= d <= upper_threshold for d in dists):
                    robot.frozen = True
                    continue
                    
                # Calculate desired movement for the third robot
                target_vectors = []
                for r in frozen_robots:
                    dist = robot.distance_to(r)
                    if dist < lower_threshold:
                        # Need to move away
                        dx = robot.x - r.x
                        dy = robot.y - r.y
                        target_vectors.append((dx, dy))
                    elif dist > upper_threshold:
                        # Need to move closer
                        dx = r.x - robot.x
                        dy = r.y - robot.y
                        target_vectors.append((dx, dy))
                    else:
                        # In good range with this robot
                        target_vectors.append((0, 0))
                
                # Combine target vectors
                if target_vectors:
                    combined_dx = sum(v[0] for v in target_vectors)
                    combined_dy = sum(v[1] for v in target_vectors)
                    
                    if combined_dx != 0 or combined_dy != 0:
                        # Calculate target angle
                        target_angle = math.degrees(math.atan2(combined_dy, combined_dx))
                        angle_diff = (target_angle - robot.angle + 180) % 360 - 180
                        
                        # Turn toward target angle if not already facing it
                        if abs(angle_diff) > 10:  # 10 degree tolerance
                            robot.turn(np.sign(angle_diff) * min(abs(angle_diff), turn_angle))
                        else:
                            # Move forward if roughly facing the right direction
                            robot.move_forward(robot_speed)
                else:
                    # Random walk if already in target zone
                    current_rssi = sum(robot.get_rssi(other) for other in robots if other != robot)
                    old_x, old_y = robot.x, robot.y
                    robot.move_forward(robot_speed)
                    new_rssi = sum(robot.get_rssi(other) for other in robots if other != robot)
                    if new_rssi >= current_rssi:  # Not moving away
                        robot.x, robot.y = old_x, old_y
                        robot.path.pop()
                        robot.turn(turn_angle)
            else:
                # Regular movement for non-last active robots
                current_rssi = sum(robot.get_rssi(other) for other in robots if other != robot)
                old_x, old_y = robot.x, robot.y
                robot.move_forward(robot_speed)
                new_rssi = sum(robot.get_rssi(other) for other in robots if other != robot)
                
                # Check if we exceeded upper threshold with any robot
                exceeded_upper = any(robot.distance_to(r) >= upper_threshold for r in robots if r != robot)
                
                if new_rssi >= current_rssi or exceeded_upper:
                    robot.x, robot.y = old_x, old_y
                    robot.path.pop()
                    robot.turn(turn_angle)
        
        # Visualization
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.set_aspect('equal')
        ax.set_title(f'Step {frame}', fontsize=10)
        ax.grid(True)
        
        # Draw threshold zones
        for robot in robots:
            if robot.frozen:
                # Lower threshold (freeze zone)
                circle = plt.Circle((robot.x, robot.y), lower_threshold, 
                                  color=robot.color, alpha=0.1, linestyle='--')
                ax.add_patch(circle)
                # Upper threshold (limit zone)
                circle = plt.Circle((robot.x, robot.y), upper_threshold, 
                                  color='black', alpha=0.05, linestyle=':')
                ax.add_patch(circle)
        
        # Draw robots and paths
        for robot in robots:
            if len(robot.path) > 1:
                path_x, path_y = zip(*robot.path)
                ax.plot(path_x, path_y, color=robot.color, alpha=0.3)
            
            marker = 's' if robot.frozen else 'o'
            ax.plot(robot.x, robot.y, marker, color=robot.color, markersize=6)
            
            # Draw orientation arrow
            arrow_length = 0.5
            dx = arrow_length * math.cos(math.radians(robot.angle))
            dy = arrow_length * math.sin(math.radians(robot.angle))
            ax.arrow(robot.x, robot.y, dx, dy, 
                    head_width=0.2, head_length=0.2, fc=robot.color, ec=robot.color)
            
            status = "FROZEN" if robot.frozen else "ACTIVE"
            ax.text(robot.x, robot.y + 0.5, status, 
                   color=robot.color, ha='center', fontsize=6)
        
        # Display distances
        for (i,j), dist in distances.items():
            mid_x = (robots[i].x + robots[j].x)/2
            mid_y = (robots[i].y + robots[j].y)/2
            ax.text(mid_x, mid_y, f"{dist:.2f}", ha='center', fontsize=6,
                   bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    
    return FuncAnimation(fig, update, frames=max_steps, interval=200, repeat=False)

# Create a 2x2 grid of animations
fig = plt.figure(figsize=(12, 10))
gs = GridSpec(2, 2, figure=fig)
fig.suptitle('Robot Dispersion Simulations (4 instances)', fontsize=14)

# Create axes and animations
ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[0, 1])
ax3 = fig.add_subplot(gs[1, 0])
ax4 = fig.add_subplot(gs[1, 1])

ani1 = create_simulation(ax1)
ani2 = create_simulation(ax2)
ani3 = create_simulation(ax3)
ani4 = create_simulation(ax4)

plt.tight_layout()

# Save the animation
print("Saving animation...")
writer = 'ffmpeg'  # or 'imagemagick'
fps = 10
filename = "robot_dispersion_grid.mp4"

# Combine all animations
# Note: This is a workaround since FuncAnimation doesn't directly support multiple animations
# We'll create a new animation that updates all subplots
def combined_update(frame):
    ani1._func(frame)
    ani2._func(frame)
    ani3._func(frame)
    ani4._func(frame)
    return []

combined_ani = FuncAnimation(fig, combined_update, frames=max_steps, interval=200, repeat=False)
combined_ani.save(filename, writer=writer, fps=fps, dpi=300)
print(f"Animation saved as {filename}")

plt.show()