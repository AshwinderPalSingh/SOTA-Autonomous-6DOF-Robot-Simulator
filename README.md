# SOTA Autonomous Factory Simulator (UR5)
 
[License: MIT](https://opensource.org/licenses/MIT)

This MATLAB project is a high-fidelity "Digital Twin" of a state-of-the-art autonomous factory work cell. It simulates a 6-DOF Universal Robots UR5 arm performing a complex, multi-part pick-and-place task.

This "super simulation" moves beyond simple animation and models advanced, intelligent robotics concepts, including:

- **Intelligent Task Planning** (TSP Optimization)
- **Machine Perception** (Simulated Vision)
- **Dynamic Collision Avoidance** (Obstacle & Table)
- **Runtime Fault Recovery** (Grip Failures)
- **Digital Twin Analytics** (Cycle Time & Energy)



## SOTA (State-of-the-Art) Features

This simulation models a truly "smart" factory:

- **Autonomous Task Queue**: Spawn multiple parts with randomized pick and place locations. The robot generates a full task queue to process all parts.

- **TSP Task Optimization**: A "Optimize Task Order" checkbox runs a Traveling Salesperson (TSP) algorithm to find the most efficient, time-saving path to complete the entire job.

- **Simulated Machine Vision**: A "Run Vision Scan" step is required to "detect" the parts. This simulates a real-world perception system, complete with visual detection boxes and (optional) position noise.

- **Runtime Fault & Recovery**: A "Grip Failure Chance" slider injects errors. The robot will dynamically detect a failed grip, log the error, and automatically re-plan a "retry" move (up to 3 times) before skipping the part to prevent line stoppage.

- **Dynamic Collision Avoidance**: A central "Safety Obstacle" and the "Table" are registered as collision objects. The robot performs a pre-flight check and will automatically re-route (e.g., move "up and over") to avoid a collision.

- **High-Fidelity Animation Engine**: All motion is driven by a single, high-framerate timer (30 FPS) for perfectly smooth, clear, and reliable animations (no pause commands).

- **Sensor-Based Moves**: The robot simulates a force-torque sensor by performing slow, "sensing" moves for the final pick/place actions, complete with a visual red sensor cone.

- **Digital Twin Analytics**: After a cycle, the "Performance Analytics" panel displays the Total Cycle Time (sec) and simulated Total Energy Used (kWh), allowing you to quantify the benefits of task optimization.

- **True E-Stop System**: A "EMERGENCY STOP" button will immediately halt any robot motion mid-animation and lock the controls until "Reset" is pressed.

## Standard Features

- **Full 3-Tab GUI**:
  - **Factory Control**: Run autonomous cycles.
  - **Manual Control**: 6 sliders for live joint control.
  - **Cartesian (IK) Control**: Move the robot by typing in a target (X, Y, Z, Roll, Pitch, Yaw) pose.

- **Manual Path Teaching**: A "Manual Programming" tab allows you to save the robot's current pose to a list and "Run Saved Path" to execute the custom sequence.

- **Live Factory Log**: All actions, plans, sensor readings, and errors are reported to a central log.

- **View Controls**: Dedicated buttons to snap the 3D-axis to Isometric (ISO), Top (X-Y), and Side (Y-Z) views.

## Project Structure

  This project is licensed under the MIT License - see the LICENSE file for details (or state the license directly).

