"""
Isaac Sim script to visualize robot arm configuration space and record video
This script moves a 3-link + end effector robot arm through its full range of motion
Updated for isaacsim.core API (Isaac Sim 4.0+)
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
import omni.replicator.core as rep
import carb

class RobotArmVisualizer:
    def __init__(self, robot_prim_path="/World/Robot"):
        """
        Initialize the robot arm visualizer
        
        Args:
            robot_prim_path: USD path to your robot articulation
        """
        self.world = World()
        self.robot_prim_path = robot_prim_path
        
        # Will load robot in setup()
        
        self.robot = None

        
        # Configure joint ranges (adjust these based on your robot's limits)
        self.joint_limits = {
            "insert_into_pipe": (0, 0.5), # prismatic
            "rotate_in_pipe": (-np.pi, np.pi), # rev
            "yaw_mount_to_base": (0.0, 0.0), # fixed
            "joint_1": (-np.pi/2, np.pi/2), # rev
            "joint_2": (-np.pi/2, np.pi/2), # rev
            "end_effector_joint": (-np.pi/2, np.pi/2), # rev
            "joint_3_pulley_spin": (-np.pi, np.pi), # rev
            "camera_link_to_pulley": (0.0,0.0), # fixed
            # "zed_joints": {
            #     "camera_link_to_zed_x_mini": (0.0,0.0), # fixed
            #     "zed_base_to_right": (0.0,0.0), # fixed
            #     "zed_base_to_imu": (0.0,0.0), # fixed
            #     "zed_left_to_optical": (0.0,0.0), # fixed
            # }
        }
        
        self.num_joints = None
        
        self.recording = False
        self.render_product = None
        
    def setup(self, usd_path=None):
        """Setup the robot articulation"""
        # self.world.scene.add_default_ground_plane()
        
        # Load the robot USD if path provided
        if usd_path:
            print(f"Loading robot from: {usd_path}")
            add_reference_to_stage(usd_path=usd_path, prim_path=self.robot_prim_path)
        
        # Get the robot articulation using the new API
        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=self.robot_prim_path, name="pit_robot_assembly")
        )
        
        self.world.reset()
        
        print(f"Robot has {self.robot.num_dof} DOF")
        print(f"Joint names: {self.robot.dof_names}")
        
        # Update number of joints based on actual robot
        self.num_joints = self.robot.num_dof
        
    def setup_recording(self, output_path="robot_motion.mp4", fps=30): # use isaac record flag ignore this
        """Setup video recording"""
        camera_path = "/World/Camera"
        
        # Create camera if it doesn't exist
        import isaacsim.core.utils.prims as prim_utils
        from pxr import UsdGeom, Gf
        
        if not prim_utils.is_prim_path_valid(camera_path):
            camera_prim = UsdGeom.Camera.Define(self.world.stage, camera_path)
            camera_prim.GetPrim().GetAttribute("focalLength").Set(24)
            
        # Position camera to view robot
        prim_utils.set_prim_attribute_value(camera_path, "xformOp:translate", Gf.Vec3d(3, 3, 2))
        prim_utils.set_prim_attribute_value(camera_path, "xformOp:rotateXYZ", Gf.Vec3d(-20, 45, 0))
        
        # Setup replicator for recording
        self.render_product = rep.create.render_product(camera_path, (1920, 1080))
        
        # Create video writer
        self.writer = rep.WriterRegistry.get("mp4_video")
        self.writer.initialize(output_dir="./", fps=fps)
        self.writer.attach([self.render_product])
        
        self.recording = True
        print(f"Recording setup complete. Output: {output_path}")
    
    def move_to_joint_positions(self, positions, duration=2.0):
        """
        Smoothly move robot to target joint positions
        
        Args:
            positions: Array of target joint positions
            duration: Time to reach target (seconds)
        """
        if self.robot is None:
            print("Robot not initialized. Call setup() first.")
            return
            
        start_positions = self.robot.get_joint_positions()
        steps = int(duration * 60)  # Assuming 60 FPS
        
        for i in range(steps):
            alpha = i / steps
            # Smooth interpolation
            alpha_smooth = 3*alpha**2 - 2*alpha**3  # Smoothstep
            
            interp_positions = start_positions + alpha_smooth * (positions - start_positions)
            self.robot.set_joint_positions(interp_positions)
            
            self.world.step(render=True)
            
    def demonstrate_full_range(self, samples_per_joint=5):
        """
        Demonstrate full configuration space by sampling joint combinations
        
        Args:
            samples_per_joint: Number of samples per joint dimension
        """
        print(f"Demonstrating configuration space with {samples_per_joint} samples per joint...")
        
        # Move to home position
        home_position = np.zeros(self.num_joints)
        self.move_to_joint_positions(home_position, duration=2.0)
        
        # Generate sample points for each joint
        joint_samples = {}
        for joint_idx, joint_name in enumerate(self.robot.dof_names):
            if joint_name in self.joint_limits: # ignore zed
                min_val, max_val = self.joint_limits[joint_name]
                samples = np.linspace(min_val, max_val, samples_per_joint)
                joint_samples[joint_idx] = samples
        
        # Strategy 1: Move each joint independently
        print("Phase 1: Individual joint motion")
        for joint_idx, joint_name in enumerate(self.robot.dof_names):
            if joint_name in self.joint_limits:
                for value in joint_samples[joint_idx]:  # Use joint_idx
                    target = home_position.copy()
                    target[joint_idx] = value  # Use joint_idx
                    self.move_to_joint_positions(target, duration=2.0)
                    self.world.step(render=True)
            
            # Return to home
            self.move_to_joint_positions(home_position, duration=2.0)
        
        # Strategy 2: Combined motion - sample configuration space
        print("Phase 2: Combined motion patterns")
        num_combined_samples = 20
        
        for _ in range(num_combined_samples):
            # Random configuration within limits
            target = np.array([
                np.random.uniform(*self.joint_limits[name]) if name in self.joint_limits 
                else 0.0
                for name in self.robot.dof_names
            ])
            self.move_to_joint_positions(target, duration=2.0)
            self.world.step(render=True)
        
        # Return to home
        self.move_to_joint_positions(home_position, duration=2.0)
        print("Configuration space demonstration complete!")
        
    def demonstrate_circular_motion(self, radius_scale=0.8):
        """Demonstrate smooth circular motion pattern"""
        print("Phase 3: Circular motion pattern")
        
        # Get actuated joints (skip fixed  and zed joints)
        actuated_joints = []
        for name in self.robot.dof_names:
            if name in self.joint_limits:
                min_val, max_val = self.joint_limits[name]
                if (min_val, max_val) != (0.0, 0.0):
                    actuated_joints.append(name)
        
        print(f"Actuated joints for circular motion: {actuated_joints}")
        
        steps = 120
        # Different motion patterns for variety
        motion_patterns = [np.sin, np.cos, lambda x: np.sin(2*x), lambda x: np.cos(2*x)]
        
        for i in range(steps):
            angle = 2 * np.pi * i / steps
            target = np.zeros(self.robot.num_dof)
            
            # Build target positions for each joint
            for joint_idx, joint_name in enumerate(self.robot.dof_names):
                # Get joint limits
                if joint_name in self.joint_limits:
                    min_val, max_val = self.joint_limits[joint_name]
                else:
                    # Unknown joint, keep at zero
                    target[joint_idx] = 0.0
                    continue
                
                # Skip fixed joints
                if (min_val, max_val) == (0.0, 0.0):
                    target[joint_idx] = 0.0
                else:
                    # Use different motion pattern for each actuated joint
                    actuated_idx = actuated_joints.index(joint_name) if joint_name in actuated_joints else 0
                    pattern_func = motion_patterns[actuated_idx % len(motion_patterns)]
                    
                    # Calculate position with pattern
                    range_half = (max_val - min_val) / 2
                    center = (max_val + min_val) / 2
                    target[joint_idx] = center + radius_scale * pattern_func(angle) * range_half
            
            self.robot.set_joint_positions(target)
            self.world.step(render=True)
    
    def run_demo(self, record=True, usd_path=None):
        """Run the full demonstration"""
        self.setup(usd_path=usd_path)
        
        if record:
            self.setup_recording()
        
        # Run demonstrations
        self.demonstrate_full_range(samples_per_joint=4)
        self.demonstrate_circular_motion()
        
        if self.recording:
            print("Finalizing recording...")
            # Let it render a bit more
            for _ in range(30):
                self.world.step(render=True)
        
        print("Demo complete! Press Ctrl+C to exit.")


def main():
    """Main function to run the visualization"""
    
    # Path to your robot USD file
    USD_PATH = r"C:\Users\LICF\hanford_wire_manipulator_with_camera_description\usd\hanford_arm.usd"
    ROBOT_PATH = "/World/hanford_arm"  # Prim path where robot will be loaded
    
    visualizer = RobotArmVisualizer(robot_prim_path=ROBOT_PATH)
    
    try:
        visualizer.run_demo(record=False, usd_path=USD_PATH)
        
        # Keep simulation running
        while simulation_app.is_running():
            visualizer.world.step(render=True)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()