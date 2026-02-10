from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.world import World
import asyncio

import omni.appwindow
import carb
from carb.input import KeyboardEventType

class RobotArmVisualizer:
    def __init__(self, robot_prim_path="/World/Robot"):
        """
        Initialize the robot arm visualizer
        
        Args:
            robot_prim_path: USD path to your robot articulation
        """
        self.world = World()
        self.robot_prim_path = robot_prim_path
        
        self.robot = None
        self.articulation_controller = None
        
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
        
        # Get keyboard and input interface
        self.app_window = None
        self.keyboard = None
        self.input = None
        self.keyboard_sub_id = None
        
        self.keys_down = set()
        
        self.step = 0.01     
           
    def setup(self, usd_path=None):
        """Setup the robot articulation"""
        # self.world.scene.add_default_ground_plane()
        
        # Load the robot USD if path provided
        if usd_path:
            print(f"Loading robot from: {usd_path}")
            add_reference_to_stage(usd_path=usd_path, prim_path=self.robot_prim_path)
        
        # Get the robot articulation using the new API
        self.robot = Articulation(prim_paths_expr=self.robot_prim_path, name="pit robot")

        # Create and initialize the articulation controller with the articulation view
        self.articulation_controller = ArticulationController()
        self.articulation_controller.initialize(self.robot)
        
        self.world.reset()
        
        print(f"Robot has {self.robot.num_dof} DOF")
        print(f"Joint names: {self.robot.dof_names}")
        
        # Update number of joints based on actual robot
        self.num_joints = self.robot.num_dof
        
        # Get keyboard and input interface
        self.app_window = omni.appwindow.get_default_app_window()
        self.keyboard = self.app_window.get_keyboard()
        self.input = carb.input.acquire_input_interface()
    
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
        self.steps = int(duration * 60)  # Assuming 60 FPS
        
        for i in range(self.steps):
            alpha = i / self.steps
            # Smooth interpolation
            alpha_smooth = 3*alpha**2 - 2*alpha**3  # Smoothself.step
            
            interp_positions = start_positions + alpha_smooth * (positions - start_positions)
            action = ArticulationAction(joint_positions=interp_positions)
            self.articulation_controller.apply_action(action)
            
            self.world.step(render=True)            
        

    def on_keyboard_input(self, e):
        """ Teleoperated robot arm with keyboard control """
        
        # # Move to home position
        home_position = np.zeros(self.num_joints)
        # self.move_to_joint_positions(home_position, duration=2.0)
        
        q = self.robot.get_joint_positions()
        
        # Only act on press or repeat
        if e.type not in (KeyboardEventType.KEY_PRESS, KeyboardEventType.KEY_REPEAT):
            return
        
        # Start when press, stop when release
        if e.type == KeyboardEventType.KEY_PRESS:
            self.keys_down.add(e.input)
        elif e.type == KeyboardEventType.KEY_RELEASE:
            self.keys_down.discard(e.input)
            
        if e.input == carb.input.KeyboardInput.W: # insert into pipe up
            q[0,0] += self.step
        elif e.input == carb.input.KeyboardInput.S: # insert into pipe down
            q[0,0] -= self.step
            
        elif e.input == carb.input.KeyboardInput.A: # rotate in pipe left
            q[0,1] += self.step
        elif e.input == carb.input.KeyboardInput.D: # rotate in pipe right
            q[0,1] -= self.step
            
        elif e.input == carb.input.KeyboardInput.UP: # joint_1 up
            q[0,2] += self.step
        elif e.input == carb.input.KeyboardInput.DOWN: # joint_1 down
            q[0,2] -= self.step
            
        elif e.input == carb.input.KeyboardInput.LEFT: # joint_2 up
            q[0,3] += self.step
        elif e.input == carb.input.KeyboardInput.RIGHT: # joint_2 right
            q[0,3] += self.step
            
        elif e.input == carb.input.KeyboardInput.R: # end effector joint up
            q[0,4] += self.step
        elif e.input == carb.input.KeyboardInput.F: # end effector joint down
            q[0,4] -= self.step
            
        elif e.input == carb.input.KeyboardInput.C: # joint 3 pulley spin up
            q[0,5] += self.step
        elif e.input == carb.input.KeyboardInput.V: # joint 3 pulley spin down
            q[0,5] -= self.step
            
        elif e.input == carb.input.KeyboardInput.L: # reset
            q = home_position
        
        action = ArticulationAction(joint_positions=q)
        self.articulation_controller.apply_action(action)

        # Return to home
        # self.move_to_joint_positions(q, duration=2.0)
        
        
    def run_demo(self, record=True, usd_path=None):
        """Run the full demonstration"""
        self.setup(usd_path=usd_path)
        
        # Run demonstrations
        self.keyboard_sub_id = self.input.subscribe_to_keyboard_events(
            self.keyboard, 
            self.on_keyboard_input
            )
        
        print("Press Ctrl+C to exit.")


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