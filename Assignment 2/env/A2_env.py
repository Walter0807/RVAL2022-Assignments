# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC San Diego.
# Created by Yuzhe Qin, Fanbo Xiang

from .stacking_env import StackingEnv
import sapien.core as sapien
import numpy as np
from typing import List, Tuple, Sequence


class A2Env(StackingEnv):
    def __init__(self, timestep: float):
        """Class for Assignment 2

        Args:
            timestep: timestep to balance the precision and the speed of physical simulation
        """
        StackingEnv.__init__(self, timestep)

        self.near, self.far = 0.1, 100
        self.camera_link = self.scene.create_actor_builder().build(is_kinematic=True)
        self.gl_pose = sapien.Pose([0, 0, 0], [0.5, -0.5, 0.5, -0.5])
        self.camera_link.set_pose(sapien.Pose([1.2, 0.0, 0.8], [0, -0.258819, 0, 0.9659258]))
        self.camera = self.scene.add_mounted_camera('fixed_camera', self.camera_link,
                                                    sapien.Pose(), 1920, 1080,
                                                    np.deg2rad(50), np.deg2rad(50), self.near, self.far)

    def cam2base_gt(self) -> np.ndarray:
        """Get ground truth transformation of camera to base transformation

        Returns:
            Ground truth transformation from camera to robot base

        """
        camera_pose = self.camera_link.get_pose() * self.gl_pose
        base_pose = self.robot.get_root_pose()
        return self.pose2mat(base_pose.inv() * camera_pose)

    def board2cam_gt(self) -> np.ndarray:
        """This function is only used to debug your code.

        Note that the output of board2cam_gt() and get_current_marker_pose() will have slight difference,
        e.g. no more than 5cm, due to different definition of checker board frame
        So you can use this function to verify that whether you get the correct marker pose.

        Returns:
            Ground truth transformation from board base to camera

        """
        cam2base = self.cam2base_gt()
        board2base = self.pose2mat(self.robot.get_links()[-5].get_pose())
        return np.linalg.inv(cam2base) @ board2base

    def get_current_ee_pose(self) -> np.ndarray:
        """Get current end effector pose for calibration calculation

        Returns:
            Transformation from end effector to robot base

        """
        return self.pose2mat(self.end_effector.get_pose())

    def get_object_point_cloud(self, seg_id: int) -> np.ndarray:
        """Fetch the object point cloud given segmentation id

        For example, you can use this function to directly get the point cloud of a colored box and use it for further
        calculation.

        Args:
            seg_id: segmentation id, you can get it by e.g. box.get_id()

        Returns:
            (3, n) dimension point cloud in the camera frame with(x, y, z) order

        """
        self.scene.update_render()
        self.camera.take_picture()
        camera_matrix = self.camera.get_camera_matrix()[:3, :3]
        gl_depth = self.camera.get_depth()
        y, x = np.where(gl_depth < 1)
        z = self.near * self.far / (self.far + gl_depth * (self.near - self.far))

        point_cloud = (np.dot(np.linalg.inv(camera_matrix),
                              np.stack([x, y, np.ones_like(x)] * z[y, x], 0)))

        seg_mask = self.camera.get_segmentation()[y, x]
        selected_index = np.nonzero(seg_mask == seg_id)[0]
        return point_cloud[:, selected_index]

    def is_grasped(self, obj: sapien.ActorBase) -> bool:
        """Check whether the robot gripper is holding the object firmly

        Args:
            obj: A object in SAPIEN simulation, e.g. a colored box

        Returns:
            Bool indicator whether the grasp is success

        """
        self.scene.set_timestep(1e-14)
        for i in range(2):
            self.step()
        contacts = self.scene.get_contacts()
        grasped = True
        for q in self.gripper_joints:
            touched = any({obj, q.get_child_link()} == {c.actor1, c.actor2} for c in contacts if c.separation < 2.5e-2)
            grasped = grasped and touched

        return grasped

    def close_gripper(self):
        """Set close gripper command. It will make effect after simulation step"""
        qpos = self.robot.get_qpos()
        qpos[-2:] = [0.02]
        self.robot.set_qpos(qpos)

    def grasp(self, qpos: Sequence[float]):
        """Grasp an object given joint pose. In robotics simulation, q is generalized coordinate which is often joint"""
        self.robot.set_qpos(qpos)
        for i in range(100):
            if self._windows:
                self.render()

        self.close_gripper()
        for i in range(100):
            if self._windows:
                self.render()

    def release_grasp(self):
        """As above, release the gripper"""
        qpos = self.robot.get_qpos()
        qpos[-2:] = [0.04]
        self.robot.set_qpos(qpos)

    @staticmethod
    def compute_ik(ee2base: np.ndarray) -> List[List[float]]:
        """Compute the inverse kinematics of franka panda robot.

        This function is provided to help do the inverse kinematics calculation.
        The output of this function is deterministic.
        It will return a list of solutions for the given cartesian pose.
        In practice, some solutions are not physically-plausible due to self collision.
        So in this homework, you may need to choose the free_joint_value and which solution to use by yourself.

        References:
            ikfast_pybind:
            ikfast_pybind is a python binding generation library for the analytic kinematics engine IKfast.
            Please visit here for more information: https://pypi.org/project/ikfast-pybind/

            ikfast:
            ikfast is a powerful inverse kinematics solver provided within
            Rosen Diankov’s OpenRAVE motion planning software.
            Diankov, R. (2010). Automated construction of robotic manipulation programs.

        Args:
            ee2base: transformation from end effector to base

        Returns:
            A list of possible IK solutions when the last joint value is set as free_joint_value

        """
        try:
            import ikfast_franka_panda as panda
        except ImportError:
            print("Please install ikfast_pybind before using this function")
            print("Install: pip3 install ikfast-pybind")
            raise ImportError

        link72ee = np.array([[0.7071068, -0.7071068, 0, 0], [0.7071068, 0.7071068, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        link7_pose = ee2base @ link72ee
        pos = link7_pose[:3, 3]
        rot = link7_pose[:3, :3]
        return panda.get_ik(pos, rot, [0.785])

    ####################################################################################################################
    # ============================== You will need to implement all the functions below ================================
    ####################################################################################################################
    @staticmethod
    def pose2mat(pose: sapien.Pose) -> np.ndarray:
        """You need to implement this function

        You will need to implement this function first before any other funcions.
        In this function, you need to convert a (position: pose.p, quaternion: pose.q) into a SE(3) matrix

        You can not directly use external library to transform quaternion into rotation matrix.
        Only numpy can be used here.
        Args:
            pose: sapien Pose object, where Pose.p and Pose.q are position and quaternion respectively

        Hint: the convention of quaternion

        Returns:
            (4, 4) transformation matrix represent the same pose

        """

        raise NotImplementedError

    def get_current_marker_pose(self) -> np.ndarray:
        """You need to implement this function

        Using the visual information from an checkerboard to calculate the transformation.
        Some useful fact: checkerboard size: (3, 4), square size: 0.012

        hint: you can use standard library to hep you find the chessboard corner, e.g. OpenCV
        https://docs.opencv.org/4.3.0/dc/dbb/tutorial_py_calibration.html
        pip3 install opencv-python

        Returns:
            Transformation matrix from checkerboard frame to camera frame
        """
        self.scene.update_render()
        self.camera.take_picture()
        image = self.camera.get_color_rgba()[:, :, :3]

        pattern_size = (3, 4)
        square_size = 0.012

        raise NotImplementedError

    def capture_calibration_data(self) -> Tuple[List[np.ndarray], List[np.ndarray]]:
        """You need to implement this function

        Capture transformation matrices that can be used for perform hand eye calibration.
        You can decide how to choose the pose and how many set of data to be captured by modifying qpos_list

        Returns:
            Transformation matrices used for calibration. The length of both list will be equal.
        """
        qpos_list = [list(self.robot.get_qpos()),
                     [0.1, 0.1, 0.1, -1.5, 0, 1.5, 0.7, 0.4, 0.4],
                     [2.6, -1.4, 0.3, -0.5, -2.9, 1.3, 0.4, 0.4, 0.4],
                     [2.6, -1.5, -0.2, 0, -1.6, 1.2, 0.8, 0.4, 0.4],
                     [0.1, 0, 0.2, -1.4, 0, 1.3, 0.6, 0.4, 0.4],
                     [0, 0.2, -0.1, -1.5, -0.3, 1.4, 0.9, 0.4, 0.4],
                     [0.1, 0.3, 0, -1.4, -0.2, 1.7, 0.85, 0.4, 0.4]]

        raise NotImplementedError

    @staticmethod
    def compute_cam2base(marker2cam_poses: List[np.ndarray], ee2base_poses: List[np.ndarray]) -> np.ndarray:
        """You need to implement this function

        Main calculation function for hand eye calibration.
        The input of this function is exactly the output of capture_calibration_data
        Implement an algorithm to solve the AX=XB equation, e.g. Tsai or any other methods

        The reference below list a traditional algorithm to solve this problem, you can also use any other method.

        References:
            Tsai, R. Y., & Lenz, R. K. (1989).
            A new technique for fully autonomous and efficient 3 D robotics hand/eye calibration.
            IEEE Transactions on robotics and automation, 5(3), 345-358.

        Args:
            marker2cam_poses: a list of SE(3) matrices represent poses of marker to camera
            ee2base_poses: a list of SE(3) matrices represent poses of robot hand to robot base (static frame)

        Returns:
            Transformation matrix from camera to base

        """

        raise NotImplementedError

    @staticmethod
    def compute_pose_distance(pose1: np.ndarray, pose2: np.ndarray) -> float:
        """You need to implement this function

        A distance function in SE(3) space
        Args:
            pose1: transformation matrix
            pose2: transformation matrix

        Returns:
            Distance scalar

        """

        raise NotImplementedError

    def compute_grasp_qpos(self, cam2base: np.ndarray, seg_id: int) -> Sequence[float]:
        """You need to implement this function

        Compute the joint values for robot in order to grasp the object with given segmentation id.
        The feasible grasp pose can be extracted from the point cloud, you need to figure out a way (maybe hard-code)
        You can use the provided IK function here but you can not use the ground-truth cam2base transformation

        Note that the end-effector is defined as robot hand, not the finger tip.
        Also the input of the compute_ik function is the pose of hand.
        Thus you may want to estimate the transformation from finger tip to hand (e.g. trail and error)
        in order to make problem easier.


        Args:
            seg_id: segmentation id of the object
            cam2base: transformation from camera to base, calculated from hand-eye calibration

        Returns:
            Joint position of the robot, can be list, one-dimensional numpy array, tuple
            Note that you need to consider the joint position of gripper when return the qpos.
            e.g. len(qpos) == robot.dof == 9

        hint: the inverse kinematics function will calculate the joint angle of robot arm in order to achieve the
            desired SE(3) pose of end-effector(robot hand). Thus it only consider the first 7 dof of the robot. For the
            joint angle of last 2 dof, which are two fingers, will not be calculated by IK. For this panda robot, if the
            gripper is closed, the joint angles (qpos) of the last 2 dof are both 0. Similarly, if joint value is
            0.04, it means that the gripper is fully open. You can play with the GUI and see what will happen if you
            change the value of any joint.


        """
        point_cloud = self.get_object_point_cloud(seg_id)
        raise NotImplementedError
