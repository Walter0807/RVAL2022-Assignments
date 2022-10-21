# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC San Diego.
# Created by Yuzhe Qin, Fanbo Xiang

from env.A2_env import A2Env


def main():
    env = A2Env(timestep=1 / 500)
    env.render()

    print("Press q to continue")
    while not env.should_quit():
        env.render()

    # ==============================Hand Eye Calibration==============================
    marker2cam_poses, ee2base_poses = env.capture_calibration_data()
    cam2base = env.compute_cam2base(marker2cam_poses, ee2base_poses)

    # Since the distance function is implemented by yourself, it's not hard to pass this test
    # However, if the hand-eye calibration error is too large, you will fail to achieve the final objective
    threshold = 0.02
    assert env.compute_pose_distance(env.cam2base_gt(), cam2base) < threshold

    # =========================Place Gripper to Desired Pose=========================
    # If you are unable to achieve hand-eye calibration, you may use GT cam2base
    # cam2base = env.cam2base_gt()
    for box in env.boxes:
        box_id = box.get_id()
        print("Grasping {} box({}).".format(box.name, box_id))
        qpos = env.compute_grasp_qpos(cam2base, box_id)
        assert len(qpos) == env.robot.dof, "Dimension mismatch, do you forget the dof of gripper?"

        env.grasp(qpos)
        if not env.is_grasped(box):
            print("Task fail, observation the visualization and see what happen")
            while not env.should_quit():
                env.render()
            env.close()
            return
        env.release_grasp()

    print("Success! Press q to quit")
    while not env.should_quit():
        env.render()


if __name__ == '__main__':
    main()
