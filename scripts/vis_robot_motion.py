from general_motion_retargeting import RobotMotionViewer, load_robot_motion
import argparse
import os
from tqdm import tqdm

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="unitree_g1")
                        
    parser.add_argument("--robot_motion_path", type=str, required=True)

    parser.add_argument("--record_video", action="store_true")
    parser.add_argument("--video_path", type=str, 
                        default="videos/example.mp4")
    parser.add_argument(
        "--highlight_joint_limits",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Highlight joints that are close to their motion limits.",
    )
    parser.add_argument(
        "--joint_limit_warning_ratio",
        type=float,
        default=0.15,
        help="Warn when remaining distance to a joint limit is below this fraction of the joint range.",
    )
    parser.add_argument(
        "--joint_limit_danger_ratio",
        type=float,
        default=0.05,
        help="Mark as danger when remaining distance to a joint limit is below this fraction of the joint range.",
    )
    parser.add_argument(
        "--joint_limit_show_labels",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show joint names next to limit markers.",
    )
                        
    args = parser.parse_args()
    
    robot_type = args.robot
    robot_motion_path = args.robot_motion_path
    
    if not os.path.exists(robot_motion_path):
        raise FileNotFoundError(f"Motion file {robot_motion_path} not found")
    
    motion_data, motion_fps, motion_root_pos, motion_root_rot, motion_dof_pos, motion_local_body_pos, motion_link_body_list = load_robot_motion(robot_motion_path)
    
    env = RobotMotionViewer(robot_type=robot_type,
                            motion_fps=motion_fps,
                            camera_follow=False,
                            record_video=args.record_video,
                            video_path=args.video_path,
                            highlight_joint_limits=args.highlight_joint_limits,
                            joint_limit_warning_ratio=args.joint_limit_warning_ratio,
                            joint_limit_danger_ratio=args.joint_limit_danger_ratio,
                            joint_limit_show_labels=args.joint_limit_show_labels)
    
    frame_idx = 0
    while True:
        env.step(motion_root_pos[frame_idx], 
                motion_root_rot[frame_idx], 
                motion_dof_pos[frame_idx], 
                rate_limit=True)
        frame_idx += 1
        if frame_idx >= len(motion_root_pos):
            frame_idx = 0
    env.close()