import argparse
import pathlib
import time

import numpy as np
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.smpl import (
    get_smplx_data_offline_fast,
    load_smplx_file,
)


def anchor_human_frame_to_pelvis(human_frame, mode="xy"):
    if mode == "none":
        return human_frame

    pelvis_pos = np.asarray(human_frame["pelvis"][0], dtype=np.float64)
    if mode == "xy":
        offset = np.array([pelvis_pos[0], pelvis_pos[1], 0.0], dtype=np.float64)
    elif mode == "xyz":
        offset = pelvis_pos.copy()
    else:
        raise ValueError(f"Unsupported anchor mode: {mode}")

    anchored_frame = {}
    for body_name, (pos, rot) in human_frame.items():
        anchored_pos = np.asarray(pos, dtype=np.float64) - offset
        anchored_rot = np.asarray(rot, dtype=np.float64)
        anchored_frame[body_name] = [anchored_pos, anchored_rot]
    return anchored_frame


def play_motion(
    robot,
    smplx_frames,
    actual_human_height,
    fps,
    rate_limit=True,
    anchor_mode="none",
    freeze_root_xy=False,
    dry_run=False,
    highlight_joint_limits=False,
    joint_limit_warning_ratio=0.15,
    joint_limit_danger_ratio=0.05,
    joint_limit_show_labels=True,
):
    print(
        f"[cyan]Start[/cyan] robot={robot}, anchor_mode={anchor_mode}, "
        f"freeze_root_xy={freeze_root_xy}, frames={len(smplx_frames)}"
    )
    retargeter = GMR(
        actual_human_height=actual_human_height,
        src_human="smplx",
        tgt_robot=robot,
    )

    viewer = None
    if not dry_run:
        viewer = RobotMotionViewer(
            robot_type=robot,
            motion_fps=fps,
            transparent_robot=0,
            record_video=False,
            highlight_joint_limits=highlight_joint_limits,
            joint_limit_warning_ratio=joint_limit_warning_ratio,
            joint_limit_danger_ratio=joint_limit_danger_ratio,
            joint_limit_show_labels=joint_limit_show_labels,
        )

    root_xy_ref = None
    for frame in smplx_frames:
        anchored_frame = anchor_human_frame_to_pelvis(frame, mode=anchor_mode)
        qpos = retargeter.retarget(anchored_frame)

        if freeze_root_xy:
            if root_xy_ref is None:
                root_xy_ref = qpos[:2].copy()
            qpos[:2] = root_xy_ref

        if viewer is not None:
            viewer.step(
                root_pos=qpos[:3],
                root_rot=qpos[3:7],
                dof_pos=qpos[7:],
                human_motion_data=retargeter.scaled_human_data,
                show_human_body_name=False,
                rate_limit=rate_limit,
            )

    if viewer is not None:
        viewer.close()

    print(f"[green]Finished[/green] robot={robot}")


def main():
    here = pathlib.Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--smplx_file",
        type=str,
        required=True,
        help="SMPLX motion file.",
    )
    parser.add_argument(
        "--first_robot",
        type=str,
        default="unitree_g1",
        help="First robot to play.",
    )
    parser.add_argument(
        "--second_robot",
        type=str,
        default="jingchu03",
        help="Second robot to play.",
    )
    parser.add_argument(
        "--tgt_fps",
        type=int,
        default=30,
    )
    parser.add_argument(
        "--anchor_mode_second",
        choices=["none", "xy", "xyz"],
        default="xy",
        help="Pelvis anchoring mode used for second robot.",
    )
    parser.add_argument(
        "--freeze_root_xy_second",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Freeze XY of root for second robot.",
    )
    parser.add_argument(
        "--pause_between",
        type=float,
        default=1.0,
        help="Pause seconds between two playbacks.",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        default=False,
        help="Run retargeting without visualization.",
    )
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

    smplx_folder = here / ".." / "assets" / "body_models"
    smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
        args.smplx_file, smplx_folder
    )
    smplx_frames, aligned_fps = get_smplx_data_offline_fast(
        smplx_data, body_model, smplx_output, tgt_fps=args.tgt_fps
    )

    play_motion(
        robot=args.first_robot,
        smplx_frames=smplx_frames,
        actual_human_height=actual_human_height,
        fps=aligned_fps,
        rate_limit=not args.dry_run,
        anchor_mode="none",
        freeze_root_xy=False,
        dry_run=args.dry_run,
        highlight_joint_limits=args.highlight_joint_limits,
        joint_limit_warning_ratio=args.joint_limit_warning_ratio,
        joint_limit_danger_ratio=args.joint_limit_danger_ratio,
        joint_limit_show_labels=args.joint_limit_show_labels,
    )

    time.sleep(max(args.pause_between, 0.0))

    play_motion(
        robot=args.second_robot,
        smplx_frames=smplx_frames,
        actual_human_height=actual_human_height,
        fps=aligned_fps,
        rate_limit=not args.dry_run,
        anchor_mode=args.anchor_mode_second,
        freeze_root_xy=args.freeze_root_xy_second,
        dry_run=args.dry_run,
        highlight_joint_limits=args.highlight_joint_limits,
        joint_limit_warning_ratio=args.joint_limit_warning_ratio,
        joint_limit_danger_ratio=args.joint_limit_danger_ratio,
        joint_limit_show_labels=args.joint_limit_show_labels,
    )


if __name__ == "__main__":
    main()
