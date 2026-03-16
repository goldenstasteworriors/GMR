import os
import time
import mujoco as mj
import mujoco.viewer as mjv
import imageio
from scipy.spatial.transform import Rotation as R
from general_motion_retargeting import ROBOT_XML_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT
from loop_rate_limiters import RateLimiter
import numpy as np
from rich import print


def draw_frame(
    pos,
    mat,
    v,
    size,
    joint_name=None,
    orientation_correction=R.from_euler("xyz", [0, 0, 0]),
    pos_offset=np.array([0, 0, 0]),
):
    rgba_list = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
    for i in range(3):
        geom = v.user_scn.geoms[v.user_scn.ngeom]
        mj.mjv_initGeom(
            geom,
            type=mj.mjtGeom.mjGEOM_ARROW,
            size=[0.01, 0.01, 0.01],
            pos=pos + pos_offset,
            mat=mat.flatten(),
            rgba=rgba_list[i],
        )
        if joint_name is not None:
            geom.label = joint_name  # 这里赋名字
        fix = orientation_correction.as_matrix()
        mj.mjv_connector(
            v.user_scn.geoms[v.user_scn.ngeom],
            type=mj.mjtGeom.mjGEOM_ARROW,
            width=0.005,
            from_=pos + pos_offset,
            to=pos + pos_offset + size * (mat @ fix)[:, i],
        )
        v.user_scn.ngeom += 1


def draw_sphere_marker(pos, v, size, rgba, label=None):
    geom = v.user_scn.geoms[v.user_scn.ngeom]
    mj.mjv_initGeom(
        geom,
        type=mj.mjtGeom.mjGEOM_SPHERE,
        size=[size, size, size],
        pos=pos,
        mat=np.eye(3).flatten(),
        rgba=rgba,
    )
    if label is not None:
        geom.label = label
    v.user_scn.ngeom += 1

class RobotMotionViewer:
    def __init__(self,
                robot_type,
                camera_follow=True,
                motion_fps=30,
                transparent_robot=0,
                # video recording
                record_video=False,
                video_path=None,
                video_width=640,
                video_height=480,
                keyboard_callback=None,
                highlight_joint_limits=False,
                joint_limit_warning_ratio=0.15,
                joint_limit_danger_ratio=0.05,
                joint_limit_show_labels=True,
                joint_limit_marker_size=0.03,
                ):
        
        self.robot_type = robot_type
        self.xml_path = ROBOT_XML_DICT[robot_type]
        self.model = mj.MjModel.from_xml_path(str(self.xml_path))
        self.data = mj.MjData(self.model)
        self.robot_base = ROBOT_BASE_DICT[robot_type]
        self.viewer_cam_distance = VIEWER_CAM_DISTANCE_DICT[robot_type]
        self.default_geom_rgba = self.model.geom_rgba.copy()
        mj.mj_step(self.model, self.data)
        
        self.motion_fps = motion_fps
        self.rate_limiter = RateLimiter(frequency=self.motion_fps, warn=False)
        self.camera_follow = camera_follow
        self.record_video = record_video
        self.highlight_joint_limits = highlight_joint_limits
        self.joint_limit_warning_ratio = joint_limit_warning_ratio
        self.joint_limit_danger_ratio = joint_limit_danger_ratio
        self.joint_limit_show_labels = joint_limit_show_labels
        self.joint_limit_marker_size = joint_limit_marker_size
        self.body_to_geom_ids = {}
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            self.body_to_geom_ids.setdefault(body_id, []).append(geom_id)
        self.limited_joint_infos = self._build_limited_joint_infos()


        self.viewer = mjv.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False, 
            key_callback=keyboard_callback
            )      

        self.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = transparent_robot
        
        if self.record_video:
            assert video_path is not None, "Please provide video path for recording"
            self.video_path = video_path
            video_dir = os.path.dirname(self.video_path)
            
            if not os.path.exists(video_dir):
                os.makedirs(video_dir)
            self.mp4_writer = imageio.get_writer(self.video_path, fps=self.motion_fps)
            print(f"Recording video to {self.video_path}")
            
            # Initialize renderer for video recording
            self.renderer = mj.Renderer(self.model, height=video_height, width=video_width)

    def _build_limited_joint_infos(self):
        joint_infos = []
        for joint_id in range(self.model.njnt):
            if not bool(self.model.jnt_limited[joint_id]):
                continue
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            next_qpos_adr = (
                int(self.model.jnt_qposadr[joint_id + 1])
                if joint_id + 1 < self.model.njnt
                else int(self.model.nq)
            )
            if next_qpos_adr - qpos_adr != 1:
                continue
            body_id = int(self.model.jnt_bodyid[joint_id])
            joint_infos.append(
                {
                    "joint_id": joint_id,
                    "joint_name": mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, joint_id),
                    "qpos_adr": qpos_adr,
                    "lower": float(self.model.jnt_range[joint_id][0]),
                    "upper": float(self.model.jnt_range[joint_id][1]),
                    "body_id": body_id,
                    "geom_ids": list(self.body_to_geom_ids.get(body_id, [])),
                }
            )
        return joint_infos

    def _joint_limit_state(self, qpos_value, lower, upper):
        joint_range = upper - lower
        if joint_range <= 0:
            return None
        distance_to_limit = min(qpos_value - lower, upper - qpos_value)
        danger_margin = joint_range * self.joint_limit_danger_ratio
        warning_margin = joint_range * self.joint_limit_warning_ratio
        if distance_to_limit <= danger_margin:
            return "danger"
        if distance_to_limit <= warning_margin:
            return "warning"
        return None

    def _joint_marker_position(self, joint_id, body_id):
        if hasattr(self.data, "xanchor"):
            return np.array(self.data.xanchor[joint_id], dtype=np.float64)
        return np.array(self.data.xpos[body_id], dtype=np.float64)

    def _apply_joint_limit_highlights(self):
        self.model.geom_rgba[:] = self.default_geom_rgba
        active_joint_infos = []
        for joint_info in self.limited_joint_infos:
            qpos_value = float(self.data.qpos[joint_info["qpos_adr"]])
            state = self._joint_limit_state(
                qpos_value,
                joint_info["lower"],
                joint_info["upper"],
            )
            if state is None:
                continue
            if state == "danger":
                rgba = np.array([1.0, 0.15, 0.15, 1.0], dtype=np.float32)
            else:
                rgba = np.array([1.0, 0.65, 0.0, 1.0], dtype=np.float32)
            for geom_id in joint_info["geom_ids"]:
                self.model.geom_rgba[geom_id] = rgba
            active_joint_infos.append((joint_info, state, rgba, qpos_value))
        return active_joint_infos
        
    def step(self, 
            # robot data
            root_pos, root_rot, dof_pos, 
            # human data
            human_motion_data=None, 
            show_human_body_name=False,
            # scale for human point visualization
            human_point_scale=0.1,
            # human pos offset add for visualization    
            human_pos_offset=np.array([0.0, 0.0, 0]),
            # rate limit
            rate_limit=True, 
            follow_camera=True,
            ):
        """
        by default visualize robot motion.
        also support visualize human motion by providing human_motion_data, to compare with robot motion.
        
        human_motion_data is a dict of {"human body name": (3d global translation, 3d global rotation)}.

        if rate_limit is True, the motion will be visualized at the same rate as the motion data.
        else, the motion will be visualized as fast as possible.
        """
        
        self.data.qpos[:3] = root_pos
        self.data.qpos[3:7] = root_rot # quat need to be scalar first! for mujoco
        self.data.qpos[7:] = dof_pos
        
        mj.mj_forward(self.model, self.data)
        if self.highlight_joint_limits:
            active_joint_infos = self._apply_joint_limit_highlights()
        else:
            self.model.geom_rgba[:] = self.default_geom_rgba
            active_joint_infos = []
        
        if follow_camera:
            self.viewer.cam.lookat = self.data.xpos[self.model.body(self.robot_base).id]
            self.viewer.cam.distance = self.viewer_cam_distance
            self.viewer.cam.elevation = -10  # 正面视角，轻微向下看
            # self.viewer.cam.azimuth = 180    # 正面朝向机器人
        
        if human_motion_data is not None or self.highlight_joint_limits:
            # Clean custom geometry
            self.viewer.user_scn.ngeom = 0

        if human_motion_data is not None:
            # Draw the task targets for reference
            for human_body_name, (pos, rot) in human_motion_data.items():
                draw_frame(
                    pos,
                    R.from_quat(rot, scalar_first=True).as_matrix(),
                    self.viewer,
                    human_point_scale,
                    pos_offset=human_pos_offset,
                    joint_name=human_body_name if show_human_body_name else None
                    )

        if self.highlight_joint_limits:
            for joint_info, state, rgba, qpos_value in active_joint_infos:
                label = None
                if self.joint_limit_show_labels:
                    label = f"{joint_info['joint_name']}:{qpos_value:.2f}"
                draw_sphere_marker(
                    self._joint_marker_position(joint_info["joint_id"], joint_info["body_id"]),
                    self.viewer,
                    self.joint_limit_marker_size * (1.2 if state == "danger" else 1.0),
                    rgba,
                    label=label,
                )

        self.viewer.sync()
        if rate_limit is True:
            self.rate_limiter.sleep()

        if self.record_video:
            # Use renderer for proper offscreen rendering
            self.renderer.update_scene(self.data, camera=self.viewer.cam)
            img = self.renderer.render()
            self.mp4_writer.append_data(img)
    
    def close(self):
        self.viewer.close()
        time.sleep(0.5)
        if self.record_video:
            self.mp4_writer.close()
            print(f"Video saved to {self.video_path}")
