import argparse
import multiprocessing as mp
import pathlib
import pickle
import traceback

import numpy as np
import torch
from rich import print
from tqdm import tqdm

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.kinematics_model import KinematicsModel
from general_motion_retargeting.utils.smpl import (
    get_smplx_data_offline_fast,
    load_smplx_file,
)


HERE = pathlib.Path(__file__).parent
SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"
_KINEMATICS_MODEL_CACHE = {}


def build_local_body_pos(retargeter: GMR, dof_pos: np.ndarray):
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    cache_key = (retargeter.xml_file, device)
    kinematics_model = _KINEMATICS_MODEL_CACHE.get(cache_key)
    if kinematics_model is None:
        kinematics_model = KinematicsModel(retargeter.xml_file, device=device)
        _KINEMATICS_MODEL_CACHE[cache_key] = kinematics_model
    num_frames = dof_pos.shape[0]

    fk_root_pos = torch.zeros((num_frames, 3), device=device)
    fk_root_rot = torch.zeros((num_frames, 4), device=device)
    fk_root_rot[:, -1] = 1.0

    dof_pos_tensor = torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
    local_body_pos, _ = kinematics_model.forward_kinematics(
        fk_root_pos,
        fk_root_rot,
        dof_pos_tensor,
    )
    return local_body_pos.detach().cpu().numpy(), kinematics_model.body_names


def collect_smplx_files(src_folder: pathlib.Path):
    files = []
    for p in sorted(src_folder.rglob("*.npz")):
        if p.name.endswith("_stagei.npz"):
            continue
        files.append(p)
    return files


def retarget_one_file(
    smplx_file: pathlib.Path,
    src_folder: pathlib.Path,
    tgt_folder: pathlib.Path,
    tgt_fps: int,
    override: bool,
):
    tgt_file = (tgt_folder / smplx_file.relative_to(src_folder)).with_suffix(".pkl")
    tgt_file.parent.mkdir(parents=True, exist_ok=True)

    if tgt_file.exists() and not override:
        return "skip", smplx_file, tgt_file, "exists"

    try:
        smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
            str(smplx_file), SMPLX_FOLDER
        )
        smplx_frames, aligned_fps = get_smplx_data_offline_fast(
            smplx_data, body_model, smplx_output, tgt_fps=tgt_fps
        )

        retargeter = GMR(
            actual_human_height=actual_human_height,
            src_human="smplx",
            tgt_robot="jingchu03",
            verbose=False,
        )

        qpos_list = []
        for frame in smplx_frames:
            qpos = retargeter.retarget(frame)
            qpos_list.append(qpos.copy())

        qpos_arr = np.asarray(qpos_list)
        root_pos = qpos_arr[:, :3]
        root_rot = qpos_arr[:, 3:7][:, [1, 2, 3, 0]]  # wxyz -> xyzw
        dof_pos = qpos_arr[:, 7:]
        local_body_pos, body_names = build_local_body_pos(retargeter, dof_pos)

        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": local_body_pos,
            "link_body_list": body_names,
        }

        with open(tgt_file, "wb") as f:
            pickle.dump(motion_data, f)

        return "ok", smplx_file, tgt_file, f"{len(smplx_frames)} frames"
    except Exception:
        return "error", smplx_file, tgt_file, traceback.format_exc()


def retarget_one_file_worker(task):
    return retarget_one_file(*task)


def main():
    parser = argparse.ArgumentParser(
        description="Batch retarget SMPL-X motions to jingchu03 without visualization."
    )
    parser.add_argument("--src_folder", type=str, required=True, help="SMPL-X root folder.")
    parser.add_argument("--tgt_folder", type=str, required=True, help="Output PKL root folder.")
    parser.add_argument("--tgt_fps", type=int, default=30, help="Target fps.")
    parser.add_argument("--override", action="store_true", help="Overwrite existing outputs.")
    parser.add_argument(
        "--num_cpus",
        type=int,
        default=min(4, mp.cpu_count()),
        help="Number of worker processes for file-level parallel retargeting.",
    )
    parser.add_argument(
        "--max_files",
        type=int,
        default=None,
        help="Only process first N files for quick testing.",
    )
    args = parser.parse_args()

    src_folder = pathlib.Path(args.src_folder).expanduser().resolve()
    tgt_folder = pathlib.Path(args.tgt_folder).expanduser().resolve()

    files = collect_smplx_files(src_folder)
    if args.max_files is not None:
        files = files[: args.max_files]

    print(f"[cyan]Found {len(files)} files[/cyan] under {src_folder}")
    if len(files) == 0:
        return
    print(f"[cyan]Using {args.num_cpus} worker process(es)[/cyan]")

    ok_count, skip_count, err_count = 0, 0, 0
    tasks = [
        (smplx_file, src_folder, tgt_folder, args.tgt_fps, args.override)
        for smplx_file in files
    ]

    if args.num_cpus <= 1:
        result_iter = (
            retarget_one_file_worker(task)
            for task in tqdm(tasks, desc="Retarget jingchu03")
        )
    else:
        ctx = mp.get_context("spawn")
        pool = ctx.Pool(processes=args.num_cpus, maxtasksperchild=1)
        result_iter = tqdm(
            pool.imap_unordered(retarget_one_file_worker, tasks),
            total=len(tasks),
            desc="Retarget jingchu03",
        )

    try:
        for status, src, dst, msg in result_iter:
            if status == "ok":
                ok_count += 1
            elif status == "skip":
                skip_count += 1
            else:
                err_count += 1
                print(f"[red]Error[/red] {src} -> {dst}\n{msg}")
    finally:
        if args.num_cpus > 1:
            pool.close()
            pool.join()

    print(
        f"[green]Done[/green] ok={ok_count}, skip={skip_count}, error={err_count}, "
        f"output={tgt_folder}"
    )


if __name__ == "__main__":
    main()
