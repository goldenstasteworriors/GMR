import argparse
import pathlib
import pickle
import traceback

import numpy as np
from rich import print
from tqdm import tqdm

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import (
    get_smplx_data_offline_fast,
    load_smplx_file,
)


HERE = pathlib.Path(__file__).parent
SMPLX_FOLDER = HERE / ".." / "assets" / "body_models"


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

        motion_data = {
            "fps": aligned_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos,
            "local_body_pos": None,
            "link_body_list": None,
        }

        with open(tgt_file, "wb") as f:
            pickle.dump(motion_data, f)

        return "ok", smplx_file, tgt_file, f"{len(smplx_frames)} frames"
    except Exception:
        return "error", smplx_file, tgt_file, traceback.format_exc()


def main():
    parser = argparse.ArgumentParser(
        description="Batch retarget SMPL-X motions to jingchu03 without visualization."
    )
    parser.add_argument("--src_folder", type=str, required=True, help="SMPL-X root folder.")
    parser.add_argument("--tgt_folder", type=str, required=True, help="Output PKL root folder.")
    parser.add_argument("--tgt_fps", type=int, default=30, help="Target fps.")
    parser.add_argument("--override", action="store_true", help="Overwrite existing outputs.")
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

    ok_count, skip_count, err_count = 0, 0, 0
    for smplx_file in tqdm(files, desc="Retarget jingchu03"):
        status, src, dst, msg = retarget_one_file(
            smplx_file=smplx_file,
            src_folder=src_folder,
            tgt_folder=tgt_folder,
            tgt_fps=args.tgt_fps,
            override=args.override,
        )
        if status == "ok":
            ok_count += 1
        elif status == "skip":
            skip_count += 1
        else:
            err_count += 1
            print(f"[red]Error[/red] {src} -> {dst}\n{msg}")

    print(
        f"[green]Done[/green] ok={ok_count}, skip={skip_count}, error={err_count}, "
        f"output={tgt_folder}"
    )


if __name__ == "__main__":
    main()
