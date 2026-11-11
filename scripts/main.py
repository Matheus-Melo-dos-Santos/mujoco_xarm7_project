"""Demonstração de juntas individuais e movimento coordenado do xArm7."""

from __future__ import annotations

import sys
import time

import numpy as np
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "scene" / "inspection_cell.xml"

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
ACTUATOR_NAMES = [f"act{i}" for i in range(1, 8)]

# Poses articulares ilustrativas (rad), dentro das faixas declaradas pelo modelo.
# Não são soluções de cinemática inversa nem garantem alcançar coordenadas cartesianas.
POSE_AREA_A = [0.00, -0.65, 0.00, 1.55, 0.00, 1.20, 0.00]
POSE_AREA_B = [0.55, -0.45, -0.55, 1.75, 0.35, 1.35, -0.35]


def object_id(model: mujoco.MjModel, object_type, name: str) -> int:
    idx = mujoco.mj_name2id(model, object_type, name)
    if idx < 0:
        raise RuntimeError(f"Elemento '{name}' não encontrado no modelo.")
    return idx


def load_model():
    if not SCENE_PATH.exists():
        raise FileNotFoundError(f"Cena não encontrada: {SCENE_PATH}")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)

    joint_ids = [
        object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in JOINT_NAMES
    ]
    actuator_ids = [
        object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        for name in ACTUATOR_NAMES
    ]
    gripper_id = object_id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "gripper"
    )

    for name, jid, aid in zip(JOINT_NAMES, joint_ids, actuator_ids):
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"{name} não é uma junta rotacional (hinge).")
        if not model.jnt_limited[jid]:
            raise RuntimeError(f"{name} não possui limites articulares definidos.")
        if model.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT:
            raise RuntimeError(f"{ACTUATOR_NAMES[aid]} não atua diretamente sobre uma junta.")

    # return model, data, joint_ids, actuator_ids
    return model, data, joint_ids, actuator_ids, gripper_id


def print_joint_table(model, joint_ids, actuator_ids):
    print(f"MuJoCo: {mujoco.__version__}")
    print("Juntas e limites lidos do modelo compilado:")
    for name, jid, aid in zip(JOINT_NAMES, joint_ids, actuator_ids):
        low, high = model.jnt_range[jid]
        ctrl_low, ctrl_high = model.actuator_ctrlrange[aid]
        print(
            f"  {name:7s} | junta [{low: .5f}, {high: .5f}] rad"
            f" | atuador {ACTUATOR_NAMES[JOINT_NAMES.index(name)]}"
            f" [{ctrl_low: .5f}, {ctrl_high: .5f}]"
        )


def step_toward(model, data, viewer, actuator_ids, target, duration=1.5):
    """Interpola os sete comandos em conjunto e avança a física."""
    start = data.ctrl[actuator_ids].copy()
    steps = max(1, int(duration / model.opt.timestep))
    for k in range(steps):
        if not viewer.is_running():
            return
        u = (k + 1) / steps
        # Interpolação smoothstep para evitar degraus bruscos no comando.
        blend = u * u * (3.0 - 2.0 * u)
        data.ctrl[actuator_ids] = start + blend * (target - start)
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)


def get_home(model, data, actuator_ids):
    key_id = object_id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    return data.ctrl[actuator_ids].copy()


def validate_pose(model, actuator_ids, pose, label):
    pose_array = np.asarray(pose, dtype=float)
    if pose_array.shape != (7,):
        raise ValueError(f"{label} deve conter sete posições.")
    for i, aid in enumerate(actuator_ids):
        lo, hi = model.actuator_ctrlrange[aid]
        if not lo <= pose_array[i] <= hi:
            raise ValueError(
                f"{label}: comando da junta {i + 1} ({pose_array[i]:.3f}) "
                f"fora do limite do atuador [{lo:.3f}, {hi:.3f}]."
            )
    return pose_array


def run_demo(model, data, viewer, joint_ids, actuator_ids, gripper_id):
    home = get_home(model, data, actuator_ids)
    ranges = [model.jnt_range[jid].copy() for jid in joint_ids]

    print("\n[1/2] Movimentação individual das sete juntas")
    data.ctrl[actuator_ids] = home
    for index, (jid, aid, limits) in enumerate(zip(joint_ids, actuator_ids, ranges), start=1):
        low, high = limits
        # Usa pontos internos da faixa, evitando os batentes mecânicos.
        target_low = low + 0.35 * (high - low)
        target_high = low + 0.45* (high - low)

        command = home.copy()
        command[index - 1] = target_low
        print(f"  Junta {index}: posição inferior de demonstração ({target_low:.3f} rad)")
        step_toward(model, data, viewer, actuator_ids, command, duration=1.0)

        command[index - 1] = target_high
        print(f"  Junta {index}: posição superior de demonstração ({target_high:.3f} rad)")
        step_toward(model, data, viewer, actuator_ids, command, duration=1.0)

        command[index - 1] = home[index - 1]
        step_toward(model, data, viewer, actuator_ids, command, duration=0.8)

    print("\n[2/2] Movimento coordenado entre as áreas de trabalho")
    pose_a = validate_pose(model, actuator_ids, POSE_AREA_A, "POSE_AREA_A")
    pose_b = validate_pose(model, actuator_ids, POSE_AREA_B, "POSE_AREA_B")
    print("  Repouso -> área A")
    step_toward(model, data, viewer, actuator_ids, pose_a, duration=2.0)
    print("  Área A -> área B")
    step_toward(model, data, viewer, actuator_ids, pose_b, duration=2.0)
    print("  Área B -> repouso")
    step_toward(model, data, viewer, actuator_ids, home, duration=2.0)

    print("\nDemonstração concluída. O viewer permanecerá aberto até ser fechado.")

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)

def main():
    try:
        model, data, joint_ids, actuator_ids, gripper_id = load_model()
        print_joint_table(model, joint_ids, actuator_ids)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            run_demo(model, data, viewer, joint_ids, actuator_ids, gripper_id)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        print(
            "Confira se executou ./setup_model.sh e instalou requirements.txt.",
            file=sys.stderr,
        )
        raise SystemExit(1)


if __name__ == "__main__":
    main()
