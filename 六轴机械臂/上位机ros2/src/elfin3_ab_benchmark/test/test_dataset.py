import copy

import pytest
import yaml

from elfin3_ab_benchmark.dataset import load_dataset


def valid_document():
    return {
        "base_frame": "elfin_base",
        "tool_frame": "elfin_end_link",
        "y_safe_min_m": 0.2,
        "y_safety_margin_m": 0.05,
        "workspace_fk_sample_max_joint_step_rad": 0.02,
        "execution_mode": "simulation",
        "allow_real_motion": False,
        "velocity_scaling": 0.1,
        "acceleration_scaling": 0.1,
        "repetitions": 3,
        "auto_reset_to_start": True,
        "reset_algorithm": "move_pose",
        "start_pose": {
            "frame_id": "elfin_base",
            "position": [0.1, 0.4, 0.3],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "targets": [
            {
                "id": "target_01",
                "category": "ordinary",
                "pose": {
                    "frame_id": "elfin_base",
                    "position": [0.2, 0.5, 0.4],
                    "orientation": [0.0, 0.0, 0.0, 2.0],
                },
            }
        ],
    }


def write_dataset(tmp_path, document):
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return str(path)


def test_loads_valid_dataset_and_normalizes_quaternion(tmp_path):
    config = load_dataset(write_dataset(tmp_path, valid_document()))
    assert config.effective_y_min_m == pytest.approx(0.25)
    assert config.targets[0].pose.orientation[3] == pytest.approx(1.0)
    assert config.repetitions == 3


def test_requires_explicit_positive_y_boundary(tmp_path):
    document = valid_document()
    document["y_safe_min_m"] = None
    with pytest.raises(ValueError, match="y_safe_min_m"):
        load_dataset(write_dataset(tmp_path, document))


def test_rejects_target_below_effective_y_boundary(tmp_path):
    document = valid_document()
    document["targets"][0]["pose"]["position"][1] = 0.24
    with pytest.raises(ValueError, match="effective_y_min_m"):
        load_dataset(write_dataset(tmp_path, document))


def test_real_mode_requires_explicit_authorization(tmp_path):
    document = valid_document()
    document["execution_mode"] = "real"
    with pytest.raises(ValueError, match="allow_real_motion"):
        load_dataset(write_dataset(tmp_path, document))


def test_real_mode_accepts_confirmed_scaling_limit(tmp_path):
    document = valid_document()
    document["execution_mode"] = "real"
    document["allow_real_motion"] = True
    document["velocity_scaling"] = 0.2
    document["acceleration_scaling"] = 0.2
    config = load_dataset(write_dataset(tmp_path, document))
    assert config.velocity_scaling == pytest.approx(0.2)
    assert config.acceleration_scaling == pytest.approx(0.2)


def test_real_mode_rejects_scaling_above_confirmed_limit(tmp_path):
    document = valid_document()
    document["execution_mode"] = "real"
    document["allow_real_motion"] = True
    document["velocity_scaling"] = 0.200001
    with pytest.raises(ValueError, match="<= 0.2"):
        load_dataset(write_dataset(tmp_path, document))


def test_accepts_move_j_reset_algorithm(tmp_path):
    document = copy.deepcopy(valid_document())
    document["reset_algorithm"] = "move_j"
    config = load_dataset(write_dataset(tmp_path, document))
    assert config.reset_algorithm == "move_j"


def test_rejects_unknown_reset_algorithm(tmp_path):
    document = copy.deepcopy(valid_document())
    document["reset_algorithm"] = "cartesian_magic"
    with pytest.raises(ValueError, match="reset_algorithm"):
        load_dataset(write_dataset(tmp_path, document))


def test_rejects_motion_span_tolerance_not_smaller_than_q0_limit(tmp_path):
    document = copy.deepcopy(valid_document())
    document["q0_tolerance_rad"] = 0.01
    document["q0_motion_span_tolerance_rad"] = 0.01
    with pytest.raises(ValueError, match="q0_motion_span_tolerance_rad"):
        load_dataset(write_dataset(tmp_path, document))
