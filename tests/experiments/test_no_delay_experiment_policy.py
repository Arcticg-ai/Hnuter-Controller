from pathlib import Path

from tools.experiments import run_no_delay_maneuver_experiments as runner


def _firmware_tree(tmp_path: Path, model_text: str) -> Path:
    firmware = tmp_path / 'firmware'
    model = firmware / 'Tools/simulation/gz/models/hnuter/model.sdf'
    model.parent.mkdir(parents=True)
    model.write_text(model_text, encoding='utf-8')
    return firmware


def test_default_validation_firmware_is_no_delay_worktree():
    assert runner.DEFAULT_FIRMWARE.name == 'PX4-Autopilot-Hnuter-tail-sitl'
    assert runner.VALIDATION_POLICY == 'no_delay_only'


def test_firmware_metadata_accepts_no_delay_model(tmp_path, monkeypatch):
    firmware = _firmware_tree(tmp_path, '<sdf><model name="hnuter"/></sdf>')
    monkeypatch.setattr(runner, 'git_value', lambda *_args: 'test-value')

    metadata = runner.firmware_metadata(firmware)

    assert metadata['validation_policy'] == 'no_delay_only'
    assert metadata['forbidden_actuator_tokens'] == []
    assert metadata['dynamic_actuator_tokens_present'] is False


def test_firmware_metadata_detects_forbidden_delay_plugin(tmp_path, monkeypatch):
    firmware = _firmware_tree(
        tmp_path,
        '<plugin name="servo_0_dynamic">transport_delay</plugin>',
    )
    monkeypatch.setattr(runner, 'git_value', lambda *_args: 'test-value')

    metadata = runner.firmware_metadata(firmware)

    assert metadata['dynamic_actuator_tokens_present'] is True
    assert metadata['forbidden_actuator_tokens'] == [
        'servo_0_dynamic',
        'transport_delay',
    ]
