"""Disk stalls must not delay control-thread event submission."""
import json
import threading
import time

from controllers.hardware import hnuter_external_direct_drcda_v2_hardware as hw
from .test_drcda_v2_hardware import controller, activate


def test_slow_fsync_does_not_block_submission(monkeypatch, tmp_path):
    entered, release = threading.Event(), threading.Event()
    def slow_sync(fd):
        entered.set()
        assert release.wait(2.)
    monkeypatch.setattr(hw.os, 'fsync', slow_sync)
    writer = hw.AsyncEventWriter(tmp_path/'events.jsonl', capacity=4)
    try:
        writer.append('{"event":"first"}\n')
        assert entered.wait(1.)
        start = time.perf_counter()
        for i in range(4):
            writer.append(json.dumps({'event': i})+'\n')
        writer.append('{"event":"overflow"}\n')
        assert time.perf_counter()-start < .05
        assert writer.dropped == 1
        assert writer.pending.qsize() <= 4
    finally:
        release.set()
        assert writer.close()
    assert len((tmp_path/'events.jsonl').read_text().splitlines()) == 5


def test_disk_error_is_observable_without_raising_in_controller(tmp_path):
    writer = hw.AsyncEventWriter(tmp_path/'missing'/'events.jsonl')
    writer.append('{"event":"test"}\n')
    assert not writer.flush()
    assert writer.error
    assert not writer.close()


def test_shutdown_drains_events_and_rejects_later_writes(tmp_path):
    writer = hw.AsyncEventWriter(tmp_path/'events.jsonl')
    for i in range(20):
        writer.append(json.dumps({'i': i})+'\n')
    assert writer.close()
    assert [json.loads(x)['i'] for x in writer.path.read_text().splitlines()] == list(range(20))
    writer.append('{}\n')
    assert writer.dropped == 1


def test_handover_publishes_while_event_disk_is_blocked(controller, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def slow_sync(fd):
        entered.set()
        assert release.wait(2.)
    monkeypatch.setattr(hw.os, 'fsync', slow_sync)
    try:
        start = time.perf_counter()
        activate(controller)
        assert time.perf_counter()-start < .05
        assert entered.wait(1.)
        assert controller._hardware_control_active
        assert not controller._hardware_fault
        controller.actuator_motors_pub.publish.assert_called()
    finally:
        release.set()
