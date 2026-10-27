"""Tests for the fleet arbiter (harbot_orchard_picker.fleet_arbiter).

The arbiter coordinates two robot *processes*, so contention is tested with
a real second process: one robot runs in this test process, its peer in a
fresh subprocess with its own copy of the module state.
"""
import json
import os
import subprocess
import sys
import textwrap
import time

import pytest

from harbot_orchard_picker import fleet_arbiter as fa

SRC_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")


@pytest.fixture(autouse=True)
def isolated_lock_dir(tmp_path, monkeypatch):
    lock_dir = str(tmp_path / "locks")
    monkeypatch.setattr(fa, "_LOCK_DIR", lock_dir)
    yield lock_dir
    fa.release("robot", list(fa.held()))


def run_peer(lock_dir, body):
    """Run `body` as robot_b in a separate process; it prints JSON."""
    code = textwrap.dedent("""
        import json, sys
        sys.path.insert(0, %r)
        from harbot_orchard_picker import fleet_arbiter as fa
        fa._LOCK_DIR = %r
    """) % (SRC_DIR, lock_dir) + textwrap.dedent(body)
    out = subprocess.run([sys.executable, "-c", code], capture_output=True,
                         text=True, timeout=30, check=True)
    return json.loads(out.stdout.strip().splitlines()[-1])


# --------------------------------------------------------------------------
# reservations
# --------------------------------------------------------------------------
def test_reserve_and_release_in_one_process():
    assert fa.reserve("robot", ["P0_I2", "P0_I1"])
    assert fa.held() == {"P0_I1", "P0_I2"}
    # re-reserving what we already hold is a no-op, not a self-deadlock
    assert fa.reserve("robot", ["P0_I1"], timeout=0.1)
    fa.release("robot", ["P0_I1"])
    assert fa.held() == {"P0_I2"}


def test_peer_cannot_take_a_held_location(isolated_lock_dir):
    assert fa.reserve("robot", ["P1_I8"])
    result = run_peer(isolated_lock_dir, """
        ok = fa.reserve("robot_b", ["P1_I8"], timeout=0.3, poll_interval=0.05)
        print(json.dumps({"ok": ok, "held": sorted(fa.held())}))
    """)
    assert result == {"ok": False, "held": []}


def test_peer_gets_location_after_release(isolated_lock_dir):
    assert fa.reserve("robot", ["P1_I8"])
    fa.release("robot", ["P1_I8"])
    result = run_peer(isolated_lock_dir, """
        ok = fa.reserve("robot_b", ["P1_I8"], timeout=0.3)
        print(json.dumps({"ok": ok}))
    """)
    assert result == {"ok": True}


def test_failed_multi_reserve_rolls_back_partial_holds(isolated_lock_dir):
    # robot holds P2_I3; the peer asks for a whole segment that includes it.
    # Locks are taken in sorted order, so the peer gets P2_I1 and P2_I2,
    # then times out on P2_I3 - and must not keep the first two.
    assert fa.reserve("robot", ["P2_I3"])
    result = run_peer(isolated_lock_dir, """
        ok = fa.reserve("robot_b", ["P2_I1", "P2_I2", "P2_I3"], timeout=0.3,
                        poll_interval=0.05)
        print(json.dumps({"ok": ok, "held": sorted(fa.held())}))
    """)
    assert result == {"ok": False, "held": []}
    # and the rolled-back nodes really are free again for this robot
    assert fa.reserve("robot", ["P2_I1", "P2_I2"], timeout=0.3)


def test_locks_are_released_when_the_holder_dies(isolated_lock_dir):
    run_peer(isolated_lock_dir, """
        fa.reserve("robot_b", ["P4_I8"])
        print(json.dumps({}))
    """)  # process exits while still "holding" P4_I8
    assert fa.reserve("robot", ["P4_I8"], timeout=0.3)


def test_lock_file_records_the_holder(isolated_lock_dir):
    fa.reserve("robot", ["P3_I1"])
    with open(os.path.join(isolated_lock_dir, "P3_I1.lock")) as f:
        assert f.read() == "robot"


# --------------------------------------------------------------------------
# broadcast states
# --------------------------------------------------------------------------
def test_waiting_state_round_trip():
    assert fa.get_waiting("robot_b") == set()
    fa.set_waiting("robot_b", ["P1_I1", "P0_I1"])
    assert fa.get_waiting("robot_b") == {"P0_I1", "P1_I1"}
    fa.clear_waiting("robot_b")
    assert fa.get_waiting("robot_b") == set()


def test_finished_state_round_trip():
    assert not fa.is_finished("robot")
    fa.set_finished("robot")
    assert fa.is_finished("robot")
    fa.clear_finished("robot")
    assert not fa.is_finished("robot")


def test_maneuvering_state_expires():
    fa.set_maneuvering("robot_b", time.monotonic() + 60.0)
    assert fa.is_maneuvering("robot_b")
    fa.set_maneuvering("robot_b", time.monotonic() - 1.0)
    assert not fa.is_maneuvering("robot_b")  # stale flag must not freeze the peer
    fa.clear_maneuvering("robot_b")
    assert not fa.is_maneuvering("robot_b")


def test_clearing_absent_states_is_harmless():
    fa.clear_waiting("nobody")
    fa.clear_finished("nobody")
    fa.clear_maneuvering("nobody")
