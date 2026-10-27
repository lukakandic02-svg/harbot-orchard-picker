"""Fleet arbiter: shared state between the two robots (coordination layer 3).

Graph node reservation on top of the distance-based proximity hold. The
proximity hold alone lets the priority robot drive on while the other one
waits, so reservations are symmetric: both robots always hold the nodes
they occupy and wait for nodes held by the other robot.

Also stores three small broadcast states each robot publishes for its peer:
*waiting for* (nodes it is blocked on), *maneuvering* (in a bypass arc,
please hold) and *finished* (parked for good).

No ROS dependency, so it can be unit-tested. Uses one fcntl.flock file per
location under /tmp/harbot_fleet_locks/. Locks belong to the process's file
descriptor and are released when the process exits.
"""

import fcntl
import os
import time

_LOCK_DIR = "/tmp/harbot_fleet_locks"

# Locations held by this process: {loc: fd}. Reserving one of them again is a no-op.
_held = {}


def _lock_path(loc):
    os.makedirs(_LOCK_DIR, exist_ok=True)
    return os.path.join(_LOCK_DIR, "%s.lock" % loc)


def _release_one(loc):
    fd = _held.pop(loc, None)
    if fd is not None:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


def reserve(robot_id, locs, timeout=30.0, poll_interval=0.2):
    """Try to acquire exclusive holds on every location in `locs`.

    Locations already held by this process are skipped. The rest are acquired
    in sorted order (same order for both robots), so two multi-location
    reservations cannot deadlock.

    Returns True once all are held. On timeout returns False and releases
    whatever this call acquired; earlier holds are kept.
    """
    to_acquire = sorted(set(locs) - set(_held))
    if not to_acquire:
        return True

    acquired_this_call = []
    deadline = time.monotonic() + timeout
    for loc in to_acquire:
        path = _lock_path(loc)
        while True:
            fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(fd)
                if time.monotonic() >= deadline:
                    for acquired_loc in acquired_this_call:
                        _release_one(acquired_loc)
                    return False
                time.sleep(poll_interval)
                continue
            # Record the holder, for debugging.
            try:
                os.ftruncate(fd, 0)
                os.write(fd, str(robot_id).encode())
            except OSError:
                pass
            _held[loc] = fd
            acquired_this_call.append(loc)
            break

    return True


def release(robot_id, locs):
    """Release the given locations held by this process."""
    for loc in locs:
        _release_one(loc)


def held():
    """All locations this process currently holds."""
    return set(_held)


def _waiting_path(robot_id):
    os.makedirs(_LOCK_DIR, exist_ok=True)
    return os.path.join(_LOCK_DIR, "%s.waiting_for" % robot_id)


def set_waiting(robot_id, locs):
    """Publish that `robot_id` is blocked waiting for `locs`.

    Used by check_symmetric_deadlock() to tell a mutual deadlock from one-sided
    congestion. Best effort: OSError is ignored.
    """
    try:
        path = _waiting_path(robot_id)
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            f.write(",".join(sorted(locs)))
        os.replace(tmp_path, path)
    except OSError:
        pass


def clear_waiting(robot_id):
    """Clear the waiting state of `robot_id`."""
    try:
        os.remove(_waiting_path(robot_id))
    except OSError:
        pass


def get_waiting(robot_id):
    """Return the set of locations the peer `robot_id` is waiting for (empty if none)."""
    try:
        with open(_waiting_path(robot_id)) as f:
            content = f.read().strip()
    except OSError:
        return set()
    if not content:
        return set()
    return set(content.split(","))


def _finished_path(robot_id):
    os.makedirs(_LOCK_DIR, exist_ok=True)
    return os.path.join(_LOCK_DIR, "%s.finished" % robot_id)


def set_finished(robot_id):
    """Publish that `robot_id` has finished its mission and will not move again.

    Permanent for the rest of the mission. Best effort: OSError is ignored.
    """
    try:
        path = _finished_path(robot_id)
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            f.write("1")
        os.replace(tmp_path, path)
    except OSError:
        pass


def is_finished(robot_id):
    """True if the peer `robot_id` has published that it is finished."""
    return os.path.exists(_finished_path(robot_id))


def clear_finished(robot_id):
    """Clear this robot's finished flag (on startup, in case a previous run was killed)."""
    try:
        os.remove(_finished_path(robot_id))
    except OSError:
        pass


def _maneuvering_path(robot_id):
    os.makedirs(_LOCK_DIR, exist_ok=True)
    return os.path.join(_LOCK_DIR, "%s.maneuvering" % robot_id)


def set_maneuvering(robot_id, expires_at):
    """Publish that `robot_id` is in a bypass-arc maneuver until `expires_at`.

    The peer holds position, regardless of priority, until the expiry
    (time.monotonic()) or clear_maneuvering().
    """
    try:
        path = _maneuvering_path(robot_id)
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            f.write(str(expires_at))
        os.replace(tmp_path, path)
    except OSError:
        pass


def clear_maneuvering(robot_id):
    """Clear the maneuvering state of `robot_id`."""
    try:
        os.remove(_maneuvering_path(robot_id))
    except OSError:
        pass


def is_maneuvering(robot_id):
    """True if the peer `robot_id` is maneuvering and the announcement has not expired."""
    try:
        with open(_maneuvering_path(robot_id)) as f:
            content = f.read().strip()
        expires_at = float(content)
    except (OSError, ValueError):
        return False
    return time.monotonic() < expires_at
