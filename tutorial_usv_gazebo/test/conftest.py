# Copyright 2026 Honu Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Shared plumbing for the integration test modules.

The gz and ros2 suites launch a long-lived process, poll it until ready, and
audit its output; the mechanics live here once so a change to (say) the log
handling cannot drift between the two modules.
"""

import os
import shutil
import signal
import subprocess
import tempfile
import time

import pytest


def make_cli(binary, default_timeout=10):
    """Return a runner for `binary` yielding (returncode, stdout, stderr)."""
    def run(env, *args, timeout=default_timeout):
        try:
            out = subprocess.run([binary, *args], env=env, capture_output=True,
                                 text=True, timeout=timeout)
            return out.returncode, out.stdout, out.stderr
        except subprocess.TimeoutExpired:
            return -1, '', f'{binary} {args[0]}: timed out after {timeout}s'
    return run


def poll_until(condition, timeout, message, interval=2):
    """Poll `condition()` until true; fail with `message` after `timeout` s."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return
        time.sleep(interval)
    pytest.fail(message() if callable(message) else message)


def stop_process_group(proc, sig=signal.SIGTERM, grace=10):
    """
    Signal the whole process group and reap it; SIGKILL stragglers.

    gz sim and ros2 launch both spawn children, so signalling only the leader
    can orphan servers that keep running (and keep ports/partitions) after the
    test ends.
    """
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=grace)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10)


def launch_sim(request, label, cmd, env, ready, stop=stop_process_group):
    """
    Start `cmd` in its own process group, wait until `ready(env)`, return env.

    Output goes to a tempfile whose tail prints unconditionally at teardown
    (warnings matter even when every assertion passes, and exit codes
    underreport partial failures) and on startup failure. `stop(proc)` must
    take down the whole process tree.
    """
    if shutil.which(cmd[0]) is None:
        pytest.fail(f'{cmd[0]} CLI not available: the suite cannot run')
    log = tempfile.NamedTemporaryFile('w+', suffix='.log', delete=False,
                                      prefix=label.replace(' ', '_') + '_')
    proc = subprocess.Popen(cmd, env=env, start_new_session=True,
                            stdout=log, stderr=subprocess.STDOUT)

    def fail(message):
        log.flush()
        tail = ''.join(open(log.name).readlines()[-40:])
        pytest.fail(f'{message}\nlast {label} output ({log.name}):\n{tail}')

    def teardown():
        stop(proc)
        log.flush()
        tail = ''.join(open(log.name).readlines()[-60:])
        print(f'\n--- {label} output tail ({log.name}) ---\n{tail}')

    request.addfinalizer(teardown)
    deadline = time.time() + 120
    while time.time() < deadline:
        if proc.poll() is not None:
            fail(f'{label} exited during startup')
        if ready(env):
            return env
        time.sleep(2)
    fail(f'{label} never became ready')
