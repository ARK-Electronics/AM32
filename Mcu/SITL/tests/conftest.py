'''pytest fixtures for AM32 SITL CI tests.'''

from __future__ import annotations

import os
import sys
import tempfile

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL_DIR = os.path.normpath(os.path.join(HERE, '..'))
sys.path.insert(0, SITL_DIR)

from sitl_harness import (  # noqa: E402
    Sitl,
    SitlStartError,
    find_sitl_binary,
    free_mcast_group,
    open_state,
)


def pytest_addoption(parser):
    parser.addoption(
        '--sitl',
        action='store',
        default=None,
        help='path to AM32 SITL binary (default: newest obj/AM32_AM32_SITL_CAN_*.elf)',
    )


@pytest.fixture(scope='session')
def sitl_path(request):
    path = find_sitl_binary(request.config.getoption('--sitl'))
    if not path or not os.path.exists(path):
        pytest.fail(
            'SITL binary not found. Build with: make AM32_SITL_CAN\n'
            'or pass --sitl /path/to/elf')
    return path


@pytest.fixture
def workdir(tmp_path):
    '''isolated cwd so eeprom files and logs do not collide'''
    d = tempfile.mkdtemp(prefix='sitl_ci_', dir=str(tmp_path))
    cwd = os.getcwd()
    os.chdir(d)
    try:
        yield d
    finally:
        os.chdir(cwd)


@pytest.fixture
def sitl_factory(sitl_path, workdir):
    '''factory: sitl_factory(extra_args=..., can_uri=..., **kw) -> Sitl'''
    instances = []

    def _make(extra_args=(), can_uri='none', **kw):
        s = Sitl(sitl_path, extra_args=extra_args, workdir=workdir,
                 can_uri=can_uri, **kw)
        instances.append(s)
        return s

    yield _make
    for s in instances:
        s.close()


@pytest.fixture
def mcast_uri():
    return 'mcast:%d' % free_mcast_group()


@pytest.fixture
def sitl_can_factory(sitl_factory):
    '''like sitl_factory, but skip the test if multicast CAN cannot start.

    GitHub macOS runners historically lack a usable multicast route; the
    SITL either dies during CAN init or never becomes reachable. Match
    upstream behaviour: skip rather than fail the job.
    '''

    def _make(extra_args=(), can_uri=None, **kw):
        if can_uri is None:
            raise TypeError('sitl_can_factory requires can_uri')
        try:
            return sitl_factory(extra_args=extra_args, can_uri=can_uri, **kw)
        except SitlStartError as e:
            if e.looks_like_mcast_failure:
                pytest.skip('SITL multicast CAN unavailable on this host:\n%s'
                            % e)
            raise

    return _make


@pytest.fixture
def state_stream():
    streams = []

    def _open(sitl, period_us=200):
        sim = open_state('127.0.0.1', sitl.state_port, period_us=period_us)
        streams.append(sim)
        return sim

    yield _open
    for s in streams:
        s.close()
