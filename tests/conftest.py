'''shared test setup'''

import os

import pytest


@pytest.fixture(autouse=True)
def restore_mavlink20():
    """put MAVLINK20 back as it was after each test.  MAVExplorer.py sets it
    when it is loaded, and pymavlink's mission loader reads it to choose the
    messages it builds, but the dialect pymavlink has already loaded does not
    change with it: tests which run after one loading MAVExplorer.py would
    otherwise build messages the loaded dialect cannot take"""
    before = os.environ.get('MAVLINK20')
    yield
    if before is None:
        os.environ.pop('MAVLINK20', None)
    else:
        os.environ['MAVLINK20'] = before
