"""Exact Phase 2 protocol test selection."""
import json
from pathlib import Path

import pytest


def pytest_addoption(parser):
    parser.addoption("--case", help="Exact Phase 2 test name")
    parser.addoption("--manifest", type=Path, help="Frozen Phase 2 run manifest")


def pytest_configure(config):
    config.addinivalue_line("markers", "p2_case(name): exact Phase 2 test name")


def pytest_collection_modifyitems(config, items):
    case = config.getoption("case", default=None)
    manifest = config.getoption("manifest", default=None)
    if case is None:
        return
    if manifest is None or not manifest.is_file():
        raise pytest.UsageError("--case requires an existing --manifest")
    selected = [
        item for item in items
        if any(mark.args == (case,) for mark in item.iter_markers("p2_case"))
    ]
    if not selected:
        raise pytest.UsageError(f"unknown or excluded Phase 2 case: {case}")
    config.hook.pytest_deselected(items=[item for item in items if item not in selected])
    items[:] = selected


@pytest.fixture
def p2_manifest(request):
    path = request.config.getoption("manifest", default=None)
    if path is None:
        pytest.skip("a frozen Phase 2 manifest is required")
    return json.loads(path.read_text(encoding="utf-8"))
