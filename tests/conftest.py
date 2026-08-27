"""Shared pytest configuration for the needlemanipulation test suite.

The package ``__init__`` imports the compiled ``_core`` extension (and the
``marinholab.solvers.qpoases`` dependency). When tests run from the repository
root the repository's ``marinholab`` package shadows the installed one, so we:

1. import ``dqrobotics`` first (the ``_core`` extension subclasses its
   pybind11 types, so the base types must be registered);
2. merge the installed ``marinholab`` directory into the top-level package's
   search path so sub-packages such as ``marinholab.solvers`` resolve;
3. import the compiled ``_core`` extension (loading the installed ``.so``
   directly when the repository checkout has no build of its own) and register
   it as ``marinholab.working.needlemanipulation._core`` before the package
   ``__init__`` runs;
4. only as a last resort (no compiled extension available at all) register a
   mock ``_core`` so the pure-Python controller / Jacobian logic stays
   importable and testable.

``CORE_AVAILABLE`` records whether the real compiled extension was usable.
"""
from __future__ import annotations

import glob
import importlib.util
import os
import site
import sys
from typing import Optional
from unittest.mock import MagicMock


def _site_dirs() -> list[str]:
    """System site-packages plus the user site-packages."""
    dirs = list(site.getsitepackages())
    try:
        usr = site.getusersitepackages()
        if usr:
            dirs.append(usr)
    except Exception:
        pass
    return dirs


def _merge_marinholab_site_packages() -> None:
    """Expose installed ``marinholab.*`` sub-packages under the repo package."""
    try:
        import marinholab
    except Exception:
        return
    try:
        pkg_dir = os.path.abspath(os.path.dirname(marinholab.__file__))
    except Exception:
        return
    for sp in _site_dirs():
        d = os.path.abspath(os.path.join(sp, "marinholab"))
        if os.path.isdir(d) and d != pkg_dir and d not in marinholab.__path__:
            marinholab.__path__.append(d)


def _installed_core_so() -> Optional[str]:
    """Path to an installed ``_core`` extension, if any."""
    for sp in _site_dirs():
        base = os.path.abspath(
            os.path.join(sp, "marinholab", "working", "needlemanipulation")
        )
        for pat in ("_core*.so", "_core*.pyd", "_core*.dll"):
            hits = sorted(glob.glob(os.path.join(base, pat)))
            if hits:
                return hits[0]
    return None


def _load_real_core() -> bool:
    """Load the compiled ``_core`` extension and register it under the repo
    submodule name. Returns ``True`` on success."""
    so = _installed_core_so()
    if so is None:
        return False
    try:
        import dqrobotics  # noqa: F401  register base pybind11 types
    except Exception:
        return False
    _merge_marinholab_site_packages()
    try:
        import marinholab.working.needlemanipulation._core  # noqa: F401
        return True
    except Exception:
        pass
    name = "marinholab.working.needlemanipulation._core"
    try:
        spec = importlib.util.spec_from_file_location(name, so)
        if spec is None or spec.loader is None:
            return False
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return True
    except Exception:
        return False


def _install_core_mock() -> None:
    mock_core = MagicMock()
    mock_core.M3_SerialManipulatorSimulatorFriendly = MagicMock()
    sys.modules["marinholab.working.needlemanipulation._core"] = mock_core


def _ensure_core_available() -> bool:
    try:
        import dqrobotics  # noqa: F401
    except Exception:
        pass
    _merge_marinholab_site_packages()
    if _load_real_core():
        return True
    _install_core_mock()
    return False


CORE_AVAILABLE: bool = _ensure_core_available()


import pytest  # noqa: E402  (imported after sys.modules patching)


@pytest.fixture(scope="session")
def core_available() -> bool:
    """Whether the compiled ``_core`` extension is importable."""
    return CORE_AVAILABLE
