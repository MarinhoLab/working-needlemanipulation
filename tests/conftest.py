"""Shared pytest configuration for the needlemanipulation test suite.

The package ``__init__`` re-exports ``SerialManipulatorSimulatorFriendly``
from ``marinholab.sas.core.modeling`` — the compiled pybind11 extension that
now ships in the separate ``marinholab-sas-core`` package (the model moved
out of this repository). When tests run from the repository root, the
repository's ``marinholab`` package (which only ships ``marinholab.working``)
shadows the installed one, so we:

1. import ``dqrobotics`` first (the sas-core extension subclasses its
   pybind11 types, so the base types must be registered);
2. merge the installed ``marinholab`` directory into the top-level package's
   search path so sub-packages such as ``marinholab.sas`` resolve;
3. check that ``marinholab.sas.core.modeling`` is importable. If
   ``marinholab-sas-core`` is not installed (a bare dev checkout), install a
   mock so the pure-Python controller / Jacobian logic stays importable and
   testable.

``CORE_AVAILABLE`` records whether the real modeling extension was usable.
"""
from __future__ import annotations

import os
import site
import sys
import types
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
    """Expose the installed ``marinholab.*`` sub-packages under the repo package.

    The repo checkout provides ``marinholab.working``; the installed
    ``marinholab-sas-core`` provides ``marinholab.sas``. Merging the installed
    ``marinholab`` directory into the repo package's ``__path__`` lets both
    sub-packages resolve when the repo shadows the installed one.
    """
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


def _install_modeling_mock() -> None:
    """Register mock ``marinholab.sas.core.*`` modules so the package
    ``__init__`` can import when ``marinholab-sas-core`` is not installed.

    The ``__init__`` re-exports from ``marinholab.sas.core.modeling`` (the
    kinematics model) and ``marinholab.sas.core.papers.icra2019`` (the ICRA 2019
    task-space ``Controller``), so both are mocked here. ``Controller`` is set
    to the ``MagicMock`` *class* (not an instance) because
    ``NeedleController`` subclasses it at import time.
    """
    for name in (
        "marinholab.sas",
        "marinholab.sas.core",
        "marinholab.sas.core.modeling",
        "marinholab.sas.core.papers",
        "marinholab.sas.core.papers.icra2019",
    ):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)

    modeling = sys.modules["marinholab.sas.core.modeling"]
    modeling.SerialManipulatorSimulatorFriendly = MagicMock()
    modeling.ActuationType = MagicMock()

    icra2019 = sys.modules["marinholab.sas.core.papers.icra2019"]
    icra2019.Controller = MagicMock


def _ensure_modeling_available() -> bool:
    """Return True if the real ``marinholab.sas.core.modeling`` is importable;
    otherwise install a mock and return False."""
    try:
        import dqrobotics  # noqa: F401
    except Exception:
        pass
    _merge_marinholab_site_packages()
    try:
        import marinholab.sas.core.modeling  # noqa: F401
        return True
    except Exception:
        _install_modeling_mock()
        return False


CORE_AVAILABLE: bool = _ensure_modeling_available()


import pytest  # noqa: E402  (imported after sys.modules patching)


@pytest.fixture(scope="session")
def core_available() -> bool:
    """Whether the real modeling extension (``marinholab.sas.core.modeling``)
    is importable."""
    return CORE_AVAILABLE
