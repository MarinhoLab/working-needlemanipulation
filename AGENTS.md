# AGENTS.md

Repository-specific notes for working in `MarinhoLab/working-needlemanipulation`.

## Building / installing the package

This package is **pure Python** — there is no in-repo C++ extension to build.
The `SerialManipulatorSimulatorFriendly` kinematics model used by the
controllers moved to the **`marinholab-sas-core`** package (the pybind11
bindings for `marinholab_sas_core`), which is now a declared runtime
dependency. The only thing installed from this repository is the
`marinholab.working.needlemanipulation` Python package.

Install with:

```
pip install .
```

Build requires `setuptools`, `wheel` and `setuptools-git-versioning`
(declared in `[build-system]` in `pyproject.toml`). The **version is taken
from git tags** (`setuptools-git-versioning`, `dynamic = ["version"]`), so an
untagged checkout produces a dev version.

## QP solver (`marinholab-solvers-qpoases`)

The controllers solve the velocity QP with
`marinholab.solvers.qpoases.Solver` — the **`marinholab-solvers-qpoases`**
package (a qpOASES wrapper that ships prebuilt binaries). It is a **declared
runtime dependency** (`pyproject.toml`) and is installed alongside the package.

The ICRA 2019 task-space controller (`Controller`, re-exported here as
`ICRA19TaskSpaceController`) now lives in the **`marinholab-sas-core`**
package at `marinholab.sas.core.papers.icra2019`. It does
`from marinholab.solvers.qpoases import Solver`
and stores it as `self.qp_solver`. Its
`solve_quadratic_program(H, f, A, b, Aeq, beq)` accepts `A=None`/`Aeq=None`
and returns the solution `x`, so the controller call sites are unchanged.
`NeedleController` (in this package) subclasses that `Controller` and stacks
the vessel VFI constraints on top of it.

## Constraint debug output (`_debug` and `verbose`)

All per-constraint debug printing lives in `marinholab/working/needlemanipulation/_debug.py`.
It centralises the messages with a single grammar: a fixed-width category tag
(e.g. `[radius    #0]`) followed by `key=value` fields in a fixed numeric
format, and red `VIOLATION:` lines for breached constraints.

The available categories are `radius`, `plane`, `orientation`, `insertion` and
`rcm` (see `CONSTRAINT_CATEGORIES`). The public API accepts a single
`verbose` kwarg (type `Verbose = bool | dict[str, bool]`):

- `True` — print every category;
- `False` — print nothing (the default);
- `{"rcm": True, ...}` — select categories by name.

`NeedleController` accepts `verbose=` and normalises it via the local
`normalize_verbose` (all five categories) at the controller boundary, so the
per-constraint helpers (`debug_radius`, `debug_plane`, `debug_orientation`,
`debug_insertion`, `debug_rcm`) each gate on their own category. The parent
`Controller` (in `marinholab-sas-core`) only understands the `rcm` category,
so `NeedleController` hands it the `rcm` flag alone. To add a new category:
extend `CONSTRAINT_CATEGORIES` and add a matching `debug_*` helper. Do not
reintroduce per-constraint `verbose_*` kwargs — the single `verbose`
setting is the intended interface.

## Simulation scripts (`saul/`)

`saul/` holds the live end-to-end scripts for the pediatric-simulator
scenario (e.g. `insertion_1.py`, `needle_driving_*.py`). They are **not** part
of the installed package and are excluded from the type-checker; they run
against a live `PedriatricSimulator` process over TCP (`127.0.0.1`) and need
it on `PYTHONPATH` plus a running instance. Use them as references for how the
controllers are driven in a closed loop, not as a test suite.

## Tests

`tests/` holds the pytest suite. `conftest.py` imports `dqrobotics` first
(the sas-core extension subclasses its pybind11 types, so the base types must
be registered), merges the installed `marinholab` site-packages into the
repo package's `__path__` so `marinholab.sas` resolves, and records in
`CORE_AVAILABLE` whether `marinholab.sas.core.modeling` is importable. When
`marinholab-sas-core` is not installed (a bare dev checkout), a mock is
installed so the pure-Python controller / Jacobian tests still run; the
modeling-dependent tests are then skipped.

Run the suite from the repo root with `marinholab-sas-core` and
`marinholab-solvers-qpoases` installed in the same environment:

```
pytest
```

## CI

The workflow (`.github/workflows/python-publish.yml`) builds a **pure-Python**
wheel in a matrix (Ubuntu / Ubuntu-aarch64 / Windows × Python 3.10/3.11/3.12)
and publishes to PyPI on push to `main`. Because there is no in-repo C++
extension:

- The wheels are **pure Python** (no binary extension), so the Linux wheels
  are not binary wheels and `auditwheel repair` is **not** required.
- There is no CMake `build/` cache to worry about; the workflow caches only
  the pip wheel cache.
- Runtime dependencies (`dqrobotics`, `marinholab-sas-core`,
  `marinholab-solvers-qpoases`, ...) are pulled from PyPI; the build job
  itself installs `libeigen3-dev`/`build-essential` on Linux only so the
  *dependency* wheels that need a C++ toolchain (e.g. `dqrobotics`'s
  prebuilt wheels may still require a linker) can be validated.

## Type checking with pyright (and the `stubs/` package)

The `marinholab` package is checked with **pyright** in `standard` mode
(`[tool.pyright]` in `pyproject.toml`). The invariant is **0 errors**.
`dqrobotics` ships neither a `py.typed` marker nor `.pyi` stubs (its core is a
compiled extension), so the repo keeps its own typed stubs under
`stubs/dqrobotics` and points pyright at them via `stubPath` — type
information comes from those stubs, not the installed package. If the runtime
dependencies are not installed, pyright can additionally report
`reportMissingModuleSource` / `reportMissingImports` for the `dqrobotics`
imports; those are an environment issue, not a regression. Run the check with
the dependencies installed (see the build section); any *other* warning or any
error is a regression to fix.

`marinholab.sas.core.modeling` (the `SerialManipulatorSimulatorFriendly`
binding, now provided by the `marinholab-sas-core` package) ships its own
typed `.pyi` stubs as package data, so pyright resolves its types from the
installed package — no stubs are needed here for it.

The package depends on the third-party **`dqrobotics`** library, which is a
compiled pybind11 extension that ships **no `py.typed` marker and no `.pyi`
stubs**. As a result every name it exposes (e.g. `DQ`, `haminus4`,
`DQ_Kinematics`) is `Unknown`/undefined to a checker, and star imports from
it fail `reportUndefinedVariable`.

To keep the rest of the codebase fully checkable **without weakening the
checks globally**, we maintain a small, closed-set stub package under
`stubs/dqrobotics/`:

```
stubs/dqrobotics/
    __init__.pyi            # DQ + math helpers (i_, j_, k_, E_, conj, dot, ...)
    robot_modeling/__init__.pyi   # DQ_SerialManipulator, DQ_Kinematics
    utils/__init__.pyi      # DQ_Geometry
```

pyright is pointed at it via `stubPath = "stubs"` in `pyproject.toml`, so it
resolves `dqrobotics` from the stubs rather than the installed (untyped)
package. The stubs declare **only the symbols this project actually
imports** — not a full mirror of `dqrobotics`' API. If a new `dqrobotics`
symbol is needed, add it to the matching stub file.

- **When `dqrobotics` ships its own type information**, delete `stubs/` and
  the `stubPath` entry — the real types will take over.
- Run the check from the repo root (with a venv on the `venvPath`/`venv`
  set in `pyproject.toml`): `pyright`.

## Annotation + Doxygen conventions

All shipped Python is fully annotated and documented:

- **Functions/methods** carry full parameter and return annotations
  (`np.ndarray`, `DQ`, `Optional[...]`, `Tuple[...]`, ...). Use `Optional`
  (or `X | None`) where a value may be `None`; never annotate a mutable
  default with a type that `None` can't satisfy.
- **Docstrings** follow the Doxygen-style form used across the repo: a short
  one-line summary, then `Args:`, `Returns:`, and `Raises:` blocks as
  applicable. Module docstrings open with a one-paragraph description of what
  the module provides.
- The `SerialManipulatorSimulatorFriendly` kinematics model lives in the
  **`marinholab-sas-core`** package (`marinholab.sas.core.modeling`), not in
  this repository; this repo re-exports it (and the
  `M3_SerialManipulatorSimulatorFriendly` backward-compatibility alias) in
  `marinholab/working/needlemanipulation/__init__.py`. Its Python-visible
  surface is documented by that package's stubs.

