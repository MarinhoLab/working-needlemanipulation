# AGENTS.md

Repository-specific notes for working in `MarinhoLab/working-needlemanipulation`.

## Building the C++ extension (`_core`)

`pip install . --no-build-isolation` requires `cmake`, `ninja` and `g++` on
`PATH`. In this sandbox the venv's `bin` (e.g. `/tmp/venv-test/bin`) holds
`cmake`/`ninja` but is not on `PATH`; export it first:

```
export PATH="/tmp/venv-test/bin:$PATH"
```

## dqrobotics solver classes (important)

`dqrobotics.solvers.DQ_QuadprogSolver` (the concrete solver used by
`icra2019_controller.py`) is a thin Python wrapper around the `quadprog`
package, and dqrobotics imports it inside a **bare `try/except: pass`**.
Without `quadprog` installed, `DQ_QuadprogSolver` silently does not exist and
only the abstract `DQ_QuadraticProgrammingSolver` (pybind11, "pure virtual
function") is available.

`quadprog` is a **declared runtime dependency** of this package
(`pyproject.toml`), and the `build` job in
`.github/workflows/python-publish.yml` installs it on purpose. If the
controller raises "pure virtual function" at `DQ_QuadprogSolver()`,
`quadprog` is missing in the environment.

## Tests

There is **no repository test suite** — `tests/` was removed together with
the `simulator_tests` merge (the old `conftest.py` mocked `_core` and is no
longer present). There is also no dedicated CI `test` job; the workflow only
runs `build` (matrix wheel builds) and `publish`. To smoke-test a change
locally, build the package in a venv and exercise the controller API:

```
python -c "
from importlib.resources import files
from dqrobotics import DQ
from marinholab.working.needlemanipulation import NeedleController
from marinholab.working.needlemanipulation.example_load_from_file import get_information_from_file
r, r1, r2 = get_information_from_file(files('marinholab.working.needlemanipulation').joinpath('left_robot.yaml').read_text())
c = NeedleController(r, 10.0, 0.01, 0.999, [(r1['position'], r1['radius'], 6)],
    DQ([1]), [DQ([1,2,3])], 0.003, insertion_constraints=True)
H, f, W, w = c._get_optimization_parameters([0.0]*9, r.fkm([0.0]*9))
print(W.shape)
"
```

## CI build job: do not cache `build/`

The C++ build configures CMake with its binary directory inside `build/`, so
`build/CMakeCache.txt` pins the exact Python path from
`/opt/hostedtoolcache` (e.g. `.../Python/3.12.13/x64`) at configure time. The
cache key for that directory used to be keyed on the minor Python version
(3.10/3.11/3.12) plus source-file hashes, so after a runner-image Python
update (3.12.13 -> 3.12.14) the restored cache pointed at a deleted install
dir and configure failed with
`Could NOT find Python ... Cannot run the interpreter ...` (the `-DPYTHON_EXECUTABLE`
CLI flag cannot override an existing `CMakeCache.txt` entry). This made the
failing matrix cells vary between runs and is NOT a pybind11/CMake 4.x
incompatibility. Fix: the workflow caches only the pip cache, not `build/`.

## CI publish job: Linux wheels must be `manylinux`-tagged

`_core` is a binary extension, so the Linux wheels are binary wheels too.
PyPI's upload endpoint rejects binary wheels with a `linux_x86_64`/
`linux_aarch64` platform tag with HTTP 400 `unsupported platform tag` — they
must carry a PEP 600 `manylinux` tag. That is why the build job runs
`auditwheel repair --plat auto` (and installs `patchelf`) on Linux before
uploading the artifact. Do not drop that step, and do not hand a pinned
`manylinux_2_X` tag: `--plat auto` derives the tag from the wheel's actual
symbols (e.g. `manylinux_2_24`), while a pinned glibc-based tag that is
stricter than the symbols allow makes `repair` fail the build.

## Type checking with pyright (and the `stubs/` package)

The `marinholab` package is checked with **pyright** in `standard` mode
(`[tool.pyright]` in `pyproject.toml`). It runs clean: `0 errors, 0 warnings`.

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
    solvers/__init__.pyi    # DQ_QuadraticProgrammingSolver, DQ_QuadprogSolver
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
  one-line summary, then `Args:`, `Returns:`/`Return:`, and `Raises:`
  blocks as applicable. Module docstrings open with the copyright header and
  a one-paragraph description of what the module provides.
- **C++** sources use Doxygen `@file` / `@brief` / `@param` / `@return` /
  `@throws` comments on the public API and the non-trivial protected
  helpers (see `include/M3_SerialManipulatorSimulatorFriendly.h` and
  `src/M3_SerialManipulatorSimulatorFriendly.cpp`).
- The C++ `_core` extension's Python-visible surface is documented via the
  `m.doc()` Sphinx text in `src/core.cpp` and the `_core.pyi` type stub;
  keep the two consistent when the API changes.

