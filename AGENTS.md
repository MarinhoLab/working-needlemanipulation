# AGENTS.md

Repository-specific notes for working in `MarinhoLab/working-needlemanipulation`.

## Building the C++ extension (`_core`)

`_core` is a pybind11 module built by CMake — `setup.py` drives CMake through
`setuptools`, and `CMakeLists.txt` compiles `src/core.cpp` +
`src/M3_SerialManipulatorSimulatorFriendly.cpp`.

Building requires:

- `cmake` (≥3.15), `ninja` and `g++` on `PATH`.
- **Eigen3** — `CMakeLists.txt` runs `find_package(Eigen3 REQUIRED)` and links
  `Eigen3::Eigen`. Debian/Ubuntu: `sudo apt-get install libeigen3-dev` (the CI
  build job does this; on Windows it is pulled in via vcpkg).
- **Initialised git submodules** — `submodules/pybind11` (v3.0) and
  `submodules/dqrobotics/cpp` are pulled in with `add_subdirectory`. A plain
  clone has empty submodule dirs, so run:

  ```
  git submodule update --init --recursive
  ```

- The **version is taken from git tags** (`setuptools-git-versioning`,
  `dynamic = ["version"]` in `pyproject.toml`), so an untagged checkout
  produces a dev version.

Then install with:

```
pip install . --no-build-isolation
```

On **aarch64/arm64** builds, `setup.py` appends `-ffp-contract=off` so the
floating-point results match the reference MATLAB behaviour — do not strip
that flag.

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

Both `ICRA19TaskSpaceController` and `NeedleController` accept `verbose=` and
normalise it via `normalize_verbose` at the controller boundary, so the
per-constraint helpers (`debug_radius`, `debug_plane`, `debug_orientation`,
`debug_insertion`, `debug_rcm`) each gate on their own category. To add a new
category: extend `CONSTRAINT_CATEGORIES` and add a matching `debug_*` helper.
Do not reintroduce per-constraint `verbose_*` kwargs — the single `verbose`
setting is the intended interface.

## Simulation scripts (`saul/`)

`saul/` holds the live end-to-end scripts for the pediatric-simulator
scenario (e.g. `insertion_1.py`, `needle_driving_*.py`). They are **not** part
of the installed package and are excluded from the type-checker; they run
against a live `PedriatricSimulator` process over TCP (`127.0.0.1`) and need
it on `PYTHONPATH` plus a running instance. Use them as references for how the
controllers are driven in a closed loop, not as a test suite.

## Tests

There is **no repository test suite** — `tests/` was removed together with
the `simulator_tests` merge (the old `conftest.py` mocked `_core` and is no
longer present). There is also no dedicated CI `test` job; the workflow only
runs `build` (matrix wheel builds) and `publish`. To smoke-test a change
locally, build the package (see the build section) and exercise the controller
API:

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

> **Run it from outside the repo root** (e.g. `cd /tmp`). Running it from the
> repo root makes `import marinholab` resolve to the **source** tree, which
> does not contain the compiled `_core` extension, so the import fails. Run it
> from the repo root only after building `_core` in-place.

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

