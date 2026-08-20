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
(`pyproject.toml`). The CI test job must install it; the hand-picked test
dependency list in `.github/workflows/python-publish.yml` includes it on
purpose.

## Tests

`tests/conftest.py` mocks `marinholab.working.needlemanipulation._core`, so the
full suite runs without building the C++ extension. Run from the repo root:

```
/tmp/venv-test/bin/python -m pytest tests/ -v
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
