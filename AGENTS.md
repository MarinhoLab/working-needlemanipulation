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

## Known pre-existing issue (not fixed here)

The C++ build fails intermittently in CI because the vendored pybind11
submodule's `find_package(Python)` is incompatible with CMake 4.x
(`Configuring incomplete, errors occurred!`). This fails on `main` too; the
failing matrix cell varies between runs.
