#!/usr/bin/env bash
# Installs everything needed to watch matches with the team's simulator.
#
# Usage:
#   ./install.sh                                # compiles our rSim on this machine
#   RSIM_WHEEL=<URL or path .whl> ./install.sh  # uses an already compiled rSim (faster)
#
# Works on macOS and on Ubuntu 24.04 or newer (also inside WSL2 on Windows).
# When it finishes:  source .venv/bin/activate && python scripts/view_match.py

# What does set -euo pipefail do?
set -euo pipefail

# The team's fork of rSim (with the omni robot). To use another one:
#   RSIM_URL=git+https://github.com/other-org/rSim@main ./install.sh
# (ALWAYS name the branch or tag: @main or @v1.3.0; @HEAD doesn't work)
RSIM_URL="${RSIM_URL:-git+https://github.com/Roborregos-VSSS2-2027/rSim@main}"
RSIM_WHEEL="${RSIM_WHEEL:-}"

echo "==> 1/4 System dependencies"
case "$(uname -s)" in
  Darwin)
    command -v brew >/dev/null || { echo "Homebrew is missing: https://brew.sh"; exit 1; }
    brew list python@3.12 >/dev/null 2>&1 || brew install python@3.12
    PY="$(brew --prefix)/bin/python3.12"
    if [ -z "$RSIM_WHEEL" ]; then
      xcode-select -p >/dev/null 2>&1 || { echo "The compiler is missing: run 'xcode-select --install'"; exit 1; }
      brew list ode >/dev/null 2>&1 || brew install ode
      export CMAKE_PREFIX_PATH="$(brew --prefix)"   # so CMake can find ODE
    fi
    ;;
  Linux)
    # Ubuntu (native or WSL2). We always use Python 3.12: it is the version
    # rSim, pybind11 and pygame are tested with.
    sudo apt-get update
    sudo apt-get install -y git build-essential cmake curl
    if apt-cache show python3.12-venv >/dev/null 2>&1; then
      # Ubuntu 24.04: Python 3.12 comes from the system repositories.
      sudo apt-get install -y python3.12 python3.12-venv python3.12-dev
      PY=python3.12
    else
      # Newer Ubuntu (26.04...) ships another Python version. We download a
      # separate Python 3.12 with uv; the system Python is not touched.
      # Why not just use the Python that comes with the system?
      echo "    This Ubuntu has no Python 3.12 package: downloading one with uv..."
      export PATH="$HOME/.local/bin:$PATH"
      command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
      uv python install 3.12
      PY="$(uv python find 3.12)"
    fi
    if [ -z "$RSIM_WHEEL" ] && [ ! -f /usr/local/include/ode/ode.h ]; then
      echo "    Compiling ODE 0.16.6 with libccd (only the first time)..."
      tmp="$(mktemp -d)"
      curl -sSL https://bitbucket.org/odedevs/ode/downloads/ode-0.16.6.tar.gz | tar xz -C "$tmp"
      cmake -S "$tmp/ode-0.16.6" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DODE_WITH_DEMOS=OFF -DODE_WITH_TESTS=OFF \
            -DODE_WITH_LIBCCD=ON
      cmake --build "$tmp/build" -j"$(nproc)"
      sudo cmake --install "$tmp/build"
      sudo ldconfig
    fi
    ;;
  *)
    echo "Unsupported system. On Windows use WSL2 (Ubuntu 24.04) and run this script there."; exit 1 ;;
esac

echo "==> 2/4 Python virtual environment (.venv)"
# Why do we need a virtual environment?
[ -d .venv ] || "$PY" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --quiet --upgrade pip

echo "==> 3/4 The team's rSim (omni robots)"
if [ -n "$RSIM_WHEEL" ]; then
  pip install --force-reinstall --no-deps "$RSIM_WHEEL"
else
  pip install --force-reinstall --no-deps "rc-robosim @ $RSIM_URL"
fi

# rSoccer requires rc-robosim >= 1.2. If the fork has no git tags, rSim is
# installed as 0.1.devN and pip would try to download (and fail to compile)
# the original rSim.
# Why does the version depend on git tags?
python - <<'EOF'
from importlib.metadata import version
v = version("rc-robosim")
try:
    ok = tuple(int(x) for x in v.split(".")[:2]) >= (1, 2)
except ValueError:
    ok = False
if not ok:
    raise SystemExit(
        f"\nrSim was installed as version {v}: the fork is missing a version tag.\n"
        "In your clone of the fork run:  git tag v1.3.0 && git push origin v1.3.0\n"
        "and run this script again. (See the guide.)")
print(f"    rSim installed: version {v}")
EOF

echo "==> 4/4 This project (strategies) + rSoccer"
pip install -e .

python -c "import robosim; robosim.VSS(0,1,0,25,[0,0,0,0],[[0,0,0]],[],omni_blue=True); print('rSim omni OK')"
echo
echo "Done. To watch a match:"
echo "  source .venv/bin/activate"
echo "  python scripts/view_match.py"
