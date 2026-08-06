import re
from pathlib import Path
from typing import Optional


_VERSION_HEADING_RE = re.compile(r"^##\s+v?([0-9]+(?:\.[0-9]+){1,3})\s*$")


def find_version_file(start: Optional[Path] = None) -> Path:
    current = Path(start or __file__).resolve()
    if current.is_file():
        current = current.parent

    for directory in (current, *current.parents):
        version_file = directory / "VERSION.md"
        if version_file.exists():
            return version_file

    return Path(__file__).resolve().parents[2] / "VERSION.md"


def _fallback_version() -> str:
    try:
        from .linkerhand import __version__
    except Exception:
        return "0.0.0"
    return str(__version__)


def get_version(version_file: Optional[Path] = None) -> str:
    path = Path(version_file) if version_file is not None else find_version_file()
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return _fallback_version()

    for line in lines:
        match = _VERSION_HEADING_RE.match(line.strip())
        if match:
            return match.group(1)

    return _fallback_version()
