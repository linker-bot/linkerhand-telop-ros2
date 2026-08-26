import ast
from pathlib import Path


def _contains_pep604_union(annotation):
    if annotation is None:
        return False
    return any(
        isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr)
        for node in ast.walk(annotation)
    )


def test_version_module_annotations_are_python39_compatible():
    source_path = Path(__file__).parents[2] / "linkerhand_retarget" / "version.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    pep604_annotations = []

    for node in ast.walk(tree):
        annotation = getattr(node, "annotation", None)
        if _contains_pep604_union(annotation):
            pep604_annotations.append(getattr(node, "arg", getattr(node, "name", "unknown")))

        returns = getattr(node, "returns", None)
        if _contains_pep604_union(returns):
            pep604_annotations.append(getattr(node, "name", "return"))

    assert pep604_annotations == []


def test_get_version_reads_first_version_heading(tmp_path):
    from linkerhand_retarget.version import get_version

    version_file = tmp_path / "VERSION.md"
    version_file.write_text(
        "# 版本说明\n\n"
        "## v9.8.7\n\n"
        "- current\n\n"
        "## v1.2.3\n\n"
        "- old\n",
        encoding="utf-8",
    )

    assert get_version(version_file=version_file) == "9.8.7"


def test_get_version_falls_back_to_package_version(tmp_path):
    from linkerhand_retarget.version import get_version

    missing_file = tmp_path / "missing_VERSION.md"

    assert get_version(version_file=missing_file) == "2.12.9"


def test_find_version_file_uses_repository_root():
    from linkerhand_retarget.version import find_version_file

    version_file = find_version_file(Path(__file__))

    assert version_file.name == "VERSION.md"
    assert version_file.exists()
