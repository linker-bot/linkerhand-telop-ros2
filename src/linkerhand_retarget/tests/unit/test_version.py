from pathlib import Path


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

    assert get_version(version_file=missing_file) == "2.12.6"


def test_find_version_file_uses_repository_root():
    from linkerhand_retarget.version import find_version_file

    version_file = find_version_file(Path(__file__))

    assert version_file.name == "VERSION.md"
    assert version_file.exists()
