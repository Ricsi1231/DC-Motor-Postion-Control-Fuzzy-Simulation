"""Unit tests for the release version-bump tooling.

The semver rules encoded here decide what every merge to main publishes, so
they get the same scrutiny as the simulation code.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import bump_version
import pytest
from bump_version import (
    VersionError,
    bump,
    classify_commit,
    classify_commits,
    main,
    read_version,
    write_version,
)


class TestClassifyCommit:
    @pytest.mark.parametrize(
        "message",
        [
            "feat: add sliding mode controller",
            "feat(control): add sliding mode controller",
            "FEAT: shouting still counts",
        ],
    )
    def test_feat_is_a_minor_bump(self, message: str) -> None:
        assert classify_commit(message) == "minor"

    @pytest.mark.parametrize(
        "message",
        [
            "feat!: drop the legacy shim",
            "fix(api)!: rename compute_control",
            "refactor!: restructure the package",
        ],
    )
    def test_bang_marks_a_breaking_change(self, message: str) -> None:
        assert classify_commit(message) == "major"

    @pytest.mark.parametrize("keyword", ["BREAKING CHANGE", "BREAKING-CHANGE"])
    def test_breaking_change_footer_is_a_major_bump(self, keyword: str) -> None:
        message = f"fix: correct the encoder velocity\n\n{keyword}: get_velocity signature changed"

        assert classify_commit(message) == "major"

    @pytest.mark.parametrize(
        "message",
        [
            "fix: correct encoder velocity",
            "docs: update the readme",
            "chore(deps): bump numpy",
            "test: add coverage for the runner",
            "ci: cache pip",
            "perf: avoid a copy",
            "refactor: extract a helper",
        ],
    )
    def test_everything_else_is_a_patch_bump(self, message: str) -> None:
        assert classify_commit(message) == "patch"

    @pytest.mark.parametrize(
        "message",
        [
            "just some words",
            "Merge pull request #2 from Ricsi1231/pid-control",
            "",
            "   ",
        ],
    )
    def test_unparseable_subjects_fall_back_to_patch(self, message: str) -> None:
        assert classify_commit(message) == "patch"

    def test_breaking_change_must_be_a_footer_not_prose(self) -> None:
        """A mention in prose is not the Conventional Commits footer form."""
        message = "fix: tidy up\n\nthis is not a BREAKING CHANGE at all"

        assert classify_commit(message) == "patch"


class TestClassifyCommits:
    def test_takes_the_strongest_level(self) -> None:
        commits = ["docs: readme", "feat: new thing", "fix: a bug"]

        assert classify_commits(commits) == "minor"

    def test_a_single_breaking_commit_wins(self) -> None:
        commits = ["docs: readme", "feat: new thing", "feat!: removed an API"]

        assert classify_commits(commits) == "major"

    def test_empty_history_still_bumps_patch(self) -> None:
        """Every merge to main ships at least a patch release."""
        assert classify_commits([]) == "patch"

    def test_only_chores_bumps_patch(self) -> None:
        assert classify_commits(["chore: tidy", "docs: typo"]) == "patch"


class TestBump:
    @pytest.mark.parametrize(
        ("current", "level", "expected"),
        [
            ("1.0.0", "patch", "1.0.1"),
            ("1.0.0", "minor", "1.1.0"),
            ("1.0.0", "major", "2.0.0"),
            ("1.2.3", "patch", "1.2.4"),
            ("1.2.3", "minor", "1.3.0"),
            ("1.2.3", "major", "2.0.0"),
            ("0.9.9", "minor", "0.10.0"),
            ("9.9.9", "major", "10.0.0"),
        ],
    )
    def test_applies_the_level(self, current: str, level: str, expected: str) -> None:
        assert bump(current, level) == expected

    def test_minor_resets_patch(self) -> None:
        assert bump("1.4.7", "minor") == "1.5.0"

    def test_major_resets_minor_and_patch(self) -> None:
        assert bump("1.4.7", "major") == "2.0.0"

    @pytest.mark.parametrize("current", ["1.0", "v1.0.0", "1.0.0-rc1", "", "abc"])
    def test_rejects_a_malformed_current_version(self, current: str) -> None:
        with pytest.raises(VersionError, match=r"not a MAJOR\.MINOR\.PATCH"):
            bump(current, "patch")

    def test_rejects_an_unknown_level(self) -> None:
        with pytest.raises(VersionError, match="unknown bump level"):
            bump("1.0.0", "epic")


class TestVersionFile:
    def test_round_trips(self, tmp_path: Path) -> None:
        path = tmp_path / "version.txt"

        write_version("2.3.4", path)

        assert path.read_text() == "2.3.4\n"
        assert read_version(path) == "2.3.4"

    def test_tolerates_surrounding_whitespace(self, tmp_path: Path) -> None:
        path = tmp_path / "version.txt"
        path.write_text("  1.2.3  \n\n")

        assert read_version(path) == "1.2.3"

    def test_rejects_a_missing_file(self, tmp_path: Path) -> None:
        with pytest.raises(VersionError, match="does not exist"):
            read_version(tmp_path / "nope.txt")

    @pytest.mark.parametrize("contents", ["v1.0.0", "1.0", "not a version", ""])
    def test_rejects_malformed_contents(self, tmp_path: Path, contents: str) -> None:
        path = tmp_path / "version.txt"
        path.write_text(contents)

        with pytest.raises(VersionError, match=r"not a MAJOR\.MINOR\.PATCH"):
            read_version(path)

    def test_refuses_to_write_a_malformed_version(self, tmp_path: Path) -> None:
        with pytest.raises(VersionError, match=r"not a MAJOR\.MINOR\.PATCH"):
            write_version("1.0", tmp_path / "version.txt")


class TestRepositoryVersionFile:
    def test_the_real_file_is_valid(self) -> None:
        assert read_version() == bump_version.VERSION_FILE.read_text(encoding="utf-8").strip()

    def test_matches_the_installed_package_version(self) -> None:
        """version.txt is what the build backend publishes."""
        import dc_motor_sim

        assert dc_motor_sim.__version__.split("+")[0].startswith(read_version())


class TestCli:
    @pytest.fixture(autouse=True)
    def isolated_version_file(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
        """Point the script at a throwaway version.txt so tests never edit the real one.

        Regression: read_version/write_version once defaulted their `path`
        argument to VERSION_FILE, which binds at import time. This patch had no
        effect and `--write` tests rewrote the repository's real version.txt.
        """
        real = bump_version.VERSION_FILE
        real_contents = real.read_text(encoding="utf-8")

        path = tmp_path / "version.txt"
        path.write_text("1.2.3\n")
        monkeypatch.setattr(bump_version, "VERSION_FILE", path)

        yield path

        assert real.read_text(encoding="utf-8") == real_contents, (
            "a CLI test wrote to the repository's real version.txt"
        )

    def test_show_prints_the_current_version(self, capsys: pytest.CaptureFixture[str]) -> None:
        assert main(["--show"]) == 0
        assert capsys.readouterr().out.strip() == "1.2.3"

    @pytest.mark.parametrize(
        ("level", "expected"),
        [("patch", "1.2.4"), ("minor", "1.3.0"), ("major", "2.0.0")],
    )
    def test_forced_level_computes_the_next_version(
        self, capsys: pytest.CaptureFixture[str], level: str, expected: str
    ) -> None:
        assert main(["--level", level]) == 0
        assert capsys.readouterr().out.strip() == expected

    def test_does_not_write_without_the_flag(
        self, isolated_version_file: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        main(["--level", "minor"])
        capsys.readouterr()

        assert isolated_version_file.read_text() == "1.2.3\n"

    def test_write_updates_the_file(
        self, isolated_version_file: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        assert main(["--level", "minor", "--write"]) == 0
        capsys.readouterr()

        assert isolated_version_file.read_text() == "1.3.0\n"

    def test_set_forces_an_exact_version(
        self, isolated_version_file: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        assert main(["--set", "5.0.0", "--write"]) == 0
        assert capsys.readouterr().out.strip() == "5.0.0"
        assert isolated_version_file.read_text() == "5.0.0\n"

    def test_set_rejects_a_malformed_version(self, capsys: pytest.CaptureFixture[str]) -> None:
        assert main(["--set", "v5.0"]) == 2
        assert "not a MAJOR.MINOR.PATCH" in capsys.readouterr().err

    def test_github_format_emits_key_value_lines(self, capsys: pytest.CaptureFixture[str]) -> None:
        """The release workflow appends this straight to $GITHUB_OUTPUT."""
        assert main(["--level", "minor", "--format", "github"]) == 0

        lines = capsys.readouterr().out.strip().splitlines()

        assert lines == ["version=1.3.0", "tag=v1.3.0", "level=minor"]

    def test_github_format_works_with_show(self, capsys: pytest.CaptureFixture[str]) -> None:
        assert main(["--show", "--format", "github"]) == 0
        assert capsys.readouterr().out.strip() == "version=1.2.3"

    def test_plain_format_prints_only_the_version(self, capsys: pytest.CaptureFixture[str]) -> None:
        assert main(["--level", "major"]) == 0
        assert capsys.readouterr().out.strip() == "2.0.0"

    def test_reports_a_corrupt_version_file(
        self, isolated_version_file: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        isolated_version_file.write_text("garbage\n")

        assert main(["--show"]) == 2
        assert "not a MAJOR.MINOR.PATCH" in capsys.readouterr().err


def _in_git_checkout() -> bool:
    """The sdist ships these tests but carries no .git directory."""
    result = subprocess.run(
        ["git", "rev-parse", "--is-inside-work-tree"],
        cwd=bump_version.REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() == "true"


@pytest.mark.skipif(not _in_git_checkout(), reason="requires a git checkout")
class TestGitIntegration:
    def test_reads_commit_messages_from_the_repository(self) -> None:
        messages = bump_version.git_commit_messages(since=None)

        assert messages
        assert all(isinstance(m, str) and m for m in messages)

    def test_unknown_ref_falls_back_to_full_history(
        self, capsys: pytest.CaptureFixture[str]
    ) -> None:
        messages = bump_version.git_commit_messages(since="v999.999.999")

        assert messages
        assert "not a known ref" in capsys.readouterr().err

    def test_latest_tag_returns_a_string_or_none(self) -> None:
        """None is legitimate: a shallow CI clone may carry no tags."""
        tag = bump_version.latest_tag()

        assert tag is None or isinstance(tag, str)

    def test_classifying_real_history_yields_a_valid_level(self) -> None:
        level = classify_commits(bump_version.git_commit_messages(since=None))

        assert level in {"major", "minor", "patch"}
