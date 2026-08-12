#!/usr/bin/env python3
"""Compute and apply the next semantic version from Conventional Commits.

``version.txt`` is the single source of truth for this project's version. CI
runs this script on every merge to ``main`` to decide the next value; see
``.github/workflows/release.yml``.

Bump rules:

- ``BREAKING CHANGE`` / ``BREAKING-CHANGE`` in the body or footer, or a ``!``
  before the colon (``feat!:``, ``fix(api)!:``) -> **major**
- ``feat`` -> **minor**
- anything else, including an unparseable subject -> **patch**

Every merge produces at least a patch bump, so a docs-only or chore-only merge
still ships a release.

Usage::

    python scripts/bump_version.py --show                 # print current version
    python scripts/bump_version.py --since v1.2.3         # print the next version
    python scripts/bump_version.py --since v1.2.3 --write # and update version.txt
    python scripts/bump_version.py --set 2.0.0            # force an exact version
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VERSION_FILE = REPO_ROOT / "version.txt"

MAJOR, MINOR, PATCH = "major", "minor", "patch"
_RANK = {PATCH: 0, MINOR: 1, MAJOR: 2}

SEMVER_RE = re.compile(r"^(?P<major>\d+)\.(?P<minor>\d+)\.(?P<patch>\d+)$")
# type(optional scope)optional-! :
SUBJECT_RE = re.compile(r"^(?P<type>[a-zA-Z]+)(?:\((?P<scope>[^)]*)\))?(?P<breaking>!)?:")
BREAKING_RE = re.compile(r"^BREAKING[ -]CHANGE\s*:", re.MULTILINE)

COMMIT_SEPARATOR = "\x1e"


class VersionError(ValueError):
    """Raised when a version string or bump request is not valid."""


def read_version(path: Path | None = None) -> str:
    """Read and validate the current version.

    ``path`` defaults to :data:`VERSION_FILE`, resolved on each call rather
    than bound at import, so the module attribute can be redirected in tests.
    """
    path = VERSION_FILE if path is None else path
    if not path.is_file():
        raise VersionError(f"{path} does not exist")
    raw = path.read_text(encoding="utf-8").strip()
    if not SEMVER_RE.match(raw):
        raise VersionError(f"{path} contains {raw!r}, which is not a MAJOR.MINOR.PATCH version")
    return raw


def write_version(version: str, path: Path | None = None) -> None:
    """Write a validated version, with a trailing newline."""
    path = VERSION_FILE if path is None else path
    if not SEMVER_RE.match(version):
        raise VersionError(f"{version!r} is not a MAJOR.MINOR.PATCH version")
    path.write_text(f"{version}\n", encoding="utf-8")


def classify_commit(message: str) -> str:
    """Return the bump level a single commit message calls for."""
    subject, _, body = message.strip().partition("\n")

    if BREAKING_RE.search(body) or BREAKING_RE.search(message):
        return MAJOR

    match = SUBJECT_RE.match(subject.strip())
    if match is None:
        return PATCH
    if match.group("breaking"):
        return MAJOR
    if match.group("type").lower() == "feat":
        return MINOR
    return PATCH


def classify_commits(messages: list[str]) -> str:
    """Return the strongest bump level required by a set of commits.

    Defaults to ``patch`` for an empty set, so every merge ships something.
    """
    level = PATCH
    for message in messages:
        candidate = classify_commit(message)
        if _RANK[candidate] > _RANK[level]:
            level = candidate
    return level


def bump(version: str, level: str) -> str:
    """Apply a bump level to a semantic version."""
    match = SEMVER_RE.match(version)
    if match is None:
        raise VersionError(f"{version!r} is not a MAJOR.MINOR.PATCH version")
    if level not in _RANK:
        raise VersionError(f"unknown bump level {level!r}")

    major, minor, patch = (int(match.group(part)) for part in ("major", "minor", "patch"))

    if level == MAJOR:
        return f"{major + 1}.0.0"
    if level == MINOR:
        return f"{major}.{minor + 1}.0"
    return f"{major}.{minor}.{patch + 1}"


def git_commit_messages(since: str | None, cwd: Path = REPO_ROOT) -> list[str]:
    """Return commit messages after ``since`` (a tag or ref), newest first.

    When ``since`` is ``None`` or unknown to git, every reachable commit is
    returned, which makes the very first release work.
    """
    revision_range = "HEAD"
    if since:
        try:
            subprocess.run(
                ["git", "rev-parse", "--verify", "--quiet", f"{since}^{{commit}}"],
                cwd=cwd,
                check=True,
                capture_output=True,
            )
            revision_range = f"{since}..HEAD"
        except subprocess.CalledProcessError:
            print(f"warning: {since!r} is not a known ref; using full history", file=sys.stderr)

    result = subprocess.run(
        ["git", "log", f"--format=%B{COMMIT_SEPARATOR}", revision_range],
        cwd=cwd,
        check=True,
        capture_output=True,
        text=True,
    )
    return [chunk.strip() for chunk in result.stdout.split(COMMIT_SEPARATOR) if chunk.strip()]


def latest_tag(cwd: Path = REPO_ROOT) -> str | None:
    """Return the most recent reachable tag, or ``None`` if there is none."""
    result = subprocess.run(
        ["git", "describe", "--tags", "--abbrev=0"],
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() or None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="bump_version.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--show", action="store_true", help="print the current version and exit")
    parser.add_argument(
        "--since",
        metavar="REF",
        default=None,
        help="classify commits after this tag or ref (default: the latest tag)",
    )
    parser.add_argument(
        "--set",
        dest="exact",
        metavar="VERSION",
        default=None,
        help="use this exact version instead of computing one",
    )
    parser.add_argument(
        "--level",
        choices=[MAJOR, MINOR, PATCH],
        default=None,
        help="force a bump level instead of reading commit messages",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="write the result to version.txt (otherwise it is only printed)",
    )
    parser.add_argument(
        "--format",
        choices=["plain", "github"],
        default="plain",
        help="'plain' prints the version alone; 'github' prints version/tag/level "
        "as key=value lines suitable for appending to $GITHUB_OUTPUT",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        current = read_version()

        if args.show:
            print(current if args.format == "plain" else f"version={current}")
            return 0

        if args.exact:
            new_version = args.exact
            if not SEMVER_RE.match(new_version):
                raise VersionError(f"{new_version!r} is not a MAJOR.MINOR.PATCH version")
            level = "explicit"
        else:
            level = args.level or classify_commits(
                git_commit_messages(args.since if args.since is not None else latest_tag())
            )
            new_version = bump(current, level)
    except VersionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"current={current}", file=sys.stderr)
    print(f"level={level}", file=sys.stderr)

    if args.write:
        write_version(new_version)
        print(f"wrote {VERSION_FILE}: {new_version}", file=sys.stderr)

    if args.format == "github":
        print(f"version={new_version}")
        print(f"tag=v{new_version}")
        print(f"level={level}")
    else:
        print(new_version)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
