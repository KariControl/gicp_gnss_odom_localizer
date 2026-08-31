#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate the curated evaluation assets committed for publication."""
from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "evaluation" / "assets"
MANIFEST = ASSETS / "manifest.json"
MAX_ASSET_BYTES = 10 * 1024 * 1024
ALLOWED_SUFFIXES = {".json", ".png"}
TEXT_SUFFIXES = {".json", ".md", ".txt", ".csv", ".yaml", ".yml"}
IGNORED_TOP_LEVEL = {"README.md", "manifest.json"}
PRIVATE_TEXT_PATTERNS = {
    "POSIX home path": re.compile(r"/(?:home|Users)/[^/\s]+/|/root/"),
    "Windows user path": re.compile(r"[A-Za-z]:[\\/]+Users[\\/]+[^\\/\s]+[\\/]"),
    "private source/run identifier": re.compile(
        r'"(?:local_source_id|source_artifacts|source_run_id|source_runs|'
        r'source_result_set|full_run_id|generated_by|source_hashes_pinned|'
        r'evaluated_profile_date|status_revalidated_without_replay)"\s*:'
    ),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(errors: list[str]) -> dict[str, object] | None:
    try:
        value = json.loads(MANIFEST.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f"cannot read {MANIFEST.relative_to(ROOT)}: {exc}")
        return None
    if not isinstance(value, dict):
        errors.append("publication asset manifest must be a JSON object")
        return None
    return value


def validate_manifest(
    manifest: dict[str, object], errors: list[str]
) -> dict[str, dict[str, object]]:
    if set(manifest) != {"schema_version", "files"}:
        errors.append("manifest must contain only schema_version and files")
    if manifest.get("schema_version") != 2:
        errors.append("manifest schema_version must be 2")
    raw_files = manifest.get("files")
    if not isinstance(raw_files, list) or not raw_files:
        errors.append("manifest files must be a non-empty array")
        return {}

    entries: dict[str, dict[str, object]] = {}
    for index, raw_entry in enumerate(raw_files):
        label = f"manifest files[{index}]"
        if not isinstance(raw_entry, dict):
            errors.append(f"{label} must be an object")
            continue
        if set(raw_entry) != {"path", "bytes", "sha256"}:
            errors.append(f"{label} must contain only path, bytes, and sha256")
            continue
        relative = raw_entry.get("path")
        byte_count = raw_entry.get("bytes")
        digest = raw_entry.get("sha256")
        if not isinstance(relative, str):
            errors.append(f"{label}.path must be a string")
            continue
        pure_path = PurePosixPath(relative)
        if (
            pure_path.is_absolute()
            or not pure_path.parts
            or ".." in pure_path.parts
            or "\\" in relative
            or str(pure_path) != relative
        ):
            errors.append(f"{label}.path is not a normalized relative path: {relative!r}")
            continue
        if relative in entries:
            errors.append(f"duplicate manifest path: {relative}")
            continue
        if not isinstance(byte_count, int) or isinstance(byte_count, bool) or byte_count < 0:
            errors.append(f"{label}.bytes must be a non-negative integer")
        if not isinstance(digest, str) or re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            errors.append(f"{label}.sha256 must be a lowercase SHA-256 digest")
        entries[relative] = raw_entry
    return entries


def published_files() -> dict[str, Path]:
    files: dict[str, Path] = {}
    for path in sorted(ASSETS.rglob("*")):
        relative = path.relative_to(ASSETS).as_posix()
        if path.is_file() and relative not in IGNORED_TOP_LEVEL:
            files[relative] = path
    return files


def validate_public_text(relative: str, path: Path, errors: list[str]) -> None:
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeError as exc:
        errors.append(f"published text asset is not UTF-8: {relative}: {exc}")
        return
    for description, pattern in PRIVATE_TEXT_PATTERNS.items():
        if pattern.search(text):
            errors.append(f"{description} found in {relative}")


def validate_file(relative: str, path: Path, entry: dict[str, object], errors: list[str]) -> None:
    if path.is_symlink():
        errors.append(f"published asset must not be a symlink: {relative}")
        return
    if path.suffix.lower() not in ALLOWED_SUFFIXES:
        errors.append(f"unsupported published asset type: {relative}")

    actual_size = path.stat().st_size
    if actual_size > MAX_ASSET_BYTES:
        errors.append(f"published asset exceeds 10 MiB: {relative} ({actual_size} bytes)")
    if entry.get("bytes") != actual_size:
        errors.append(
            f"size mismatch for {relative}: manifest={entry.get('bytes')!r}, actual={actual_size}"
        )
    actual_digest = sha256(path)
    if entry.get("sha256") != actual_digest:
        errors.append(f"SHA-256 mismatch for {relative}")

    if path.suffix.lower() in TEXT_SUFFIXES:
        validate_public_text(relative, path, errors)
        if path.suffix.lower() == ".json":
            try:
                json.loads(path.read_text(encoding="utf-8"))
            except (UnicodeError, json.JSONDecodeError) as exc:
                errors.append(f"invalid JSON in {relative}: {exc}")


def main() -> int:
    errors: list[str] = []
    manifest = load_manifest(errors)
    entries = validate_manifest(manifest, errors) if manifest is not None else {}
    actual_files = published_files()

    for public_text in (ASSETS / "README.md", MANIFEST):
        if public_text.is_file():
            validate_public_text(public_text.name, public_text, errors)

    for relative in sorted(set(entries) - set(actual_files)):
        errors.append(f"manifest entry is missing from checkout: {relative}")
    for relative in sorted(set(actual_files) - set(entries)):
        errors.append(f"unlisted file in publication assets: {relative}")
    for relative in sorted(set(entries) & set(actual_files)):
        validate_file(relative, actual_files[relative], entries[relative], errors)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"Publication assets OK: {len(entries)} files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
