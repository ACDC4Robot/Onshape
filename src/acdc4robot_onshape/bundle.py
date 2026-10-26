"""Safe local URDF bundle intake and atomic preparation."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
from urllib.parse import unquote, urlsplit
from xml.etree import ElementTree as ET
import zipfile

from . import __version__
from .serialization import normalize_numbers
from .validation import validate_robot

MAX_ARCHIVE_BYTES = 512 * 1024 * 1024
MAX_ARCHIVE_ENTRIES = 10000
MAX_URDF_BYTES = 16 * 1024 * 1024


def digest(path):
    hasher = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def safe_relative(value, context):
    if not value or "\\" in value or ":" in value or "\x00" in value:
        raise ValueError(f"{context} must be a relative path with forward slashes")
    path = PurePosixPath(value)
    if not path.parts or path.is_absolute() or any(part == ".." for part in path.parts):
        raise ValueError(f"Unsafe {context}: {value!r}")
    return path


def contained_file(path, root):
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(root.resolve()):
        raise ValueError(f"Asset or URDF escapes its allowed root: {path.name!r}")
    if not resolved.is_file():
        raise ValueError(f"Expected a regular file: {path.name!r}")
    return resolved


def extract_archive(path, target):
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        if len(entries) > MAX_ARCHIVE_ENTRIES or sum(item.file_size for item in entries) > MAX_ARCHIVE_BYTES:
            raise ValueError("ZIP exceeds beta limits: 10000 entries or 512 MiB uncompressed")
        seen = set()
        for item in entries:
            relative = safe_relative(item.filename, "ZIP entry")
            name = relative.as_posix().rstrip("/").casefold()
            if name in seen:
                raise ValueError(f"Duplicate or case-colliding ZIP entry: {item.filename!r}")
            seen.add(name)
            mode = item.external_attr >> 16
            if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
                raise ValueError("ZIP links and special files are unsupported")
            destination = target / relative
            if item.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
                continue
            destination.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(item) as source, destination.open("xb") as output:
                shutil.copyfileobj(source, output, length=1024 * 1024)


@contextmanager
def input_directory(path):
    path = Path(path).resolve(strict=True)
    if path.is_dir():
        yield path, {"kind": "directory", "name": path.name}
    elif path.is_file() and path.suffix.lower() == ".zip":
        with tempfile.TemporaryDirectory(prefix="acdc4robot-onshape-input-") as temporary:
            root = Path(temporary)
            extract_archive(path, root)
            yield root, {"kind": "zip", "name": path.name, "archive_sha256": digest(path)}
    else:
        raise ValueError("Input must be an exported URDF directory or ZIP archive")


def select_urdf(root, selector):
    if selector is not None:
        relative = safe_relative(selector, "--urdf selector")
        selected = contained_file(root / relative, root)
        if selected.suffix.lower() != ".urdf":
            raise ValueError("--urdf must select a .urdf file")
        return selected
    choices = sorted(path for path in root.rglob("*") if path.is_file() and path.suffix.lower() == ".urdf")
    if len(choices) != 1:
        raise ValueError(f"Expected one URDF, found {len(choices)}; select a bundle-relative path with --urdf")
    return contained_file(choices[0], root)


def resolve_asset(reference, urdf_directory, bundle_root, package_roots):
    if not reference or "\\" in reference or "\x00" in reference:
        raise ValueError("Asset references must use local paths with forward slashes")
    uri = urlsplit(reference)
    if uri.query or uri.fragment:
        raise ValueError(f"Asset query strings and fragments are unsupported: {reference!r}")
    if uri.scheme == "package":
        if uri.netloc not in package_roots:
            raise ValueError(f"Package {uri.netloc!r} needs an explicit --package NAME=ROOT mapping")
        root = Path(package_roots[uri.netloc]).resolve(strict=True)
        if not root.is_dir():
            raise ValueError(f"Package root {uri.netloc!r} is not a directory")
        path = root / safe_relative(unquote(uri.path.lstrip("/")), "package asset path")
    elif uri.scheme or uri.netloc or Path(unquote(uri.path)).is_absolute():
        raise ValueError(f"Unsupported asset URI {reference!r}; use bundle-relative or explicitly mapped package:// paths")
    else:
        # ../meshes is legitimate from a nested URDF, provided the resolved
        # file remains inside the input bundle. This also checks symlinks.
        root = bundle_root
        path = urdf_directory / unquote(uri.path)
    resolved = contained_file(path, root)
    if resolved.stat().st_size == 0:
        raise ValueError(f"Referenced asset is empty: {reference!r}")
    return resolved


def read_bundle(root_directory, selector=None, package_roots=None):
    selected = select_urdf(root_directory, selector)
    if selected.stat().st_size > MAX_URDF_BYTES:
        raise ValueError("URDF exceeds the beta 16 MiB XML limit")
    source = selected.read_bytes()
    if b"<!DOCTYPE" in source.upper() or b"<!ENTITY" in source.upper():
        raise ValueError("URDF DTDs and entities are unsupported")
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    robot = ET.fromstring(source, parser=parser)
    validation = validate_robot(robot)
    assets = []
    for node in list(robot.iter("mesh")) + list(robot.iter("texture")):
        reference = node.get("filename")
        path = resolve_asset(reference, selected.parent, root_directory, package_roots or {})
        if node.tag == "mesh" and path.suffix.lower() != ".stl":
            raise ValueError(f"Unsupported mesh format {path.suffix!r}; this beta supports STL only")
        assets.append((node, path, {"role": node.tag, "source_reference": reference,
                                  "sha256": digest(path), "bytes": path.stat().st_size}))
    return selected, source, robot, validation, assets


def inspect_bundle(source, *, urdf=None, package_roots=None):
    with input_directory(source) as (directory, input_info):
        selected, source_xml, _, validation, assets = read_bundle(directory, urdf, package_roots)
        return {"schema": "acdc4robot-onshape-inspection/v1", "passed": True,
                "input": dict(input_info, urdf=selected.relative_to(directory).as_posix(),
                              urdf_sha256=hashlib.sha256(source_xml).hexdigest()),
                "validation": validation, "assets": [record for _, _, record in assets],
                "live_onshape_validation": False}


def prepare_bundle(source, output, *, urdf=None, package_roots=None, metadata=None):
    source = Path(source).resolve(strict=True)
    output_path = Path(output).absolute()
    if output_path.exists() or output_path.is_symlink():
        raise ValueError("Output directory must not already exist")
    output_path = output_path.resolve()
    if not output_path.parent.is_dir():
        raise ValueError("Output parent directory must already exist")
    if source.is_dir() and (output_path.is_relative_to(source) or source.is_relative_to(output_path)):
        raise ValueError("Input and output must be separate, non-nested directories")
    metadata = deepcopy(metadata or {})
    if set(metadata) - {"source_url", "revision", "configuration"}:
        raise ValueError("Metadata supports only source_url, revision, and configuration")
    if metadata.get("source_url"):
        uri = urlsplit(metadata["source_url"])
        if uri.scheme != "https" or not uri.hostname or uri.username or uri.password:
            raise ValueError("Source metadata URL must be HTTPS without embedded credentials")
    json.dumps(metadata, allow_nan=False)
    with input_directory(source) as (directory, input_info):
        selected, source_xml, robot, validation, assets = read_bundle(directory, urdf, package_roots)
        normalize_numbers(robot)
        report = {"schema": "acdc4robot-onshape-bundle/v1", "tool_version": __version__,
                  "input": dict(input_info, urdf=selected.relative_to(directory).as_posix(),
                                urdf_sha256=hashlib.sha256(source_xml).hexdigest()),
                  "onshape": dict(metadata, verification="user-supplied metadata; no live Onshape access"),
                  "validation": validation, "assets": [], "output_urdf": "robot.urdf",
                  "live_onshape_validation": False}
        with tempfile.TemporaryDirectory(prefix=f".{output_path.name}.stage-", dir=output_path.parent) as temporary:
            staging = Path(temporary) / "bundle"
            staging.mkdir()
            for node, path, record in assets:
                relative = f"assets/{record['sha256']}/{path.name}"
                destination = staging / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                if not destination.exists():
                    shutil.copyfile(path, destination)
                    if digest(destination) != record["sha256"]:
                        raise ValueError(f"Asset changed during preparation: {record['source_reference']!r}")
                node.set("filename", relative)
                report["assets"].append(dict(record, path=relative))
            provenance = staging / "provenance"
            provenance.mkdir()
            (provenance / "source-urdf.xml").write_bytes(source_xml)
            ET.ElementTree(robot).write(staging / "robot.urdf", encoding="utf-8", xml_declaration=True)
            report["output_urdf_sha256"] = digest(staging / "robot.urdf")
            (staging / "acdc4robot-onshape-report.json").write_text(
                json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            if output_path.exists() or output_path.is_symlink():
                raise ValueError("Output appeared during preparation; refusing to replace it")
            os.rename(staging, output_path)
        return report
