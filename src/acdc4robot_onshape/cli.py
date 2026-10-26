"""Local-only standalone beta command line."""
import argparse
import json
from pathlib import Path
from xml.etree import ElementTree as ET
import zipfile
from .bundle import inspect_bundle, prepare_bundle


def main(argv=None):
    parser = argparse.ArgumentParser(description="Prepare or inspect exported Onshape URDF bundles locally (beta)")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "prepare"):
        command = commands.add_parser(name)
        command.add_argument("source", type=Path, help="export directory or ZIP")
        command.add_argument("--urdf", help="bundle-relative URDF path; required if there are multiple")
        command.add_argument("--package", action="append", default=[], metavar="NAME=ROOT",
                             help="explicit local root for a package:// URI")
        if name == "prepare":
            command.add_argument("output", type=Path, help="new separate output directory")
            command.add_argument("--source-url", help="optional user-supplied HTTPS Onshape source URL")
            command.add_argument("--revision", help="optional user-supplied version/revision identifier")
            command.add_argument("--configuration", help="optional configuration as a JSON object")
    args = parser.parse_args(argv)
    try:
        packages = {}
        for item in args.package:
            name, separator, value = item.partition("=")
            if not separator or not name or not value or name in packages:
                raise ValueError("Each --package must be a unique NAME=ROOT mapping")
            packages[name] = Path(value)
        if args.command == "prepare":
            metadata = {key: value for key, value in (("source_url", args.source_url),
                        ("revision", args.revision)) if value is not None}
            if args.configuration is not None:
                configuration = json.loads(args.configuration)
                if not isinstance(configuration, dict):
                    raise ValueError("--configuration must be a JSON object")
                metadata["configuration"] = configuration
            report = prepare_bundle(args.source, args.output, urdf=args.urdf,
                                    package_roots=packages, metadata=metadata)
        else:
            report = inspect_bundle(args.source, urdf=args.urdf, package_roots=packages)
    except (ValueError, OSError, RuntimeError, ET.ParseError, zipfile.BadZipFile) as exc:
        parser.exit(1, f"Bundle processing stopped: {exc}\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
