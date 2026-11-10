"""Run the registered P1 tests against isolated output directories."""
import json
from pathlib import Path
import subprocess


def bracket(value):
    text = str(value)
    if "]===]" in text:
        raise ValueError("unsupported CTest argument")
    return "[===[" + text + "]===]"


def prepare_ctest(build, destination):
    build = Path(build).resolve(strict=True)
    destination = Path(destination)
    destination.mkdir(exist_ok=False)
    listing = subprocess.check_output(["ctest", "--test-dir", str(build), "--show-only=json-v1"])
    tests = json.loads(listing)["tests"]
    if not tests:
        raise ValueError("zero registered P1 CTests")
    (destination / "original-registration.json").write_bytes(listing)
    with (destination / "CTestTestfile.cmake").open("x") as output:
        for test in tests:
            args = []
            for value in test["command"]:
                path = Path(value)
                if path.parent == build and (path.name.endswith("-artifacts") or
                        path.name.startswith(("delivery-", "execution-", "termination-"))):
                    # These parameters are output roots in the existing CMake
                    # registration. All binaries and checked-in fixtures stay bound.
                    if path.is_file():
                        raise ValueError("expected an output directory: " + value)
                    value = destination / path.name
                args.append(bracket(value))
            output.write("add_test(" + bracket(test["name"]) + " " + " ".join(args) + ")\n")
            for prop in test["properties"]:
                if prop["name"] not in ("WORKING_DIRECTORY", "TIMEOUT", "RUN_SERIAL", "ENVIRONMENT"):
                    raise ValueError("unhandled test property: " + prop["name"])
                value = prop["value"]
                if prop["name"] == "WORKING_DIRECTORY" and value == str(build):
                    value = destination
                if prop["name"] == "ENVIRONMENT":
                    if not all(";" not in entry for entry in value):
                        raise ValueError("unsupported semicolon in CTest environment")
                    value = ";".join(value)
                output.write("set_tests_properties(" + bracket(test["name"]) + " PROPERTIES " +
                             prop["name"] + " " + bracket(value) + ")\n")
    return len(tests)
