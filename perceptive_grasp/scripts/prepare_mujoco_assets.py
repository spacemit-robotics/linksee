#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

"""Download and verify pinned MuJoCo simulation assets."""

import argparse
import hashlib
import os
import time
import urllib.error
import urllib.request
from pathlib import Path


MENAGERIE_COMMIT = "da76818e269b82289eba39808e2fb91d679d6994"
BASE_URL = (
    "https://raw.githubusercontent.com/google-deepmind/"
    f"mujoco_menagerie/{MENAGERIE_COMMIT}"
)
MANIFEST_ENTRIES = (
    (
        "universal_robots_ur5e/assets/base_0.obj",
        "9a3b21f1524baef6af8a52d7af90b6e94b79f7e3c2b04a0f6581f74d22291b0b",
    ),
    (
        "universal_robots_ur5e/assets/base_1.obj",
        "cf7d54691a2c1eeda280955ebd6bd83c2dbecfc46fb53c10198f05d107511b50",
    ),
    (
        "universal_robots_ur5e/assets/forearm_0.obj",
        "64c049a55149c2ab355594d52302b53d3e34da3677ceaa1917c427cde7743146",
    ),
    (
        "universal_robots_ur5e/assets/forearm_1.obj",
        "a96fe203015cf532b193f414cd01268c7c36795e51a837dd33afd122334f81f2",
    ),
    (
        "universal_robots_ur5e/assets/forearm_2.obj",
        "05b436505fc595aeab26675590c2b56b88a65b2d52d6352096fed9f07cb4c4ca",
    ),
    (
        "universal_robots_ur5e/assets/forearm_3.obj",
        "fa31af19cfad7a765b62e8a2d5931cb014b88de84195a7850b4e5ee902112e9a",
    ),
    (
        "universal_robots_ur5e/assets/shoulder_0.obj",
        "477387a7e7312782a6040484a3c2c2fcee33f40109f419076e31b6f6e1692c68",
    ),
    (
        "universal_robots_ur5e/assets/shoulder_1.obj",
        "8537bba02804640ddf6e260f79fe66e895b3229ed3d44298a95e8e027eb5d435",
    ),
    (
        "universal_robots_ur5e/assets/shoulder_2.obj",
        "291714518416bb8f0bafbf123867db011ecbf5146a05245ca5893ee8c57d4e21",
    ),
    (
        "universal_robots_ur5e/assets/upperarm_0.obj",
        "14143d4f1175a7dbacc8e2f2f98f1cac5a2b2e238cd46cd03654d31f2610eb53",
    ),
    (
        "universal_robots_ur5e/assets/upperarm_1.obj",
        "272899e927f4a32b7c25e34560ec70654c0346cc8ed1c616c0555f3f53b32d14",
    ),
    (
        "universal_robots_ur5e/assets/upperarm_2.obj",
        "86c05350d60eb0e7e28a81b544267478b7db997baba41cef19088ebbeffecfa1",
    ),
    (
        "universal_robots_ur5e/assets/upperarm_3.obj",
        "b257dab441e6fab64ea03b4b87029d8a96554ad1a513cce944d56acba0ad948a",
    ),
    (
        "universal_robots_ur5e/assets/wrist1_0.obj",
        "7ef0c59b5c5d56a9902435c9a058b27d2a38c85fe2bae02d00a9284dd1f4d519",
    ),
    (
        "universal_robots_ur5e/assets/wrist1_1.obj",
        "ee6d5c562e6becef94109b0543329080d285d1aef521a01ed98e18a5bf1e8684",
    ),
    (
        "universal_robots_ur5e/assets/wrist1_2.obj",
        "88c66dc64f15391287147d50add37729a3f41dadd80b1f859cf2cb6d45daf32a",
    ),
    (
        "universal_robots_ur5e/assets/wrist2_0.obj",
        "2ac2e2e39d8f7005bc3b3ab26cd61634be2e14930074615da5318b9ee7c569a8",
    ),
    (
        "universal_robots_ur5e/assets/wrist2_1.obj",
        "a721d06dc61d3b3a8ec58c035c9e3358ed30d6991a72e077d8abf3c3c203b633",
    ),
    (
        "universal_robots_ur5e/assets/wrist2_2.obj",
        "6df4335c8bd5e881fda2c224f770115ffe9ddd0cd321ebe3c0a4a310796f0987",
    ),
    (
        "universal_robots_ur5e/assets/wrist3.obj",
        "0e51748f9c93bd66604360a5cf7e2e8a6e754550be64076333ff4fc19ec90c61",
    ),
    (
        "robotiq_2f85/assets/base.stl",
        "1c7b9f2bd92d705fc4e897c94e905973a3c05f406845e942229433deb7041453",
    ),
    (
        "robotiq_2f85/assets/base_mount.stl",
        "87971dfdd6e5311b5a723e7b73c4701ebf5c2364df671500f30dfc1c53c8e61a",
    ),
    (
        "robotiq_2f85/assets/coupler.stl",
        "d5ee95e62f8415bf5b6e503c831a958f5fc1990bf9b2865329ec38a28932727c",
    ),
    (
        "robotiq_2f85/assets/driver.stl",
        "bbd6e868b1778bead60d2516c7ede581d7ad4e431744b1828757d6ea5129c112",
    ),
    (
        "robotiq_2f85/assets/follower.stl",
        "633793332c081641ce200df27a40643cc293b29956e3cb5cb29cb811c33ef1c7",
    ),
    (
        "robotiq_2f85/assets/pad.stl",
        "4af1fa8d9bb285abecbf5dfc88e74cb30f059a7a074c2030dc35e9ee4316019d",
    ),
    (
        "robotiq_2f85/assets/silicone_pad.stl",
        "ca20c0fa61e6d3ce7b04bed25360e048e31dfd88ace04187aa6d9e8f4ca3fd0f",
    ),
    (
        "robotiq_2f85/assets/spring_link.stl",
        "4fb74a8a0d76c0e471cf19fd48bc676fd5b19123798e7a39eb6aa56869354283",
    ),
)
MANIFEST = dict(MANIFEST_ENTRIES)

FRUIT_COMMIT = "89186e044112db74ca455e86d7f020c67d06572a"
FRUIT_BASE_URL = (
    "https://raw.githubusercontent.com/SkyLineHXY/"
    f"Piper_Mujoco_Sim/{FRUIT_COMMIT}/manipulator_grasp/assets/fruit"
)
FRUIT_REPOSITORY_URL = (
    "https://raw.githubusercontent.com/SkyLineHXY/"
    f"Piper_Mujoco_Sim/{FRUIT_COMMIT}"
)
FRUIT_MANIFEST = {
    "Apple.STL": (
        "de617c6c6ad1140db80ca49544ea5bc04e5e08cd22fc47906e5ace2e29ea2252"
    ),
    "Banana.STL": (
        "e41942de3a58e94e34a4bdf2e43e5fd69e3c7e1deab1911b08fc949fd2d417bd"
    ),
    "mesh/red-apple.png": (
        "bc87705659763670d7aed130e271b41e7b9c54805f8a0d217e1e075b1611580b"
    ),
    "mesh/yellow-banana.png": (
        "3cc4e4ea24a2883af5445d4a6be6a1921e002496b21667642b5614996e28a8ff"
    ),
}
FRUIT_LICENSE_MANIFEST = {
    "LICENSE": (
        "554b385e65f5ab67d76c1edbf08f5d1d0f7fb6ae6ac11169153cd34ee643a54d"
    ),
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".part")
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=60) as response:
                with temporary.open("wb") as destination:
                    while True:
                        block = response.read(1024 * 1024)
                        if not block:
                            break
                        destination.write(block)
            os.replace(temporary, output)
            return
        except (OSError, urllib.error.URLError):
            temporary.unlink(missing_ok=True)
            if attempt == 2:
                raise
            time.sleep(attempt + 1)


def prepare_manifest(base_url, manifest, output_root, check_only=False):
    failures = []
    downloaded = 0
    for relative_path, expected_hash in manifest.items():
        output = output_root / relative_path
        if output.is_file() and sha256(output) == expected_hash:
            continue
        if check_only:
            failures.append(relative_path)
            continue
        print(f"[mujoco-assets] download {relative_path}", flush=True)
        download(f"{base_url}/{relative_path}", output)
        if sha256(output) != expected_hash:
            output.unlink(missing_ok=True)
            raise RuntimeError(f"checksum mismatch: {relative_path}")
        downloaded += 1
    if failures:
        raise RuntimeError("missing or invalid assets: " + ", ".join(failures))
    return downloaded


def prepare_assets(output_root, fruit_output_root, check_only=False):
    downloaded = prepare_manifest(
        BASE_URL, MANIFEST, output_root, check_only
    )
    downloaded += prepare_manifest(
        FRUIT_BASE_URL, FRUIT_MANIFEST, fruit_output_root, check_only
    )
    downloaded += prepare_manifest(
        FRUIT_REPOSITORY_URL,
        FRUIT_LICENSE_MANIFEST,
        fruit_output_root,
        check_only,
    )
    print(
        "[mujoco-assets] ready: "
        f"files={len(MANIFEST) + len(FRUIT_MANIFEST) + 1} "
        f"downloaded={downloaded}",
        flush=True,
    )


def main():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="prepare pinned mujoco simulation assets"
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_root / "simulation" / "mujoco" / "models",
    )
    parser.add_argument(
        "--fruit-output-root",
        type=Path,
        default=project_root / "simulation" / "mujoco" / "assets" / "fruit",
    )
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        prepare_assets(
            args.output_root.expanduser().resolve(),
            args.fruit_output_root.expanduser().resolve(),
            args.check,
        )
    except (OSError, RuntimeError, urllib.error.URLError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
