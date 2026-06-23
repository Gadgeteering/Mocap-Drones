import os
import sys

# Windows DLL handling
if sys.platform == "win32":
    try:
        pkg_dir = os.path.dirname(__file__)

        dll_dirs = [
            os.path.join(pkg_dir, "_libs"),
            os.path.join(pkg_dir, "ext", "win", "lib"),
        ]

        for dll_dir in dll_dirs:
            if os.path.isdir(dll_dir):
                try:
                    os.add_dll_directory(dll_dir)
                except AttributeError:
                    os.environ["PATH"] = (
                        dll_dir + os.pathsep + os.environ.get("PATH", "")
                    )
                break
    except Exception:
        pass

# Linux/macOS/Raspberry Pi shared library handling
elif sys.platform.startswith(("linux", "darwin")):
    pkg_dir = os.path.dirname(__file__)

    lib_dirs = [
        os.path.join(pkg_dir, "_libs"),
        os.path.join(pkg_dir, "ext", "lib"),
    ]

    for lib_dir in lib_dirs:
        if os.path.isdir(lib_dir):
            os.environ["LD_LIBRARY_PATH"] = (
                lib_dir + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")
            )
            os.environ["DYLD_LIBRARY_PATH"] = (
                lib_dir + os.pathsep + os.environ.get("DYLD_LIBRARY_PATH", "")
            )

from .cameras import Camera, cam_count
from .ui import Display
from .io import Stream