from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension(
        "franka_ik",
        ["franka_ik_pybind.cpp"],
        include_dirs=[".", "./Eigen"],  # Add your include directories here
    ),
]

setup(
    cmdclass={"build_ext": build_ext},
    ext_modules=ext_modules,
)
