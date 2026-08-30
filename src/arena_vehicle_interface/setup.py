from setuptools import find_packages, setup


package_name = "arena_vehicle_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="GIST ISAAC Robotics",
    maintainer_email="leejinh0225@users.noreply.github.com",
    description="Stable command adapters for the IT ARENA vehicle.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ackermann_to_twist = arena_vehicle_interface.ackermann_to_twist:main",
            "sim_wheel_encoder = arena_vehicle_interface.sim_wheel_encoder:main",
        ],
    },
)
