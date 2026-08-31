from setuptools import find_packages, setup

setup(name="arena_autonomy", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/arena_autonomy"]),
                  ("share/arena_autonomy", ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="GIST ISAAC Robotics", maintainer_email="leejinh0225@users.noreply.github.com",
      description="Sensor-only baseline wall following for IT ARENA.", license="Apache-2.0",
      entry_points={"console_scripts": ["wall_follow = arena_autonomy.wall_follow:main",
                                         "tof_safety = arena_autonomy.tof_safety:main"]})
