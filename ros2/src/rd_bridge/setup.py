from setuptools import setup

package_name = "rd_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="김성민",
    maintainer_email="rlatjdals6142@gmail.com",
    description="RosettaDrone ↔ ROS 2 브리지 (영상 발행 · cmd_vel 구독)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "video_node = rd_bridge.video_node:main",
            "link_node = rd_bridge.link_node:main",
            "nav_node = rd_bridge.nav_node:main",
        ],
    },
)
