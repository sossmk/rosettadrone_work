from glob import glob

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
        # glob 이다 — 내일 보정으로 생길 config/camera.yaml 을 빠뜨리지 않으려고
        ("share/" + package_name + "/config", glob("config/*.yaml")),
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
            "tag_follow_node = rd_bridge.tag_follow_node:main",
            "usv_follow_node = rd_bridge.usv_follow_node:main",
        ],
    },
)
