from setuptools import find_packages, setup

package_name = "ssros_ts"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kihochoi",
    maintainer_email="kiho.choi@lge.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = ssros_ts.publisher_member_function:main",
            "listener = ssros_ts.subscriber_member_function:main",
        ],
    },
)
