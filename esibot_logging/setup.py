from setuptools import find_packages, setup

package_name = "esibot_logging"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed El Amine Kherroubi",
    maintainer_email="mm_kherroubi@esi.dz",
    description="Shared logging utilities for EsiBot packages.",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
)
