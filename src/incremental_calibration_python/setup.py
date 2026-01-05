from setuptools import setup

package_name = 'incremental_calibration_python'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'python'},
    install_requires=['setuptools'],
)
