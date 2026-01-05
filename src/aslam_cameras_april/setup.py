from setuptools import setup

package_name = 'aslam_cameras_april'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'python'},
    install_requires=['setuptools'],
)
