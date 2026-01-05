from setuptools import setup

package_name = 'python_module'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'python'},
    install_requires=['setuptools'],
)
