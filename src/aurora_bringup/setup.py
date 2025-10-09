
from setuptools import setup
from glob import glob
import os

package_name = 'aurora_bringup'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for sub in ['launch', 'config', 'rviz']:
    files = package_files(sub)
    if files:
        data_files.append(('share/{}/{}'.format(package_name, sub), files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daniel Henning',
    author_email='daniel.h.1896@googlemail.com',
    maintainer='Daniel Henning',
    maintainer_email='daniel.h.1896@googlemail.com',
    description='Unified bringup for AURORA - DevDrone',
    license='BSD-3-Clause',
    tests_require=['pytest'],
)
