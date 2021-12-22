from setuptools import setup, find_packages

from os import path

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    readme = f.read()

setup(
    name='commonroad-vehicle-models',
    version='2.0.0',
    description='Implementation of vehicle models with varying abstraction levels ranging from kinematic single track model to a multi-body model.',
    keywords='autonomous automated vehicles driving motion planning',
    url='https://commonroad.in.tum.de/',
    author='Cyber-Physical Systems Group, Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    packages=find_packages(exclude=['scripts']),
    long_description_content_type='text/markdown',
    long_description=readme,
    classifiers=[
        "Programming Language :: Python :: 3.6",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ],
    data_files=[('.', ['LICENSE.txt'])],
    include_package_data=True,
)
