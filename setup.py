from setuptools import setup, find_packages
from pathlib import Path
import os

directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'yumi_gym', 'envs', 'assets')
data_files = []

for root, dirs, files in os.walk(directory):
    for file in files:
        data_files.append(os.path.join(root, file))

setup(
    name='yumi-gym',
    version='0.0.3',
    packages=find_packages(),
    package_data={'yumi_gym': data_files},
    include_package_data=True,
    install_requires=['gym', 'pybullet', 'numpy'],
    description="Physics simulation for ABB's collaborative robot yumi",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    url='https://github.com/0aqz0/yumi-gym',
    author='Haodong Zhang',
    author_email='aqz@zju.edu.cn',
    license='MIT',
)