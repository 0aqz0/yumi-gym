from setuptools import setup, find_packages
from pathlib import Path

setup(
    name='yumi-gym',
    version='0.0.1',
    packages=find_packages(),
    install_requires=['gym', 'pybullet', 'numpy'],
    description="Physics simulation for ABB's collaborative robot yumi",
    long_description_content_type="text/markdown",
    long_description=Path("README.md").read_text(),
    url='https://github.com/0aqz0/yumi-gym',
    author='Haodong Zhang',
    author_email='aqz@zju.edu.cn',
    license='MIT',
)