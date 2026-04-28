from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unitree_lib'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键：安装到 site-packages 根目录，使用 os.path.join 动态构建路径
        (os.path.join('lib', 'python3.10', 'site-packages'),
         glob('unitree_lib/unitree_lib/*.so')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Wrapper for Unitree motor SDK (ARM64)',
    license='Proprietary',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)