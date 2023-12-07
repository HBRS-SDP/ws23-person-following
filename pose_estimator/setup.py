from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'pose_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*.py'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swithin',
    maintainer_email='swithin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'estimate_pose=pose_estimator.estimate_pose:main',
        'estimate_pose_realsense=pose_estimator.estimate_pose_realsense:main',
        'follow_bot=pose_estimator.follow_bot:main',
        ],
    },
)
