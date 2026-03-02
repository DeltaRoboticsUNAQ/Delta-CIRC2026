from setuptools import find_packages, setup

package_name = 'arm_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stc',
    maintainer_email='saidtorrescervantes@gmail.com',
    description='Tasks CIRC 2026',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'task_manager = arm_tasks.task_manager:main',
        ],
    },
)
