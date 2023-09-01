from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wur_osu_apples23'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miranda',
    maintainer_email='miranda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'send_twist = wur_osu_apples23.send_twist:main',
                'draw_square = wur_osu_apples23.draw_square:main',
                'heuristic_controller = wur_osu_apples23.heuristic_controller:main',
                'pose_listener = wur_osu_apples23.pose_listener:main',
                'simulate_spring = wur_osu_apples23.simulate_spring:main',
                'run_trials = wur_osu_apples23.picking_trials:main',
                'measurement_listener = wur_osu_apples23.measurement_listener:main',
                'pull_twist_controller = wur_osu_apples23.pull_twist_controller:main',
                'input_tester = wur_osu_apples23.input_tester:main',
                'recorder = wur_osu_apples23.recorder:main',
                'force_filter = wur_osu_apples23.force_filter:main'
        ],
    },
)
