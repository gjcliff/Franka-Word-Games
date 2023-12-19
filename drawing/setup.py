from setuptools import find_packages, setup

package_name = 'drawing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         [
             'package.xml',
             'launch/drawing.launch.xml',
             'launch/april_tag.launch.xml',
             'launch/image_proc.launch.py',
             'launch/ocr_game.launch.xml',
             'config/tag.yaml',
             'config/view_camera.rviz',
             'launch/game_time.launch.xml'
         ]
         ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Graham Clifford',
    maintainer_email='grahamclifford2024@u.northwestern.edu',
    description='This package controls the Emika Franka Panda robot and gets \
    it to pick up a piece of paper.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "draw = drawing.draw:main",
            "executor = drawing.send_trajectories:main",
            "tags = drawing.tags:Tags_entry",
            "kickstart = drawing.kickstart:main",
            "paddle_ocr = drawing.paddle_ocr:main",
            "hangman = drawing.hangman:main",
            "brain = drawing.brain:main",
            "image_modification = drawing.image_modification:main"
        ],
    },
)
