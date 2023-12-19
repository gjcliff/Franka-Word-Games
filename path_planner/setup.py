from setuptools import find_packages, setup

package_name = 'path_planner'

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
    maintainer='Ishani Narwankar, Srikanth Schelbert, Abhishek Sankar, Ananya \
        Agarwal, Graham Clifford',
    maintainer_email='ishaninarwankar2024@u.northwestern.edu, \
        srikanthschelbert2024@u.northwestern.edu, \
        abhisheksankar2024@u.northwestern.edu, \
        ananyaagarwal2024@u.northwestern.edu \
        grahamclifford2024@u.northwestern.edu',
    description='Plans and executes paths for a robot arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
