from setuptools import find_packages, setup

package_name = 'ocr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', "launch/ocr_game.launch.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi2001',
    maintainer_email='241abhishek@gmail.com',
    description='ROS package for OCR on a whiteboard',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ocr = ocr.ocr:main",
            "hangman = ocr.hangman:main",
            "image_modification = ocr.image_modification:main",
            "paddle_ocr = ocr.paddle_ocr:main",
            "tesseract_ocr = ocr.tesseract_ocr:main"
        ],
    },
)
