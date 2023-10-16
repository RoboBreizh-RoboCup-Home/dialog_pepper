from setuptools import find_packages, setup
import os

package_name = 'dialog_pepper'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='scripts', exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maelic',
    maintainer_email='teoneau@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboNLU_demo = scripts.roboNLU_demo:main',
            'SpeechToText = scripts.SpeechToText:main',
            'TranscriptIntent = scripts.TranscriptIntent:main',
        ],
    },
)

