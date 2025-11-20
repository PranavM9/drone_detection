from setuptools import setup

package_name = 'drone_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PranavM9',
    maintainer_email='pranavmoolapattu@gmail.com',
    description='AprilTag listener node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tag_listener = drone_vision.tag_listener:main',
        ],
    },
)
