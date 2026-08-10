from setuptools import find_packages, setup

package_name = 'cv_watchdog'

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
    maintainer='yusuf',
    maintainer_email='yusuf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_pub = cv_watchdog.ip_cam:main',
            'cam_sub = cv_watchdog.ip_cam_reader:main '
        ],
    },
)
