from setuptools import find_packages, setup

package_name = 'Agri'

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
    maintainer='naren',
    maintainer_email='naren@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joystickcontrol = Agri.joystickcontrol:main",
            "motorcontrol = Agri.motorcontrol:main",
            #"fields2cover_nav2_node = Agri.fields2cover_nav2_node:main",
        ],
    },
)
