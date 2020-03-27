from setuptools import setup

package_name = 'tpk4128_opencv_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorgen',
    maintainer_email='jorgen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_camera_node = tpk4128_opencv_camera.opencv_camera_node:main'
        ],
    },
)
