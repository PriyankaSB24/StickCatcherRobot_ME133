from setuptools import find_packages, setup
from glob import glob

package_name = 'stickcatcher'
otherfiles = [
    ('share/' + package_name + '/urdf', glob('urdf/*')),
    ('share/' + package_name + '/rviz', glob('rviz/*')),
]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]+otherfiles,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a Final Project Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stick      = stickcatcher.stick:main',
            'catch      = stickcatcher.catch:main',
        ],
    },
)
