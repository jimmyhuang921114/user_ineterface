from setuptools import find_packages, setup

package_name = 'medicine_management_client'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hospital Admin',
    maintainer_email='admin@hospital.com',
    description='Medicine management client for automatic prescription polling',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'medicine_management_client = medicine_management_client.medicine_management_client:main',
        ],
    },
)