from setuptools import find_packages, setup

package_name = 'ir_sen'

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
    maintainer='dev',
    maintainer_email='fallslarissa12@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ir_sen.publisher_member_function:main',
            'listener = ir_sen.subscriber_member_function:main',
            'test = ir_sen.test_publisher_member_function:main'
        ],
    },
)
