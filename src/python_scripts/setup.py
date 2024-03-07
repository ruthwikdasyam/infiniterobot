from setuptools import setup

package_name = 'python_scripts'

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
    maintainer='zahir',
    maintainer_email='zahir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'drift = python_scripts.drift:main',
            'autodrive = python_scripts.autodrive:main',
            'teleop = python_scripts.teleop:main',
            'publisher = python_scripts.publisher:main',
            'subscriber = python_scripts.subscriber:main'
        ],
    },
)
