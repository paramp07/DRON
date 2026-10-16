from setuptools import setup

package_name = 'dron_capture'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if False else []),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DRON Visualization Team',
    maintainer_email='dev@dron.local',
    description='Synchronized stereo and radiometric thermal camera drivers for DRON',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_driver = dron_capture.stereo_driver:main',
            'thermal_driver = dron_capture.thermal_driver:main',
        ],
    },
)
