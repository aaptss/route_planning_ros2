from setuptools import setup

package_name = 'route_planner'

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
    maintainer='rosubuntu',
    maintainer_email='armanpts@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "npc_with_quest = route_planner.terminal_pts_fetcher:main",
            "path_planner = route_planner.path_planner:main",
            "user = route_planner.user:main"
        ],
    },
)
