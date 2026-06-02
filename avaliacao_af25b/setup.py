from setuptools import find_packages, setup

package_name = 'avaliacao_af25b'

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
    maintainer='borg',
    maintainer_email='kiqueabreu@icloud.com',
    description='Avaliacao Final 25b - Robotica Computacional',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "q1 = avaliacao_af25b.q1:main",
        ],
    },
)