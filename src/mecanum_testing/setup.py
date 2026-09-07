from setuptools import find_packages, setup


package_name = 'mecanum_testing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Esat',
    maintainer_email='esat@example.com',
    description='Repo sozlesme denetimi (contract_audit) ve teker/hareket dogrulama (motion_probe)',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contract_audit = mecanum_testing.contract_audit:main',
            'motion_probe = mecanum_testing.motion_probe:main',
        ],
    },
)
