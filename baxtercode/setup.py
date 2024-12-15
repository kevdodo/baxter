from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'baxtercode'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/meshes/base',              glob('meshes/base/*')),
        ('share/' + package_name + '/meshes/cable_connector',   glob('meshes/cable_connector/*')),
        ('share/' + package_name + '/meshes/f_distal/bt_2p',    glob('meshes/f_distal/bt_2p/*')),
        ('share/' + package_name + '/meshes/f_distal/bt_sp',    glob('meshes/f_distal/bt_sp/*')),
        ('share/' + package_name + '/meshes/f_distal/pst',      glob('meshes/f_distal/pst/*')),
        ('share/' + package_name + '/meshes/f_knuckle',         glob('meshes/f_knuckle/*')),
        ('share/' + package_name + '/meshes/f_middle',          glob('meshes/f_middle/*')),
        ('share/' + package_name + '/meshes/f_proximal',        glob('meshes/f_proximal/*')),
        ('share/' + package_name + '/meshes/forearm',           glob('meshes/forearm/*')),
        ('share/' + package_name + '/meshes/hand_wrist',        glob('meshes/hand_wrist/*')),
        ('share/' + package_name + '/meshes/head',              glob('meshes/head/*')),
        ('share/' + package_name + '/meshes/lf_metacarpal',     glob('meshes/lf_metacarpal/*')),
        ('share/' + package_name + '/meshes/lower_elbow',       glob('meshes/lower_elbow/*')),
        ('share/' + package_name + '/meshes/lower_forearm',     glob('meshes/lower_forearm/*')),
        ('share/' + package_name + '/meshes/lower_shoulder',    glob('meshes/lower_shoulder/*')),
        ('share/' + package_name + '/meshes/mounting_plate',    glob('meshes/mounting_plate/*')),
        ('share/' + package_name + '/meshes/palm',              glob('meshes/palm/*')),
        ('share/' + package_name + '/meshes/th_distal/bt_2p',   glob('meshes/th_distal/bt_2p/*')),
        ('share/' + package_name + '/meshes/th_distal/bt_sp',   glob('meshes/th_distal/bt_sp/*')),
        ('share/' + package_name + '/meshes/th_distal/pst',     glob('meshes/th_distal/pst/*')),
        ('share/' + package_name + '/meshes/th_middle',         glob('meshes/th_middle/*')),
        ('share/' + package_name + '/meshes/th_proximal',       glob('meshes/th_proximal/*')),
        ('share/' + package_name + '/meshes/torso',             glob('meshes/torso/*')),
        ('share/' + package_name + '/meshes/upper_elbow',       glob('meshes/upper_elbow/*')),
        ('share/' + package_name + '/meshes/upper_forearm',     glob('meshes/upper_forearm/*')),
        ('share/' + package_name + '/meshes/upper_shoulder',    glob('meshes/upper_shoulder/*')),
        ('share/' + package_name + '/meshes/wrist',             glob('meshes/wrist/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='The 133a baxtercode Code',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'baxhand_demo    = baxtercode.baxhand_demo:main',
            'baxhand_juggle  = baxtercode.baxhand_juggle:main',
        ],
    },
)
