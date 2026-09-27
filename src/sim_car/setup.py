from setuptools import setup
from glob import glob
setup(name='sim_car',version='0.1.0',packages=['sim_car'],
 data_files=[('share/ament_index/resource_index/packages',['resource/sim_car']),('share/sim_car',['package.xml']),*[(f'share/sim_car/{d}',glob(d+'/*')) for d in ['config','urdf','launch','rviz']]],
 entry_points={'console_scripts':['simulator = sim_car.ros_node:main','viewer = sim_car.app:main']})
