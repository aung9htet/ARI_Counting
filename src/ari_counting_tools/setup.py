from setuptools import setup
from glob import glob
from os.path import isfile
name = 'ari_counting_tools'
setup(name=name, version='0.1.0', packages=[name], data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]), ('share/'+name, ['package.xml'])] + [('share/'+name+'/'+d, [f for f in glob(d+'/*') if isfile(f)]) for d in ['launch','config','worlds']], install_requires=['setuptools'], zip_safe=True, maintainer='Aung', maintainer_email='aung@example.com', description='ARI experiment tools', license='Apache-2.0', entry_points={'console_scripts': ['gateway = ari_counting_tools.gateway:main', 'joint_control = ari_counting_tools.joint_control:main', 'ik_control = ari_counting_tools.ik_control:main', 'record = ari_counting_tools.recording:main', 'export_episode = ari_counting_tools.export:main', 'check = ari_counting_tools.check:main']})
