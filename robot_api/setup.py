from setuptools import setup


setup(
    name='robot_api',
    version='1.11.0',
    packages=['robot_api'],
    install_requires=['setuptools'],
    maintainer='Alexander Sung',
    maintainer_email='Alexander.Sung@dfki.de',
    description='A pre- and concise Python API to control robots with simple commands.',
    license='MIT',
    tests_require=['pytest'],
)
