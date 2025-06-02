from setuptools import setup, find_packages

setup(
    name='aut_project',
    version='1.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'matplotlib'],
    author='Zsombor Ménes',
    description='DC motor simulation and control package',
)
