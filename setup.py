import os
import numpy
from Cython.Build import cythonize
from Cython.Compiler.Options import _directive_defaults
from setuptools import Extension, setup

_directive_defaults['linetrace'] = True
_directive_defaults['binding'] = True


def read(filename):
    path = os.path.join(os.path.dirname(__file__), filename)
    contents = open(path).read()
    return contents


extensions = [

    Extension("geometry_code", ["src\\geom\\*.pyx"],
              include_dirs=[numpy.get_include()],
              define_macros=[]),
]

setup(
    name         = "SpiderBUG",
    version      = "1.0.0",
    description  = "Code to test the SpiderBUG path-planner",
    long_description = read('README.md'),
    author       = "Johannes Weck",
    author_email = "johannes.weck@gmail.com",
    url          = "http://github.com/the-jojo/SpiderBUG",
    license      = "MIT",
    ext_modules  = cythonize(extensions),
    install_requires=['dill', 'matplotlib', 'numpy', 'pyzmq', 'networkx', 'pybullet',
                                                         'pyquaternion', 'dubins', 'Cython']
)
