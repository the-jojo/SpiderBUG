from setuptools import Extension, setup
from Cython.Build import cythonize
import numpy

from Cython.Compiler.Options import _directive_defaults

_directive_defaults['linetrace'] = True
_directive_defaults['binding'] = True

extensions = [

    Extension("geometry_code", ["src\\geom\\*.pyx"],
              include_dirs=[numpy.get_include()],
              define_macros=[]),
]

setup(
    name="SpiderBUG",
    ext_modules=cythonize(extensions), install_requires=['dill', 'matplotlib', 'numpy', 'pyzmq', 'networkx', 'pybullet',
                                                         'pyquaternion', 'dubins', 'Cython']
)
