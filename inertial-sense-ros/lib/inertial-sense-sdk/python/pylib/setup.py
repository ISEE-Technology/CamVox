'''
Created on Oct 19, 2015

@author: Walter
'''

try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension

from Cython.Distutils import build_ext

modules = [Extension("pose", ["pose.pyx"]),
           Extension("filterTools", ["filterTools.pyx"]),
           Extension("plotTools", ["plotTools.pyx"]),
           Extension("ISToolsData", ["ISToolsData.pyx"]),
           Extension("ISToolsDataSorted", ["ISToolsDataSorted.pyx"]),
#           Extension("ISDataAnalytics", ["ISDataAnalytics.pyx"]),
#           Extension("ISToolsPlot", ["ISToolsPlot.pyx"]),
           ]

setup(
    name = 'InertialSenseTools', 
    cmdclass = {'build_ext': build_ext}, 
    ext_modules = modules 
    )
