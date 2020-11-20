from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import setuptools

__version__ = '0.0.1'
# os.environ["CC"] = "g++-4.7" os.environ["CXX"] = "g++-4.7"

class get_pybind_include(object):
    """Helper class to determine the pybind11 include path

    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked. """

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)


ext_modules = [
    Extension(
        'log_reader',
        ['src/log_reader.cpp',
         '../../src/cltool.cpp',
         '../../src/cltool_main.cpp',
         '../../src/convert_ins.cpp',
         '../../src/com_manager.c',
         '../../src/data_sets.c',
         '../../src/DataChunk.cpp',
         '../../src/DataChunkSorted.cpp',
         '../../src/DataCSV.cpp',
         '../../src/DataJSON.cpp',
         '../../src/DataKML.cpp',
         '../../src/DeviceLog.cpp',
         '../../src/DeviceLogCSV.cpp',
         '../../src/DeviceLogJSON.cpp',
         '../../src/DeviceLogKML.cpp',
         '../../src/DeviceLogSerial.cpp',
         '../../src/DeviceLogSorted.cpp',
         '../../src/InertialSense.cpp',
         '../../src/inertialSenseBootLoader.c',
         '../../src/ISComm.c',
         '../../src/ISDataMappings.cpp',
         '../../src/ISDisplay.cpp',
         '../../src/ISEarth.c',
         '../../src/ISFileManager.cpp',
         '../../src/ISLogFile.cpp',
         '../../src/ISLogger.cpp',
         '../../src/ISMatrix.c',
         '../../src/ISPose.c',
         '../../src/ISSerialPort.cpp',
         '../../src/ISStream.cpp',
         '../../src/ISTcpClient.cpp',
         '../../src/ISTcpServer.cpp',
         '../../src/ISUtilities.cpp',
         '../../src/linked_list.c',
         '../../src/serialPort.c',
         '../../src/serialPortPlatform.c',
         '../../src/time_conversion.c',
         '../../src/tinystr.cpp',
         '../../src/tinyxml.cpp',
         '../../src/tinyxmlerror.cpp',
         '../../src/tinyxmlparser.cpp'],
        include_dirs=[
            # Path to pybind11 headers
            'include',
            '../src',
            '../../src',
            get_pybind_include(),
            get_pybind_include(user=True)
        ],
        language='c++'
    ),
]


# As of Python 3.6, CCompiler has a `has_flag` method.
# cf http://bugs.python.org/issue26689
def has_flag(compiler, flagname):
    """Return a boolean indicating whether a flag name is supported on
    the specified compiler.
    """
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.cpp') as f:
        f.write('int main (int argc, char **argv) { return 0; }')
        try:
            compiler.compile([f.name], extra_postargs=[flagname])
        except setuptools.distutils.errors.CompileError:
            return False
    return True


def cpp_flag(compiler):
    """Return the -std=c++[11/14] compiler flag.

    The c++14 is prefered over c++11 (when it is available).
    """
    if has_flag(compiler, '-std=c++14'):
        return '-std=c++14'
    elif has_flag(compiler, '-std=c++11'):
        return '-std=c++11'
    else:
        raise RuntimeError('Unsupported compiler -- at least C++11 support '
                           'is needed!')


class BuildExt(build_ext):
    """A custom build extension for adding compiler-specific options."""
    c_opts = {
        'msvc': ['/EHsc'],
        'unix': ['-O3'],
    }

    if sys.platform == 'darwin':
        c_opts['unix'] += ['-stdlib=libc++', '-mmacosx-version-min=10.7']

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        if ct == 'unix':
            opts.append('-DVERSION_INFO="%s"' % self.distribution.get_version())
            opts.append(cpp_flag(self.compiler))
#            if has_flag(self.compiler, '-fvisibility=hidden'):
#                opts.append('-fvisibility=hidden ')
        elif ct == 'msvc':
            opts.append('/DVERSION_INFO=\\"%s\\"' % self.distribution.get_version())
        for ext in self.extensions:
            ext.extra_compile_args = opts
        build_ext.build_extensions(self)

setup(
    name='log_reader',
    version=__version__,
    author='James Jackson',
    author_email='superjax08@gmail.com',
    description='pybind interface to reading InertialSense Log files',
    long_description='',
    ext_modules=ext_modules,
    install_requires=['pybind11>=2.2', 
        'pyqt5', 
        'numpy', 
        'matplotlib', 
        'pyyaml', 
        'pyserial', 
        'scipy', 
        'tqdm',
        'simplekml'],
    cmdclass={'build_ext': BuildExt},
    zip_safe=False,
)
