from distutils.core import setup, Extension

cpp_args = ['-Wall', '-pedantic']
cxx_args = ['-std=c++11'].extend(cpp_args)

module_eigen_python = Extension('eigen_python',
                          define_macros = [('MAJOR_VERSION', '0'),
                                            ('MINOR_VERSION', '1')],
                          include_dirs = ['/usr/local/include'],
                          sources = ['compute_com.cpp'],
                          extra_compile_args = cpp_args
#                          sources = ['eigen_python.cxx'],
#                          extra_compile_args = cxx_args
                      )

setup (name = 'eigen_python',
       version = '0.1',
       description = 'This is just a demo',
       author = 'Solkar',
       url = 'http://stackoverflow.com/questions' 
         + '/15573557/call-c-using-eigen-library-function-in-python',
       long_description = 'just a toy',
       ext_modules = [module_eigen_python])