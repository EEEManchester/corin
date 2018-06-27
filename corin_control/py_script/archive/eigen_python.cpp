#include <stdexcept>
#include <iostream>
#include <string>
#include <sstream>
#include <python2.7/Python.h>
#include <eigen3/Eigen/Dense>


using std::size_t;
typedef double real_t;
// create type Matrix for MatrixXd
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> 
        Matrix;
// creates an object class
static PyObject* p_eigen_python_error(NULL);

static PyObject *
randomDxDMatrix(PyObject *self, PyObject *args) {
    PyObject* p(NULL);
    PyObject* item(NULL);    

    try{
        size_t d(0);

        PyArg_ParseTuple(args, "L", &d);
        Matrix M = Matrix::Random(d,d);

        size_t length = d * d;

        p = PyList_New(length);

        if (p == NULL) {
            std::stringstream msg;
            msg << "Could not allocate a Pylist of "
                << d << "x" << d << " = " << d*d 
                << " size for the return Object";
            throw std::runtime_error(msg.str().c_str());
        } else {
            for (size_t i = 0; i < length; ++i) {
                item = PyFloat_FromDouble(M.data()[i]);
                PyList_SET_ITEM(p, i, item);
            }   
        }

    } catch (const std::exception& e) {
        delete p; p = NULL;
        delete item; item = NULL;

        std::string msg = ("randomDxDMatrix failed: ");
        msg += e.what();
        PyErr_SetString(p_eigen_python_error, msg.c_str());
    }

    return p;
}
// Function prototype
static PyMethodDef EigenMethods[] = {
    {"randomDxDMatrix",  randomDxDMatrix, METH_VARARGS, 
    "Gets a random DxD matrix column-major as a list of (python) floats"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initeigen_python(void) {
    // create a pyton object
    PyObject* p;
    // Initialises the python object
    p = Py_InitModule("eigen_python", EigenMethods);
    if (p == NULL)
        return;
    // create a new python exception class
    p_eigen_python_error = PyErr_NewException(
                                const_cast<char*>("eigen_python.error"), 
                                NULL, NULL
                            );
    // increase object reference count
    Py_INCREF(p_eigen_python_error);
    // adds the error module to object p
    PyModule_AddObject(p, "error", p_eigen_python_error);
}