#include <boost/python.hpp>
using namespace boost::python;

#include <mirte_telemetrix_cpp/hello_world.hpp>
BOOST_PYTHON_MODULE(hello_world)
{
      Py_Initialize();

    class_<World>("World")
        .def("set", &World::set)
        .def("greet", &World::greet);
}