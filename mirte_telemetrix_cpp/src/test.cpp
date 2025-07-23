#include <mirte_telemetrix_cpp/mirte-board.hpp>
#include <boost/python.hpp>
#include <mirte_telemetrix_cpp/hello_world.hpp>
#include <thread>
#include <chrono>
using namespace boost::python;

// struct World
// {
//     void set(std::string msg) { this->msg = msg; }
//     std::string greet() { return msg; }
//     std::string msg;
// };
// BOOST_PYTHON_MODULE(mirte_board)
// {
//     class_<World>("World")
//         .def("set", &World::set)
//         .def("greet", &World::greet);
// }
int main() {
  // Your code here
  Mirte_Board_pico board1; //(/*tmx, s_node*/);
  Mirte_Board_pcb pcb(std::make_shared<Mirte_Board_pico>(board1), "v06");
  std::shared_ptr<Mirte_Board> board = std::make_shared<Mirte_Board_pcb>(pcb);
  std::cout << board->resolvePin("GP0") << std::endl;
  auto x = board->resolveConnector("MC2-B");
  for (auto i : x) {
    std::cout << i.first << " " << i.second << std::endl;
  }
  Py_Initialize();
  try {
  boost::python::object main_module = boost::python::import("__main__");
  object main_namespace = main_module.attr("__dict__");
// object ignored = exec("result = 5 ** 2", main_namespace);
// int five_squared = extract<int>(main_namespace["result"]);
  object dunno = exec_file("test.py", main_namespace);
  
  // std::cout << "Python exec result: " << extract<int>(dunno) << std::endl;
  std::cout << "Python exec result: " << extract<int>(main_namespace["result"]) << std::endl;
  World x = extract<World>(main_namespace["x"]);
  std::cout << "Python object x: " << x.greet() << std::endl;
  // object loop = main_namespace["loop"];
  std::thread xthread([&]() {
    try {
      // object loop = main_namespace["loop"];
      while (true) {
        std::cout << "Python thread running..." << std::endl;
        auto out = eval("loop()", main_namespace);
        std::cout << "Python loop done" << std::endl;
        // std::cout << "Python loop output: " << extract<int>(out) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (error_already_set &) {
      PyErr_Print();
      std::cerr << "Error executing Python code in thread" << std::endl;
    }
  });
  // for(int i = 0; i < 10000; i) {
  //   auto out = eval("loop()", main_namespace);
  //   std::cout << "Python loop output: " << extract<int>(out) << std::endl;
  //   // std::cout << "Python loop iteration: " << i << std::endl;
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
  std::this_thread::sleep_for(std::chrono::seconds(4));
  // auto out = eval("upd(3)", main_namespace);
  // std::cout << "Python upd output: " << extract<int>(out) << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(100));
  std::cout << "done!" << std::endl;
  // std::cout << "Python object x: " << x << std::endl;
} catch (error_already_set &) {
    PyErr_Print();
    std::cerr << "Error executing Python code" << std::endl;
  }
  // Py_Finalize(); // Uncomment if you
  // std::cout << "5 squared is: " << five_squared << std::endl;
  // Py_Finalize();
  return 0;
}
