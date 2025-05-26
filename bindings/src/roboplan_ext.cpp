#include <iostream>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <roboplan/scene.hpp>
#include <roboplan/utils.hpp>
namespace roboplan {

NB_MODULE(roboplan, m) {
    m.def("add", &add);
    m.def("createPinocchioModel", &createPinocchioModel);

    nanobind::class_<Scene>(m, "Scene")
        .def(nanobind::init<const int>())
        .def("print", &Scene::print);
}

}  // namespace roboplan
