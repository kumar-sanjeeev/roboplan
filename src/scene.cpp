#include <roboplan/scene.hpp>

namespace roboplan {

    Scene::Scene(const int ndof)
    {
        q_.resize(ndof);
        q_.setZero();
    }

    void Scene::print()
    {
        std::cout << "Scene has " << q_.size() << "DOF.\n  q: " << q_ << "\n";
    }

} // namespace roboplan
