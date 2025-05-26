#pragma once

#include <iostream>
#include <string>

#include <Eigen/Dense>

namespace roboplan
{
    /// @brief Primary scene representation for planning and control.
    class Scene
    {
    public:
        /// @brief Basic constructor
        Scene(const int ndof);

        /// @brief Prints basic information
        void print();

    private:
        Eigen::VectorXd q_;
    };

} // namespace roboplan
