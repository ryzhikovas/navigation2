/*********************************************************************
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 * Author: Andrey Ryzhikov
 *********************************************************************/
#include "nav2_costmap_2d/denoise_layer.hpp"

//<>

#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

void
DenoiseLayer::reset()
{
  current_ = false;
}

bool
DenoiseLayer::isClearable()
{
  return false;
}

void
DenoiseLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/,
  double * /*max_x*/, double * /*max_y*/) {}

void
DenoiseLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_x, int min_y, int max_x, int max_y)
{

  current_ = true;
}

void
DenoiseLayer::onInitialize()
{
  // Enable/disable plugin
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smaller groups should be filtered
  declareParameter("minimal_group_size", rclcpp::ParameterValue(2));
  // Pixels connectivity type
  declareParameter("group_connectivity_type", rclcpp::ParameterValue(8));

  // node != nullptr. If node_ is nullptr, declareParameter (...) will throw
  const auto node = node_.lock();
  node->get_parameter(name_ + "." + "enabled", enabled_);

  auto getInt = [&](const std::string & parameter_name) {
      int param{};
      node->get_parameter(name_ + "." + parameter_name, param);
      return param;
    };

  const int minimal_group_size_param = getInt("minimal_group_size");

  if (minimal_group_size_param <= 1) {
    RCLCPP_WARN(
      logger_,
      "DenoiseLayer::onInitialize(): param minimal_group_size: %i."
      " A value of 1 or less means that all map cells will be left as they are.",
      minimal_group_size_param);
    this->minimal_group_size = 1;
  } else {
    this->minimal_group_size = static_cast<size_t>(minimal_group_size_param);
  }

  const int group_connectivity_type_param = getInt("group_connectivity_type");

  if (group_connectivity_type_param == 4) {
    this->group_connectivity_type = ConnectivityType::Way4;
  } else if (group_connectivity_type_param == 8) {
    this->group_connectivity_type = ConnectivityType::Way8;
  } else {
    RCLCPP_WARN(
      logger_, "DenoiseLayer::onInitialize(): param group_connectivity_type: %i."
      " Possible values are  4 (neighbors pixels are connected horizontally and vertically) "
      "or 8 (neighbors pixels are connected horizontally, vertically and diagonally)."
      "The default value 8 will be used",
      group_connectivity_type_param);
    this->group_connectivity_type = ConnectivityType::Way8;
  }
  current_ = true;
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a DenoiseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DenoiseLayer, nav2_costmap_2d::Layer)
