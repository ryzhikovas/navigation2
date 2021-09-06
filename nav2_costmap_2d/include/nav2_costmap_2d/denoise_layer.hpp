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
#ifndef NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
#define NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_

// <>

#include "nav2_costmap_2d/layer.hpp"

namespace nav2_costmap_2d
{
/**
 * @class DenoiseLayer
 * @brief Layer filters noise-induced freestanding obstacles
 * (white costmap pixels) or small obstacles groups
 */
class DenoiseLayer : public Layer
{

private:
  /// Pixels connectivity type (is the way in which pixels in image relate to
  /// their neighbors)
  enum class ConnectivityType: int
  {
    /// neighbors pixels are connected horizontally and vertically
    Way4 = 4,
    /// neighbors pixels are connected horizontally, vertically and diagonally
    Way8 = 8
  };

public:
  DenoiseLayer() = default;
  ~DenoiseLayer() = default;

  /**
   * @brief Reset this layer
   */
  void reset() override;

  /**
   * @brief Reports that no clearing operation is required
   */
  bool isClearable() override;

  /**
   * @brief Reports that no expansion is required
   *
   * The method is called to ask the plugin: which area of costmap it needs to update.
   * A layer is essentially a filter, so it never needs to expand bounds.
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Filters noise-induced obstacles in the selected region of the costmap
   *
   * The method is called when costmap recalculation is required.
   * It updates the costmap within its window bounds.
   * @param master_grid The master costmap grid to update
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_x, int min_y, int max_x, int max_y) override;

protected:
  /**
   * @brief Initializes the layer on startup
   *
   * This method is called at the end of plugin initialization.
   * Reads plugin parameters from a config file
   */
  void onInitialize() override;

private:
  // Pixels connectivity type. Determines how pixels belonging to the same group can be arranged
  size_t minimal_group_size{};
  // The border value of group size. Groups of this and larger size will be kept
  ConnectivityType group_connectivity_type{ConnectivityType::Way8};
  const uint8_t empty_cell_value = 0;
  const uint8_t filled_cell_value = 255;
};

template<class SourceElement, class TargetElement, class Converter>
void DenoiseLayer::convert(const cv::Mat & source, cv::Mat & target, Converter operation) const
{
  checkImagesSizesEqual(source, target, "DenoiseLayer::convert. The source and target");

  for (int row = 0; row < source.rows; ++row) {
    const auto src_begin = source.ptr<const SourceElement>(row);
    const auto src_end = src_begin + source.cols;
    auto trg = target.ptr<TargetElement>(row);

    for (auto src = src_begin; src != src_end; ++src, ++trg) {
      operation(*src, *trg);
    }
  }
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
