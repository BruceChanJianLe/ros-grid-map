# ROS ETH grid map

This repository demonstrate the usage of using ETH ROS grid map.

## Error

### 1.

Error using `grid_map_filters`

```bash
SlidingWindowIterator cannot be used with grid maps that don't have a default buffer start index.
```

Solution:
```cpp
std::unique_lock<std::recursive_mutex> lk(m_);
for(grid_map::GridMapIterator itr(raw_gridmap_); !itr.isPastEnd(); ++itr)
{
    // Replace nan with low value to detect gap
    if(std::isnan(raw_gridmap_.at(elevation_gridmap_layer_, *itr)))
        raw_gridmap_.at(elevation_gridmap_layer_, *itr) = low_value_;
}

// solution start
// This error happens because the default start index is not (0, 0)
// Which we can convert by using the below method.
// Don't set the startIndex manually as it will corrupt the grid_map
if(!raw_gridmap_.isDefaultStartIndex())
    raw_gridmap_.convertToDefaultStartIndex();
// solution end

try
{
    // Apply filter
    if(!filters_.update(raw_gridmap_, gap_gridmap_))
    {
        ROS_WARN_STREAM(
            ros::this_node::getName()
            << " "
            << __func__
            << " example filter failed to update."
        );
        update_ = false;
        return;
    }
}
catch(const std::exception & e)
{
    ROS_ERROR_STREAM(
        ros::this_node::getName()
        << " "
        << __func__
        << " caught error: "
        << e.what()
    );
}
first_time_ = update_ = false;
```

[reference](https://github.com/ANYbotics/elevation_mapping/issues/190)

### 2.

Error using `grid_map_filter`

Did not get the params set for filters.

Solution
```cpp
    filters::FilterChain<grid_map::GridMap> filters_("grid_map::GridMap");

    void exampleFilter::onInit()
    {
        global_nh_ = this->getMTNodeHandle();
        private_nh_ = this->getPrivateNodeHandle();

        loadROSParams();

        // if(!filters_.configure(filter_chain_parameter_name_, ""))
        if(!filters_.configure(filter_chain_parameter_name_, private_nh_))
        {
            ROS_ERROR_STREAM(
                ros::this_node::getName()
                << " Failed to start example filter."
            );
            return;
        }

        ROS_INFO_STREAM(
            ros::this_node::getName()
            << " Started example filter."
        );
    }

    void exampleFilter::loadROSParams()
    {
        private_nh_.param<std::string>("example/filter_chain_parameter_name", filter_chain_parameter_name_, "example/grid_map_filters");
    }
```

Use private nodehandle for configuring the filter chain object.

```yaml
example:
  rate: 5
  debug: true
  normalize: true
  filter_chain_parameter_name: "edge/grid_map_filters"
  grid_map_filters:
    # Reduce noise with a radial blurring filter.
    - name: mean_in_radius
      type: gridMapFilters/MeanInRadiusFilter
      params:
        input_layer: elevation
        output_layer: elevation_smooth
        radius: 0.06

    # Compute surface normals.
    - name: surface_normals
      type: gridMapFilters/NormalVectorsFilter
      params:
        input_layer: elevation_smooth
        output_layers_prefix: normal_vectors_
        radius: 0.05
        normal_vector_positive_axis: z

    # Compute slope from surface normal.
    - name: slope
      type: gridMapFilters/MathExpressionFilter
      params:
        # input_layer: elevation
        output_layer: slope
        expression: acos(normal_vectors_z)
      # Edge detection by computing the standard deviation from slope.

    - name: edge_detection
      type: gridMapFilters/SlidingWindowMathExpressionFilter
      params:
        input_layer: slope
        output_layer: edges
        expression: sqrt(sumOfFinites(square(slope - meanOfFinites(slope))) ./ numberOfFinites(slope)) # Standard deviation
        compute_empty_cells: false
        edge_handling: crop # options: inside, crop, empty, mean
        window_length: 0.05
```
