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
            << " gap filter failed to update."
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

```cpp
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
                << " Failed to start gap filter."
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
        private_nh_.param<std::string>("gap/filter_chain_parameter_name", filter_chain_parameter_name_, "gap/grid_map_filters");
    }
```
