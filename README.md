# ROS ETH grid map

This repository demonstrate the usage of using ETH ROS grid map.

## Error

Error using `grid_map_filters`

```
SlidingWindowIterator cannot be used with grid maps that don't have a default buffer start index.
```

Solution:
```
std::unique_lock<std::recursive_mutex> lk(m_);
for(grid_map::GridMapIterator itr(raw_gridmap_); !itr.isPastEnd(); ++itr)
{
    // Replace nan with low value to detect gap
    if(std::isnan(raw_gridmap_.at(elevation_gridmap_layer_, *itr)))
        raw_gridmap_.at(elevation_gridmap_layer_, *itr) = low_value_;
}

if(!raw_gridmap_.isDefaultStartIndex())
    raw_gridmap_.convertToDefaultStartIndex();

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
