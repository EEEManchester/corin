# Corin

## Overview

This is a python library with [ROS] interface to control the Corin hexapod. There are some C++ tools but these are not used.

Features:

* **ROS interface:** The robot and the core modules of the library interfaces to ROS. A number of library functions are able to run independantly.
* **Visualizations:** The robot can be visualised in RViZ and Gazebo.
* **Physics Engine:** The gazebo simulator enables evaluation of motion control algorithms in real-world scenarios.

The Corin package has been tested with [ROS] Kinetic (under Ubuntu 16.04). This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Wei Cheah<br />
Maintainer: Wei Cheah, wei.cheah@manchester.ac.uk<br />
With contributions by: Hassan Hakim Khalili<br />
Robotics for Extreme Environment Group, University of Manchester**

## Documentation

TODO

## Installation

### Building from Source

#### Dependencies

The Corin package depends on the following libraries:

    sudo apt-get install libeigen3-dev

The other packages depend additionally on the [ROS] standard installation. 

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_ws/src
    git clone https://github.com/ethz-asl/grid_map.git
    cd ../
    catkin_make

### Packages Overview

This repository consists of following packages:

* ***corin*** is the meta-package for the Corin library.
* ***corin_control*** implements the algorithms of the Corin library. 
* ***corin_description*** contains the urdf description of Corin and the RViZ visualizer launch file.
* ***corin_gazebo*** contains the launch file for the gazebo simulator and the different worlds.
* ***corin_manager*** is the node to interface with the Dynamixel motors on Corin using ROBOTIS-Framework package.
* ***corin_rviz_plugin*** is an [RViz] plugin for user control interface.

## Usage

### Demonstrations



        roslaunch grid_map_demos simple_demo.launch

* *[tutorial_demo](grid_map_demos/src/tutorial_demo_node.cpp)* is an extended demonstration of the library's functionalities. Launch the *tutorial_demo* with

        roslaunch grid_map_demos tutorial_demo.launch

* *[iterators_demo](grid_map_demos/src/IteratorsDemo.cpp)* showcases the usage of the grid map iterators. Launch it with

        roslaunch grid_map_demos iterators_demo.launch

* *[image_to_gridmap_demo](grid_map_demos/src/ImageToGridmapDemo.cpp)* demonstrates how to convert data from an [image](grid_map_demos/data/eth_logo.png) to a grid map. Start the demonstration with

        roslaunch grid_map_demos image_to_gridmap_demo.launch

    ![Image to grid map demo result](grid_map_demos/doc/image_to_grid_map_demo_result.png)

* *[opencv_demo](grid_map_demos/src/opencv_demo_node.cpp)* demonstrates map manipulations with help of [OpenCV] functions. Start the demonstration with

        roslaunch grid_map_demos opencv_demo.launch

    ![OpenCV demo result](grid_map_demos/doc/opencv_demo_result.gif)

* *[resolution_change_demo](grid_map_demos/src/resolution_change_demo_node.cpp)* shows how the resolution of a grid map can be changed with help of the [OpenCV] image scaling methods. The see the results, use

        roslaunch grid_map_demos resolution_change_demo.launch

* *[filters_demo](grid_map_demos/src/FiltersDemo.cpp)* uses a chain of [ROS Filters] to process a grid map. Starting from the elevation of a terrain map, the demo uses several filters to show how to compute surface normals, use inpainting to fill holes, smoothen/blur the map, and use math expressions to detect edges, compute roughness and traversability. The filter chain setup is configured in the [`filters_demo_filter_chain.yaml`](grid_map_demos/config/filters_demo_filter_chain.yaml) file. Launch the demo with

        roslaunch grid_map_demos filters_demo.launch

    [![Filters demo results](grid_map_demos/doc/filters_demo_preview.gif)](grid_map_demos/doc/filters_demo.gif)

For more information about grid map filters, see [grid_map_filters](#grid_map_filters).

### Conventions & Definitions

[![Grid map layers](grid_map_core/doc/grid_map_layers.png)](grid_map_core/doc/grid_map_layers.pdf)

[![Grid map conventions](grid_map_core/doc/grid_map_conventions.png)](grid_map_core/doc/grid_map_conventions.pdf)


### Iterators

The grid map library contains various iterators for convenience.

Grid map | Submap | Circle | Line | Polygon
:---: | :---: | :---: | :---: | :---:
[![Grid map iterator](grid_map_core/doc/iterators/grid_map_iterator_preview.gif)](grid_map_core/doc/iterators/grid_map_iterator.gif) | [![Submap iterator](grid_map_core/doc/iterators/submap_iterator_preview.gif)](grid_map_core/doc/iterators/submap_iterator.gif) | [![Circle iterator](grid_map_core/doc/iterators/circle_iterator_preview.gif)](grid_map_core/doc/iterators/circle_iterator.gif) | [![Line iterator](grid_map_core/doc/iterators/line_iterator_preview.gif)](grid_map_core/doc/iterators/line_iterator.gif) | [![Polygon iterator](grid_map_core/doc/iterators/polygon_iterator_preview.gif)](grid_map_core/doc/iterators/polygon_iterator.gif)
__Ellipse__ | __Spiral__
[![Ellipse iterator](grid_map_core/doc/iterators/ellipse_iterator_preview.gif)](grid_map_core/doc/iterators/ellipse_iterator.gif) | [![Spiral iterator](grid_map_core/doc/iterators/spiral_iterator_preview.gif)](grid_map_core/doc/iterators/spiral_iterator.gif)

Using the iterator in a `for` loop is common. For example, iterate over the entire grid map with the `GridMapIterator` with

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        cout << "The value at index " << (*iterator).transpose() << " is " << map.at("layer", *iterator) << endl;
    }

The other grid map iterators follow the same form. You can find more examples on how to use the different iterators in the *[iterators_demo](grid_map_demos/src/IteratorsDemo.cpp)* node.

Note: For maximum efficiency when using iterators, it is recommended to locally store direct access to the data layers of the grid map with `grid_map::Matrix& data = map["layer"]` outside the `for` loop:

    grid_map::Matrix& data = map["layer"];
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << endl;
    }

You can find a benchmarking of the performance of the iterators in the `iterator_benchmark` node of the `grid_map_demos` package which can be run with

    rosrun grid_map_demos iterator_benchmark

Beware that while iterators are convenient, it is often the cleanest and most efficient to make use of the built-in [Eigen] methods. Here are some examples:

* Setting a constant value to all cells of a layer:

        map["layer"].setConstant(3.0);

* Adding two layers:

        map["sum"] = map["layer_1"] + map["layer_2"];

* Scaling a layer:

        map["layer"] = 2.0 * map["layer"];

* Max. values between two layers:

        map["max"] = map["layer_1"].cwiseMax(map["layer_2"]);

* Compute the root mean squared error:

        map.add("error", (map.get("layer_1") - map.get("layer_2")).cwiseAbs());
        unsigned int nCells = map.getSize().prod();
        double rootMeanSquaredError = sqrt((map["error"].array().pow(2).sum()) / nCells);


### Changing the Position of the Map

There are two different methods to change the position of the map:
* `setPosition(...)`: Changes the position of the map without changing data stored in the map. This changes the corresponce between the data and the map frame.
* `move(...)`: Relocates the grid map such that the corresponce between data and the map frame does not change. Data in the overlapping region before and after the position change remains stored. Data that falls outside of the map at its new position is discarded. Cells that cover previously unknown regions are emptied (set to nan). The data storage is implemented as two-dimensional circular buffer to minimize computational effort.

`setPosition(...)` | `move(...)`
:---: | :---:
![Grid map iterator](grid_map_core/doc/setposition_method.gif) | ![Submap iterator](grid_map_core/doc/move_method.gif)|


## Packages

### grid_map_rviz_plugin

This [RViz] plugin visualizes a grid map layer as 3d surface plot (height map). A separate layer can be chosen as layer for the color information.

![Grid map visualization in RViz](grid_map_rviz_plugin/doc/grid_map_rviz_plugin.png)


### grid_map_visualization

This node subscribes to a topic of type [grid_map_msgs/GridMap] and publishes messages that can be visualized in [RViz]. The published topics of the visualizer can be fully configure with a YAML parameter file. Any number of visualizations with different parameters can be added. An example is [here](grid_map_demos/config/tutorial_demo.yaml) for the configuration file of the *tutorial_demo*.

Point cloud | Vectors | Occupancy grid | Grid cells
--- | --- | --- | ---
[![Point cloud](grid_map_visualization/doc/point_cloud_preview.jpg)](grid_map_visualization/doc/point_cloud.jpg) | [![Vectors](grid_map_visualization/doc/vectors_preview.jpg)](grid_map_visualization/doc/vectors.jpg) | [![Occupancy grid](grid_map_visualization/doc/occupancy_grid_preview.jpg)](grid_map_visualization/doc/occupancy_grid.jpg) | [![Grid cells](grid_map_visualization/doc/grid_cells_preview.jpg)](grid_map_visualization/doc/grid_cells.jpg)

#### Parameters

* **`grid_map_topic`** (string, default: "/grid_map")

    The name of the grid map topic to be visualized. See below for the description of the visualizers.


#### Subscribed Topics

* **`/grid_map`** ([grid_map_msgs/GridMap])

    The grid map to visualize.


#### Published Topics

The published topics are configured with the [YAML parameter file](grid_map_demos/config/tutorial_demo.yaml). Possible topics are:

* **`point_cloud`** ([sensor_msgs/PointCloud2])

    Shows the grid map as a point cloud. Select which layer to transform as points with the `layer` parameter.

        name: elevation
        type: point_cloud
        params:
         layer: elevation
         flat: false # optional

* **`flat_point_cloud`** ([sensor_msgs/PointCloud2])

    Shows the grid map as a "flat" point cloud, i.e. with all points at the same height *z*. This is convenient to visualize 2d maps or images (or even video streams) in [RViz] with help of its `Color Transformer`. The parameter `height` determines the desired *z*-position of the flat point cloud.

        name: flat_grid
        type: flat_point_cloud
        params:
         height: 0.0

    Note: In order to omit points in the flat point cloud from empty/invalid cells, specify the layers which should be checked for validity with `setBasicLayers(...)`.

* **`vectors`** ([visualization_msgs/Marker])

    Visualizes vector data of the grid map as visual markers. Specify the layers which hold the *x*-, *y*-, and *z*-components of the vectors with the `layer_prefix` parameter. The parameter `position_layer` defines the layer to be used as start point of the vectors.

        name: surface_normals
        type: vectors
        params:
         layer_prefix: normal_
         position_layer: elevation
         scale: 0.06
         line_width: 0.005
         color: 15600153 # red

* **`occupancy_grid`** ([nav_msgs/OccupancyGrid])

    Visualizes a layer of the grid map as occupancy grid. Specify the layer to be visualized with the `layer` parameter, and the upper and lower bound with `data_min` and `data_max`.

        name: traversability_grid
        type: occupancy_grid
        params:
         layer: traversability
         data_min: -0.15
         data_max: 0.15

* **`grid_cells`** ([nav_msgs/GridCells])

    Visualizes a layer of the grid map as grid cells. Specify the layer to be visualized with the `layer` parameter, and the upper and lower bounds with `lower_threshold` and `upper_threshold`.

        name: elevation_cells
        type: grid_cells
        params:
         layer: elevation
         lower_threshold: -0.08 # optional, default: -inf
         upper_threshold: 0.08 # optional, default: inf

* **`region`** ([visualization_msgs/Marker])

    Shows the boundary of the grid map.

        name: map_region
        type: map_region
        params:
         color: 3289650
         line_width: 0.003

*Note: Color values are in RGB form as concatenated integers (for each channel value 0-255). The values can be generated like [this](http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D0%2C+g%3D255%2C+b%3D0%7D) as an example for the color green (red: 0, green: 255, blue: 0).*

### grid_map_filters

The *grid_map_filters* package containts several filters which can be applied a grid map to perform computations on the data in the layers. The grid map filters are based on [ROS Filters], which means that a chain of filters can be configured as a YAML file. Furthermore, additional filters can be written and made available through the ROS plugin mechanism, such as the [`InpaintFilter`](grid_map_cv/include/grid_map_cv/InpaintFilter.hpp) from the `grid_map_cv` package.

Several basic filters are provided in the *grid_map_filters* package:

* **`gridMapFilters/ThresholdFilter`**

    Set values below/above a threshold to a specified value.

        name: lower_threshold
        type: gridMapFilters/ThresholdFilter
        params:
          layer: layer_name
          lower_threshold: 0.0 # alternative: upper_threshold
          set_to: 0.0 # # Other uses: .nan, .inf

* **`gridMapFilters/MeanInRadiusFilter`**

    Compute for each cell of a layer the mean value inside a radius.

        name: mean_in_radius
        type: gridMapFilters/MeanInRadiusFilter
        params:
          input_layer: input
          output_layer: output
          radius: 0.06 # in m.

* **`gridMapFilters/NormalVectorsFilter`**

    Compute the normal vectors of a layer in a map.

        name: surface_normals
        type: gridMapFilters/NormalVectorsFilter
        params:
          input_layer: input
          output_layers_prefix: normal_vectors_
          radius: 0.05
          normal_vector_positive_axis: z

* **`gridMapFilters/NormalColorMapFilter`**

    Compute a new color layer based on normal vectors layers.

        name: surface_normals
        type: gridMapFilters/NormalColorMapFilter
        params:
          input_layers_prefix: normal_vectors_
          output_layer: normal_color

* **`gridMapFilters/MathExpressionFilter`**

    Parse and evaluate a mathematical matrix expression with layers of a grid map. See [EigenLab] for the documentation of the expressions.

        name: math_expression
        type: gridMapFilters/MathExpressionFilter
        params:
          output_layer: output
          expression: acos(normal_vectors_z) # Slope.
          # expression: abs(elevation - elevation_smooth) # Surface roughness.
          # expression: 0.5 * (1.0 - (slope / 0.6)) + 0.5 * (1.0 - (roughness / 0.1)) # Weighted and normalized sum.

* **`gridMapFilters/SlidingWindowMathExpressionFilter`**

    Parse and evaluate a mathematical matrix expression within a sliding window on a layer of a grid map. See [EigenLab] for the documentation of the expressions.

        name: math_expression
        type: gridMapFilters/SlidingWindowMathExpressionFilter
        params:
          input_layer: input
          output_layer: output
          expression: meanOfFinites(input) # Box blur
          # expression: sqrt(sumOfFinites(square(input - meanOfFinites(input))) ./ numberOfFinites(input)) # Standard deviation
          # expression: 'sumOfFinites([0,-1,0;-1,5,-1;0,-1,0].*elevation_inpainted)' # Sharpen with kernel matrix
          compute_empty_cells: true
          edge_handling: crop # options: inside, crop, empty, mean
          window_size: 5 # in number of cells (optional, default: 3), make sure to make this compatible with the kernel matrix
          # window_length: 0.05 # instead of window_size, in m

* **`gridMapFilters/DuplicationFilter`**

    Duplicate a layer of a grid map.

        name: duplicate
        type: gridMapFilters/DuplicationFilter
        params:
          input_layer: input
          output_layer: output

* **`gridMapFilters/DeletionFilter`**

    Delete layers from a grid map.

        name: delete
        type: gridMapFilters/DeletionFilter
        params:
          layers: [color, score] # List of layers.

Additionally, the *grid_map_cv* package provides the following filters:

* **`gridMapCv/InpaintFilter`**

    Use OpenCV to inpaint/fill holes in a layer.

        name: inpaint
        type: gridMapCv/InpaintFilter
        params:
          input_layer: input
          output_layer: output
          radius: 0.05 # in m


## Build Status

### Devel Job Status

| | Indigo | Jade | Kinetic |
| --- | --- | --- | --- |
| grid_map | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__grid_map__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__grid_map__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jdev__grid_map__ubuntu_trusty_amd64)](http://build.ros.org/job/Jdev__grid_map__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__grid_map__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__grid_map__ubuntu_xenial_amd64/) |
| doc | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__grid_map__ubuntu_trusty_amd64)](http://build.ros.org/job/Idoc__grid_map__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jdoc__grid_map__ubuntu_trusty_amd64)](http://build.ros.org/job/Jdoc__grid_map__ubuntu_trusty_amd64/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__grid_map__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__grid_map__ubuntu_xenial_amd64/) |

### Release Job Status

| | Indigo | Jade | Kinetic |
| --- | --- | --- | --- |
| grid_map | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map__ubuntu_xenial_amd64__binary/) |
| grid_map_core | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_core__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_core__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_core__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_core__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_core__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_core__ubuntu_xenial_amd64__binary/) |
| grid_map_ros | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_ros__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_ros__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_ros__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_ros__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_ros__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_ros__ubuntu_xenial_amd64__binary/) |
| grid_map_msgs | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_msgs__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_msgs__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_msgs__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_msgs__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_msgs__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_msgs__ubuntu_xenial_amd64__binary/) |
| grid_map_rviz_plugin | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_rviz_plugin__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_rviz_plugin__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_rviz_plugin__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_rviz_plugin__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_rviz_plugin__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_rviz_plugin__ubuntu_xenial_amd64__binary/) |
| grid_map_visualization | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_visualization__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_visualization__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_visualization__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_visualization__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_visualization__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_visualization__ubuntu_xenial_amd64__binary/) |
| grid_map_filters | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_filters__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_filters__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_filters__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_filters__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_filters__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_filters__ubuntu_xenial_amd64__binary/) |
| grid_map_loader | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_loader__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_loader__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_loader__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_loader__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_loader__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_loader__ubuntu_xenial_amd64__binary/) |
| grid_map_demos | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__grid_map_demos__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__grid_map_demos__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__grid_map_demos__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Jbin_uT64__grid_map_demos__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__grid_map_demos__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__grid_map_demos__ubuntu_xenial_amd64__binary/) |


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/grid_map/issues).

[ROS]: http://www.ros.org
[RViz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[OpenCV]: http://opencv.org/
[OctoMap]: https://octomap.github.io/
[PCL]: http://pointclouds.org/
[costmap_2d]: http://wiki.ros.org/costmap_2d
[grid_map_msgs/GridMapInfo]: http://docs.ros.org/api/grid_map_msgs/html/msg/GridMapInfo.html
[grid_map_msgs/GridMap]: http://docs.ros.org/api/grid_map_msgs/html/msg/GridMap.html
[grid_map_msgs/GetGridMap]: http://docs.ros.org/api/grid_map_msgs/html/srv/GetGridMap.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[visualization_msgs/Marker]: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
[geometry_msgs/PolygonStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html
[nav_msgs/OccupancyGrid]: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
[nav_msgs/GridCells]: http://docs.ros.org/api/nav_msgs/html/msg/GridCells.html
[ROS Filters]: http://wiki.ros.org/filters
[EigenLab]: https://github.com/leggedrobotics/EigenLab
