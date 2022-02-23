# constrained_planning_ompl
Planning Plugin for Geometrically Constrained Motion Planning in MoveIt (Melodic Version).

This package allows the usage of OMPL's constrained planning functions in MoveIt in order to plan paths which satisfy geometrical / kinematic task constraints. An example path is given below in which the end-effector rotation around the world's x- and y-axis are forced to zero. 

The implementation is inspired by the work of https://github.com/ros-planning/moveit/pull/2273. The difference is that a different framework for representing constraints is used and the atlas-based constraint adherence method is enabled as well. For constraint representation, this framework given by https://ieeexplore.ieee.org/document/5467152 is used, which only requires the specification of a task frame and a motion constraint vector C. These parameters can be specified in 'kin_constr_examples/config/test_panda.yaml'. Here, other parameters such as planning algorithm or constraint adherence method can be specified as well.

https://user-images.githubusercontent.com/86610929/155374688-66e6c9cf-e34b-483b-8c35-e2b05e5bcebd.mp4

Depending on the choice of planning algorithm or constraint adherence method, resulting paths do look quite different. It is assumed that choosing the atlas-based method instead of the projection-based method results in shorter paths.

https://user-images.githubusercontent.com/86610929/155374708-92065ac7-0c0b-447f-8ff7-6f2f5a94ac21.mp4

For quick start, clone this package into the moveit workspace and build it. Then open the panda RVIZ gui by calling 
 
`roslaunch panda_base_moveit_config demo.launch`
  
and start planning by calling

`roslaunch kin_constr_examples test_pipeline_panda.launch`.
