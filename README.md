# MSL-RAPTOR ROS implementation

This repository contains a C++ ROS implementation of the back-end of [MSL-RAPTOR](https://arxiv.org/pdf/2012.09264). 

This implementation is based on [MSL-RAPTOR-Backend by Benjamin Ramtoula](https://github.com/bramtoula/MSL-RAPTOR-Backend).

[multi_tracker](https://github.com/tgieruc/multi_tracker) is the frontend and [angledbox_msgs](https://github.com/tgieruc/angledbox_msgs) is the message type for communication of the angled bounding box arrays.

The documentation for the different parameters can be found in the  [original project](https://github.com/bramtoula/MSL-RAPTOR-Backend).

# Citation

If you find this work helpful, please consider citing the original paper.

```
@inproceedings{MSL-RAPTOR2021,
  address   = {Cham},
  author    = {Ramtoula, Benjamin
               and Caccavale, Adam
               and Beltrame, Giovanni
               and Schwager, Mac},
  booktitle = {Experimental Robotics},
  editor    = {Siciliano, Bruno
               and Laschi, Cecilia
               and Khatib, Oussama},
  isbn      = {978-3-030-71151-1},
  pages     = {520--532},
  publisher = {Springer International Publishing},
  title     = {MSL-RAPTOR: A 6DoF Relative Pose Tracker for Onboard Robotic Perception},
  year      = {2021}
}
```

# Contact

For any questions, please email me at <theo.gieruc@gmail.com>
