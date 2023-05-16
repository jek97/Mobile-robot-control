.. Mobile robot control documentation master file, created by
   sphinx-quickstart on Tue May 16 16:05:33 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Mobile robot control's documentation!
================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Mobile robot controll documentation
***********************************
This is the documentation of the Mobile robot control package in which a simulation of a differential drive robot, equipped with encoders and lidar is provided, the robot moves inside a closed environment following the bug0 algorithm to reach the desired goal specified by the user by the user interface.
the whole simulation rely on Gazebo and Rviz and it's encoded by a ROS architecture showing the use of publish/subscribe, services and action communication, both with standard formats and custom ones.
moreover the package is provided with a launch file used to launch all the simulation at once.

Go to point Module:
===========================
.. automodule:: scripts.go_to_point_service
   :members:
   
Wall follower Module:
===========================
.. automodule:: scripts.wall_follow_service
   :members:
   
Bug0 Module:
===========================
.. automodule:: scripts.bug_as
   :members:
   
Node A Module:
===========================
.. automodule:: scripts.node_a
   :members:
   
Node C Module:
===========================
.. automodule:: scripts.node_c
   :members:
   
Info server Module:
===========================
.. automodule:: scripts.info_server
   :members:
   
User interface Module:
===========================
.. automodule:: scripts.user_interface
   :members:
   

