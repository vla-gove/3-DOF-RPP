# 3dofrppRVC
3DOF Cylindrical Robot (RPP) in RVC Matlab toolbox;
Defining a 3DOF RPP robot, use of DH tables and forward kinematics (fkine) for finding transformation matrices, and inverse kinematics (ikine) for finding joint parameters in a <6DOF robot. Using ikine on a robot with less than six degrees of freedom requires both using a mask which specifices cartesian degrees of freedom that should be ignored, and supplying the function an estimation of joint parameter values it should return in under 1000 steps.
