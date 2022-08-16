startup_rvc;

%defining a three dof cylinder robot (RTT)
L(1) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(2) = Link('theta', 0, 'a', 0, 'alpha', pi/2);
L(3) = Link('theta', 0, 'a', 0, 'alpha', 0);
cyl3d=SerialLink(L, 'name', '3dof cylindrical');

q1=[0 1 1];

%jacobian for given position q1
J=jacob0(cyl3d,q1)

