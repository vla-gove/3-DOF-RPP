startup_rvc;

%defining a three dof cylinder robot (RTT)
L(1) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(2) = Link('theta', 0, 'a', 0, 'alpha', pi/2);
L(3) = Link('theta', 0, 'a', 0, 'alpha', 0);
cyl3d=SerialLink(L, 'name', '3dof cylindrical');
%plotting various joint configurations
qz1=[0 0.0001 0.0001];
cyl3d.plot(qz1,'workspace',[-2 2 -2 2 -2 2]);
pause(2)
qz2=[0 1 0.0001];
cyl3d.plot(qz2,'workspace',[-2 2 -2 2 -2 2]);
pause(2)
qz3=[0 1 1];
cyl3d.plot(qz3,'workspace',[-2 2 -2 2 -2 2]);

%calculating a transformation matrix using a DH table for joint parameters
%qz4=[q1=pi/2 d1=1 d2=1]
alfa1=-pi/2;
q1=pi/2;
Z1r=trotz(q1);
X1r=trotx(alfa1);
T10=Z1r*X1r;

d2=1;
alfa2=pi/2;
q2=0;
Z2t=transl(0,0,d2);
Z2r=trotz(q2);
X2r=trotx(alfa2);
T21=Z2r*Z2t*X2r;

d3=1;
Z3t=transl(0,0,d3);
T32=Z3t;

disp('transformation matrix for a 3dof cylinder robot is:')
T30=T10*T21*T32


%calculating a transformation matrix using forward kinematics and given
%joint parameters qz4
disp('given joint parameters are:')
qz4=[pi/2 1 1]
disp('transformation matrix fkine returns is:')
T=cyl3d.fkine(qz4)

%we can notice both transformation matrices are the same

%calculating joint parameters using inverse kinematics
M=[0 0 1 0 1 1]; %defining a mask because the robot has <6 dof
P=[0 1 0]; %estimated joint parameters 
disp('joint parameters ikine returns are')
m=cyl3d.ikine(T, P, M , 'deg')

%due to our robot having less than 6 degrees of freedom, an estimate is
%necessary such that joint parameters can be calculated under 1000 steps
%through ikine

