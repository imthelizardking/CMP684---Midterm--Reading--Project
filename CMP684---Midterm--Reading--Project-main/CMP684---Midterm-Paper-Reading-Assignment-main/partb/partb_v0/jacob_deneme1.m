clear all; clc;
q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0; q6 = 0;
dhparams = [ 0,0,342,0;...
             0,0,0,q1;...
             40,-90,0,q2;...
             275,0,0,q3;...
             25,-90,280,q4;...
             0,90,0,q5;...
             0,-90,0,q6;...
             0,0,73,0];
         
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('base');
jnt1 = robotics.RigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
