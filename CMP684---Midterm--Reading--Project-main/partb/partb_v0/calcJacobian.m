function Jacobian = calcJacobian(q)
    conf.jnt1 = q(1); conf.jnt2 = q(2);   
    conf.jnt3 = q(3); conf.jnt4 = q(4);
    conf.jnt5 = q(5); conf.jnt6 = q(6);
    
    dhparams = [ 0,0,0,q(1);...
                 40,-90,0,q(2);...
                 275,0,0,q(3);...
                 25,-90,280,q(4);...
                 0,90,0,q(5);...
                 0,-90,0,q(6);];
             
    robot = rigidBodyTree; % Create a rigid body tree object to build the robot.
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('jnt1','revolute');
    
    setFixedTransform(jnt1,dhparams(1,:),'dh');
    body1.Joint = jnt1;
    
    addBody(robot,body1,'base') % Call addBody to attach the first body joint to the base frame of the robot.
    %% Create and add other rigid bodies to the robot.
    %% Specify the previous body name when calling addBody to attach it.
    %% Each fixed transform is relative to the previous joint coordinate frame.
    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('jnt2','revolute');
    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('jnt3','revolute');
    body4 = rigidBody('body4');
    jnt4 = rigidBodyJoint('jnt4','revolute');
    body5 = rigidBody('body5');
    jnt5 = rigidBodyJoint('jnt5','revolute');
    body6 = rigidBody('body6');
    jnt6 = rigidBodyJoint('jnt6','revolute');
    
    setFixedTransform(jnt2,dhparams(2,:),'dh');
    setFixedTransform(jnt3,dhparams(3,:),'dh');
    setFixedTransform(jnt4,dhparams(4,:),'dh');
    setFixedTransform(jnt5,dhparams(5,:),'dh');
    setFixedTransform(jnt6,dhparams(6,:),'dh');
    
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;
    
    addBody(robot,body2,'body1')
    addBody(robot,body3,'body2')
    addBody(robot,body4,'body3')
    addBody(robot,body5,'body4')
    addBody(robot,body6,'body5')
    
    Jacobian = geometricJacobian(robot,conf,'body6');
end