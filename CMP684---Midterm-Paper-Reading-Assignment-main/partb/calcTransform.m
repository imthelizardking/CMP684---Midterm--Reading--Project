function transformationMatrix = calcTransform(q)
% 
% 342 ilk satır
% dh; base ve tool satırlarını da almak lazım, -340 fln çıkıyor
%
%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    conf(1).JointName = 'jnt1'; conf(1).JointPosition = 0;
    conf(2).JointName = 'jnt2'; conf(2).JointPosition = q(1);
    conf(3).JointName = 'jnt3'; conf(3).JointPosition = q(2);
    conf(4).JointName = 'jnt4'; conf(4).JointPosition = q(3);
    conf(5).JointName = 'jnt5'; conf(5).JointPosition = q(4);
    conf(6).JointName = 'jnt6'; conf(6).JointPosition = q(5);
    conf(7).JointName = 'jnt7'; conf(7).JointPosition = q(6);
    conf(8).JointName = 'jnt8'; conf(8).JointPosition = 0;
    
    dhparams = [0,0,342,0;...
        0,0,0,q(1);...
        40,-90,0,q(2);...
        275,0,0,q(3);...
        25,-90,280,q(4);...
        0,90,0,q(5);...
        0,-90,0,q(6)
        0,0,73,0];
    
    robot = rigidBodyTree; % Create a rigid body tree object to build the robot.
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('jnt1','revolute');
    
    setFixedTransform(jnt1,dhparams(1,:),'dh');
    body1.Joint = jnt1;
    
    addBody(robot,body1,'base') % Call addBody to attach the first body joint to the base frame of the robot.
    
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
    body7 = rigidBody('body7');
    jnt7 = rigidBodyJoint('jnt7','revolute');
    body8 = rigidBody('body8');
    jnt8 = rigidBodyJoint('jnt8','revolute');
    
    setFixedTransform(jnt2,dhparams(2,:),'dh');
    setFixedTransform(jnt3,dhparams(3,:),'dh');
    setFixedTransform(jnt4,dhparams(4,:),'dh');
    setFixedTransform(jnt5,dhparams(5,:),'dh');
    setFixedTransform(jnt6,dhparams(6,:),'dh');
    setFixedTransform(jnt7,dhparams(7,:),'dh');
    setFixedTransform(jnt8,dhparams(8,:),'dh');
    
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;
    body7.Joint = jnt7;
    body8.Joint = jnt8;
    
    addBody(robot,body2,'body1')
    addBody(robot,body3,'body2')
    addBody(robot,body4,'body3')
    addBody(robot,body5,'body4')
    addBody(robot,body6,'body5')
    addBody(robot,body7,'body6')
    addBody(robot,body8,'body7')
    %showdetails(robot)
    transformationMatrix = getTransform(robot,conf,'base','body8');
end