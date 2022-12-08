function pose = calcPose(q)
    mTransformation = calcTransform(q);
    pose = zeros(6,1);
    pose(1:3) = mTransformation(1:3,4);
    pose(5) = atan2(-mTransformation(3,1),sqrt(mTransformation(1,1)+mTransformation(2,1)));
    pose(6) = atan2(mTransformation(2,1)/cos(pose(5)),mTransformation(1,1)/cos(pose(5)));
    pose(4) = atan2(mTransformation(3,2)/cos(pose(5)),mTransformation(3,3)/cos(pose(5)));
end