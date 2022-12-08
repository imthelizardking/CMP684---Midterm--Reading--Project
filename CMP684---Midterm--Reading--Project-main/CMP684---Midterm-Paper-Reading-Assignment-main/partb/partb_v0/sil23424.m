E = [];
for i=1:nPredictionStep
    E = cat(1, E, eye(nOutputs));
end