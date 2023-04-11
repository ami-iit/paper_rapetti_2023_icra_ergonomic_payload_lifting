function w_H = rotationAverage(w_H_1, w_H_2)

posQuat_1 = wbc.fromTransfMatrixToPosQuat(w_H_1);
posQuat_2 = wbc.fromTransfMatrixToPosQuat(w_H_2);

pos_mean = (posQuat_1(1:3) + posQuat_2(1:3)) / 2;

% TODO: average rotation
w_H = w_H_1;

w_H(1:3, 4) = pos_mean;

end
