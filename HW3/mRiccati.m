function dVdt = mRiccati(t, V, A, B, Q, R)
V = reshape(V, size(A)); %Convert from "n^2"-by-1 to "n"-by-"n"
dVdt = -(Q - V*B*(R\(B.'))*V + V*A + A.'*V); %Determine derivative
dVdt = dVdt(:); %Convert from "n"-by-"n" to "n^2"-by-1

% Method inspired by https://www.mathworks.com/matlabcentral/answers/94722
% -how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab