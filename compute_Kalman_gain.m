function [ K ] = compute_Kalman_gain( P,H, resCov, EnableMeasurement)
%compute_Kalman_gain calculates the Kalman gain
K = P*H'*inv(resCov).*EnableMeasurement;
end
