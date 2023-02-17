function s = gauss_samples(x,P,n)
% Produce N random sample from multivariate Gaussian distribution.
% Renamed from multivariate_gauss.m
% INPUTS: 
% x, P - mean vector and covariance matrix 
% N - number of samples
%
% OUTPUT:
% s - set of N samples

len=length(x);
S=chol(P)';
X=randn(len,n); 
s=S*X+x*ones(1,n);
