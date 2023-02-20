function [L]=rss_foward(N,alpha,d0,L0,rad,sigma,x_target)
% RSS received signal
% N - sensor number
% alpha - sensor parameter
% d0 - sensor parameter
% L0 - power
% rad - sensor geometry
% sigma - noise std
% x_target - source

for i=1:N 
    s(1,i)=rad*cos(2*pi*(i-1)/N);
    s(2,i)=rad*sin(2*pi*(i-1)/N);
    S(:,:,i)=[eye(2) -s(:,i); -s(:,i).' s(:,i).'*s(:,i)];
end

% CRB��������
% alpha1=10*alpha/(sigma*log(10));
Q_gauss=0*sigma^2*ones(N)+1*1*sigma^2*eye(N);  % ������Ե��Ƶ�����  .. 
n_gauss=gauss_samples(zeros(N,1),Q_gauss,1);
L=zeros(N,1);

% observed signal
for i=1:N
    L(i)=L0+10*alpha*log10(norm(x_target-s(:,i))/d0)+n_gauss(i);
end 

end