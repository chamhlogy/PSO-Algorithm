%% ģ������
N=8; % ����������
x_target=[15;15]; % Ŀ��λ��

% ������λ��
rad=50;
for i=1:N 
    s(1,i)=rad*cos(2*pi*(i-1)/N);
    s(2,i)=rad*sin(2*pi*(i-1)/N);
    S(:,:,i)=[eye(2) -s(:,i); -s(:,i).' s(:,i).'*s(:,i)];
end

% ��ͼ
scatter(s(1,:),s(2,:),'b','LineWidth',1.5); hold on
scatter(x_target(1),x_target(2),'r','LineWidth',1.5); hold off
set(gca,'LineWidth',1);
axis([-80 80 -80 80]);
% set(gca,'Xtick',2800:100:3200);
% set(gca,'Ytick',0.5:0.1:0.9);
set(gca,'FontSize',13);
xlabel('West (m)','Fontname','LucidaBrightRegular','FontSize',10);
ylabel('North (m)','Fontname','LucidaBrightRegular','FontSize',10);
set(gcf,'Position',[180,180,500,450]);grid on
legend('������','Ŀ��λ��','Location','NorthEast');

%% �Ż����� (����������)

tic

% �����������ź�RSS
sigma=1; % ����
alpha=3; d0=1; L0=40; rad=50; % ��������������
L=rss_foward(N,alpha,d0,L0,rad,sigma,x_target); % �����������ź�
for i=1:N 
    s(1,i)=rad*cos(2*pi*(i-1)/N);
    s(2,i)=rad*sin(2*pi*(i-1)/N);
end

% ����Ⱥ�Ż���������
num_ptc=20; % ������
num_gen=100; % ��������
c1=2; c2=2; % ����ϵ����ѧϰ����

% �洢����
L_best=zeros(N,1);
L_new=zeros(N,1);
ptcs=zeros(num_ptc,2); % ����
vel=zeros(num_ptc,2); % �ٶ�
fit_P=zeros(num_ptc,1);  % ��Ӧ�ȴ洢����
Pi=zeros(num_ptc,2); % ȫ����������
vel_max=2; % �ٶ����ֵ
Pbest=zeros(num_gen,1); % �洢RMSE(��������)

% ����������ӳ�ʼλ�ú��ٶ�
for i=1:num_ptc
    for j=1:2
        ptcs(i,j)=50*(rand-0.5); % ��ʼ����λ��
        vel(i,j)=(0.5-rand)*2*vel_max; % ��ʼ�����ٶ�
    end
end         

% �����ʼ������Ӧ��ֵ�����ó�ʼ�����������Ӻ�ȫ����������
for i=1:num_ptc

    for j=1:N
        L_best(j)=L0+10*alpha*log10(norm(ptcs(i,:)'-s(:,j))/d0);
    end
    e_best=sum(abs(L_best-L)); % ��Ӧ��ֵ
    
    fit_P(i)=e_best;                    
    Pi(i,:)=ptcs(i,:); % ��ʼ������������

end

[P_min,P_index]=min(fit_P);
Pg=ptcs(P_index,:);
fit_Pg=P_min; % ��ʼȫ����������

% ���µ���������

for t=1:num_gen
        
      for i=1:num_ptc
            
            for j=1:2
                
%                 w(t)=(1-(1-0.1)*(t/num_gen))*exp(-t/num_gen); % Ȩ��ϵ����������
                w(t)=1;                                      % Ȩ��Ϊ����
                vel(i,j)=w(t)*vel(i,j)+c1*rand*(Pi(i,j)-ptcs(i,j))+c2*rand*(Pg(j)-ptcs(i,j)); % �ٶȸ���
                 
                % �޶��ٶȷ�Χ
                if vel(i,j)>vel_max
                   vel(i,j)=vel_max;
                end   
            
                if vel(i,j)<-vel_max
                   vel(i,j)=-vel_max;
                end 
                
                % ��������λ��
                ptcs(i,j)=ptcs(i,j)+w(t)*vel(i,j);
        
            end
        
        % �������λ�ú��������Ӧ��ֵ    
        for j=1:N
             L_best(j)=L0+10*alpha*log10(norm(ptcs(i,:)'-s(:,j))/d0);
        end
        e_best_new=sum(abs(L_best-L)); % �µ���Ӧ��ֵ
    
            if e_best_new<fit_P(i)
               fit_P(i)=e_best_new;
               Pi(i,:)=ptcs(i,:);
            end
        
            if fit_P(i)<fit_Pg
               Pg=Pi(i,:);
               fit_Pg=fit_P(i);
            end
      end
        
      Pbest(t)=fit_Pg;  % �洢ȫ��������Ӧ��ֵ���������̣�
        
end

%% ��������ͼ
% ��������
figure
plot(1:num_gen,Pbest,'LineWidth',1.5);
set(gca,'LineWidth',1); 
set(gca,'FontSize',10);
xlabel('��������','Fontname','LucidaBrightRegular','FontSize',10);
ylabel('������Ӧ��ֵRMSE','Fontname','LucidaBrightRegular','FontSize',10);
set(gcf,'Position',[180,180,500,350]);grid on

figure
scatter(s(1,:),s(2,:),'b','LineWidth',1.5); hold on
scatter(x_target(1),x_target(2),'r','LineWidth',1.5); hold on
scatter(Pg(1),Pg(2),'g','LineWidth',1.5); hold off
set(gca,'LineWidth',1.5);
axis([-80 80 -80 80]);
set(gca,'FontSize',10);
xlabel('West (m)','Fontname','LucidaBrightRegular','FontSize',10);
ylabel('North (m)','Fontname','LucidaBrightRegular','FontSize',10);
set(gcf,'Position',[180,180,500,450]);grid on
legend('������','Ŀ��λ��','����λ��','Location','NorthEast');

%% ����������

tic

M1=100; % ���Դ���
sigmav=1:1:20; % ��������
rms_ave=zeros(1,length(sigmav));
rms_rss=zeros(M1,length(sigmav));

for i=1:N 
    s(1,i)=rad*cos(2*pi*(i-1)/N);
    s(2,i)=rad*sin(2*pi*(i-1)/N);
end 

% ����Ⱥ�Ż���������
num_ptc=20; % ������
num_gen=100; % ��������
c1=2; c2=2; % ����ϵ����ѧϰ����

for sig=1:length(sigmav)

alpha=3; d0=1; L0=40; rad=50; % ��������������
L=rss_foward(N,alpha,d0,L0,rad,sigmav(sig),x_target); % �����������ź�
   
    for k=1:M1
        
    % �洢����
    L_best=zeros(N,1);
    L_new=zeros(N,1);
    ptcs=zeros(num_ptc,2); % ����
    vel=zeros(num_ptc,2); % �ٶ�
    fit_P=zeros(num_ptc,1);  % ��Ӧ�ȴ洢����
    Pi=zeros(num_ptc,2); % ȫ����������
    vel_max=2; % �ٶ����ֵ
    Pbest=zeros(num_gen,1); % �洢RMSE(��������)

    % ����������ӳ�ʼλ�ú��ٶ�
    for i=1:num_ptc
        for j=1:2
            ptcs(i,j)=50*(rand-0.5); % ��ʼ����λ��
            vel(i,j)=(0.5-rand)*2*vel_max; % ��ʼ�����ٶ�
        end
    end         

    % �����ʼ������Ӧ��ֵ�����ó�ʼ�����������Ӻ�ȫ����������
    for i=1:num_ptc

        for j=1:N
            L_best(j)=L0+10*alpha*log10(norm(ptcs(i,:)'-s(:,j))/d0);
        end
        e_best=sum(abs(L_best-L)); % ��Ӧ��ֵ
    
        fit_P(i)=e_best;                    
        Pi(i,:)=ptcs(i,:); % ��ʼ������������

    end

    [P_min,P_index]=min(fit_P);
    Pg=ptcs(P_index,:);
    fit_Pg=P_min; % ��ʼȫ����������

% ���µ���������

    for t=1:num_gen
        
          for i=1:num_ptc
            
            for j=1:2
                
                w(t)=(1-(1-0.1)*(t/num_gen))*exp(-t/num_gen); % Ȩ��ϵ����������
%                 w(t)=1;                                        % Ȩ��Ϊ����
                vel(i,j)=w(t)*vel(i,j)+c1*rand*(Pi(i,j)-ptcs(i,j))+c2*rand*(Pg(j)-ptcs(i,j)); % �ٶȸ���
                 
                % �޶��ٶȷ�Χ
                if vel(i,j)>vel_max
                   vel(i,j)=vel_max;
                end   
            
                if vel(i,j)<-vel_max
                   vel(i,j)=-vel_max;
                end 
                
                % ��������λ��
                ptcs(i,j)=ptcs(i,j)+w(t)*vel(i,j);
        
            end
        
        % �������λ�ú��������Ӧ��ֵ    
              for j=1:N
                   L_best(j)=L0+10*alpha*log10(norm(ptcs(i,:)'-s(:,j))/d0);
              end
              e_best_new=sum(abs(L_best-L)); % �µ���Ӧ��ֵ
    
              if e_best_new<fit_P(i)
                 fit_P(i)=e_best_new;
                 Pi(i,:)=ptcs(i,:);
              end
        
              if  fit_P(i)<fit_Pg
                  Pg=Pi(i,:);
                  fit_Pg=fit_P(i);
              end
          end
        
       Pbest(t)=fit_Pg;  % �洢ȫ��������Ӧ��ֵ���������̣�
      
       
    end
    
       rms_rss(k,sig)=fit_Pg; % ÿ�β��Ե����Ŀ�꺯��ֵrms
    
    end
    
     rms_ave(sig)=sqrt(sum(rms_rss(:,sig))/M1); % M1�β������Ŀ�꺯��ֵrms��ƽ��ֵ(��¼��ͬ�����µ�rms)
%        rms_ave(sig)=min(rms_rss(:,sig));
       disp(sig)     
    
end

toc

%% ���������Խ����ͼ(��ͬ�����µ����rms)
figure
rms_aver=rms_ave;
% plot(sigmav,rms_aver);hold on ;
box off
scatter(sigmav,rms_aver,'r','LineWidth',1.0); hold on;grid on
a=polyfit(sigmav,rms_aver,3);
x0=0:0.1:20;
y0=polyval(a,x0);
plot(x0,y0,'-b');hold off
set(gca,'LineWidth',1);
% axis([1 6 0 18]);
% set(gca,'Xtick',1:0.5:6);
% set(gca,'Ytick',0:2:18);
% set(gca,'xdir','reverse'); 
set(gca,'FontSize',10);
xlabel('\sigma','Fontname','LucidaBrightRegular','FontSize',10);
ylabel('RMSE (m)','Fontname','LucidaBrightRegular','FontSize',10);
set(gcf,'Position',[180,180,500,350]);grid on