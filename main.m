%��׼��Ⱥ�Ż��㷨����
%���Ժ�����f(x,y)=100(x^2-y)^2+(1-x)^2, -2.048<x,y<2.048
%��⺯����Сֵ

global popsize;     %��Ⱥ��ģ
%global popnum;      %��Ⱥ����
global pop;         %��Ⱥ
%global c0;          %�ٶȹ���ϵ��,Ϊ0��1�������
global c1;          %�������ŵ���ϵ��
global c2;          %ȫ�����ŵ���ϵ��
global gbest_x;       %ȫ�����Ž�x������
global gbest_y;       %ȫ�����Ž�y������
global best_fitness;    %���Ž�
global best_in_history; %���Ž�仯�켣
global x_min;           %x������
global x_max;           %x������
global y_min;           %y������
global y_max;           %y������
global gen;             %��������
global exetime;         %��ǰ��������
global max_velocity;    %����ٶ�

initial;        %��ʼ��

for exetime=1:gen
    outputdata;     %ʵʱ������
    adapting;       %������Ӧֵ
    updatepop;      %��������λ��
    pause(0.01);
end


clear i;
clear exetime;
clear x_max;
clear x_min;
clear y_min;
clear y_max;

