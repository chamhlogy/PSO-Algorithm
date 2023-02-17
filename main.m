%标准粒群优化算法程序
%测试函数：f(x,y)=100(x^2-y)^2+(1-x)^2, -2.048<x,y<2.048
%求解函数最小值

global popsize;     %种群规模
%global popnum;      %种群数量
global pop;         %种群
%global c0;          %速度惯性系数,为0—1的随机数
global c1;          %个体最优导向系数
global c2;          %全局最优导向系数
global gbest_x;       %全局最优解x轴坐标
global gbest_y;       %全局最优解y轴坐标
global best_fitness;    %最优解
global best_in_history; %最优解变化轨迹
global x_min;           %x的下限
global x_max;           %x的上限
global y_min;           %y的下限
global y_max;           %y的上限
global gen;             %迭代次数
global exetime;         %当前迭代次数
global max_velocity;    %最大速度

initial;        %初始化

for exetime=1:gen
    outputdata;     %实时输出结果
    adapting;       %计算适应值
    updatepop;      %更新粒子位置
    pause(0.01);
end


clear i;
clear exetime;
clear x_max;
clear x_min;
clear y_min;
clear y_max;

