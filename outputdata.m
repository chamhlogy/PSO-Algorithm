%实时输出结果

%输出当前种群中粒子位置
subplot(1,2,1);
for i=1:popsize
    plot(pop(i,1),pop(i,2),'b*');
    title('Particle positions');
    xlabel('X coordinate');
    ylabel('Y coordinate');
    hold on;
end

plot(gbest_x,gbest_y,'r.','markersize',20);axis([-2,2,-2,2]);
hold off;

subplot(1,2,2);
axis([0,gen,-0.00005,0.00005]);
title('Best fitness');
xlabel('Iteration');
ylabel('Fitness');

if exetime-1>0
    line([exetime-1,exetime],[best_in_history(exetime-1),best_fitness]);hold on;
end