%��ֵ����
%���Ժ���Ϊf(x,y)=100(x^2-y)^2+(1-x)^2, -2.048<x,y<2.048

%������Ӧֵ����ֵ
for i=1:popsize
    pop(i,8)=100*(pop(i,1)^2-pop(i,2))^2+(1-pop(i,1))^2;
    if pop(i,7)>pop(i,8)    %����ǰ��Ӧֵ���ڸ�������ֵ������и���������Ϣ�ĸ���
        pop(i,7)=pop(i,8);          %��ֵ����
        pop(i,5:6)=pop(i,1:2);      %λ���������
    end
end

%��������Ӧֵ��Ѱ�ҵ�ǰȫ������λ�ò���¼������
if best_fitness>min(pop(:,7))
    best_fitness=min(pop(:,7));     %ȫ������ֵ
    gbest_x=pop(find(pop(:,7)==min(pop(:,7))),1);    %ȫ���������ӵ�λ��   
    gbest_y=pop(find(pop(:,7)==min(pop(:,7))),2);
end



best_in_history(exetime)=best_fitness;  %��¼��ǰȫ������
