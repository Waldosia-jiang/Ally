clc
clear
close all
%% ������
% ����μ��߶�
poly = [-1,1; 1,1; 1,-1; -1,-1;-1,1];
line1 = [0.5,0; 0.5,1.5];
line2 = [0,2; 0,1];
line3 = [-0.5,1.2; -0.5,2];
line4 = [-2 1.5; -1.5,1.5];
line5 = [-2 1; -1.5,1];
line6 = [-0.5 1; 1.5,1];

% ��ͼ
figure
hold on
plot(poly(2:end,1), poly(2:end,2),'k','linewidth',2);
plot(poly(1:2,1), poly(1:2,2),'k--','linewidth',2);
plot(line1(:,1), line1(:,2),'b-*');
plot(line2(:,1), line2(:,2),'c-*');
plot(line3(:,1), line3(:,2),'m-*');
plot(line4(:,1), line4(:,2),'g-*');
plot(line5(:,1), line5(:,2),'k-*');
plot(line6(:,1), line6(:,2),'-*');

% ������ײ��⺯��
poly_temp = poly(1:2,:);
is_intersect1 = intersect_check(line1,poly_temp);
is_intersect2 = intersect_check(line2,poly_temp);
is_intersect3 = intersect_check(line3,poly_temp);
is_intersect4 = intersect_check(line4,poly_temp);
is_intersect5 = intersect_check(line5,poly_temp);
is_intersect6 = intersect_check(line6,poly_temp);

%% ����
% ��ײ���
function is_intersect = intersect_check(line,poly)
[A,B] = sortPoint(line);
ployPointNum = size(poly,1);
is_intersect = false;
for i = 1:ployPointNum-1
    line_temp = poly(i:i+1,:);
    [C,D] = sortPoint(line_temp);
    % 1-����߶�CD�������˵��Ƿ�λ���߶�AB����
    AB = B - A;
    AC = C - A;
    AD = D - A;
    result1 = AB(1) * AC(2) - AB(2) * AC(1);
    result2 = AB(1) * AD(2) - AB(2) * AD(1);
    
    % 2-����߶�AB�������˵��Ƿ�λ���߶�CD����
    CD = D - C;
    CA = A - C;
    CB = B - C;
    result3 = CD(1) * CA(2) - CD(2) * CA(1);
    result4 = CD(1) * CB(2) - CD(2) * CB(1);
    
    % 3-�ж������߶��Ƿ��ཻ
    if result1 * result2 < 0 && result3 * result4 < 0 || ...
            result1 * result2 == 0 && result3 * result4 < 0 ||...
            result1 * result2 < 0 && result3 * result4 == 0
        % ��������ΪX�Σ�����һ���˵�����һ���߶��ϣ�T�Σ�,���ཻ
        is_intersect = true;
        break
    elseif result1 == 0 && result2 == 0 &&  result3 == 0 && result4 == 0
        % 4����Ϊ0�����������߶ι���,���Ƿ��غ����һ���ж�
        % �����߶ζ˵��Ѿ�����ֻ��Ҫ�ų����ߵ����غϵ��������
        if ~(C(1) > B(1) || D(1) < A(1) || ...  % X����
                C(2) > B(2) || D(2) < A(2))  % Y����
            is_intersect = true;
            break
        end
    end
end
end

% ���߶ε������˵�����
function [A,B] = sortPoint(line)
A = line(1,:);
B = line(2,:);
% ���߶εĶ˵㰴�մ�С����
if line(1,1) < line(2,1)
    A = line(1,:);
    B = line(2,:);
elseif line(1,1) > line(2,1)
    A = line(2,:);
    B = line(1,:);
else
    if line(1,2) < line(2,2)
        A = line(1,:);
        B = line(2,:);
    elseif line(1,2) > line(2,2)
        A = line(2,:);
        B = line(1,:);
    end
end
end