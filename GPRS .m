clear all
area_l=1000;
area_w=1000;
R=100;
num_node =300; 

% 随机生成源节点及目标节点
des_x(1) =area_l*0.9+area_l*0.1*rand(1,1);     
des_y(1) =area_w*rand(1,1);
source_x(1) =area_l*0.1*rand(1,1);     
source_y(1) =area_w*rand(1,1);

% 随机生成300个传感器节点
for i=1:num_node
    node_x(i)=area_l*rand(1,1);     
    node_y(i)=area_w*rand(1,1);
end

%initial 初始化
for node_i=1:num_node
    for num_mem=1:num_node
        neb_node_x(node_i,num_mem) =0;
        neb_node_y(node_i,num_mem) =0;
        num_neb(node_i)=num_mem;  % 记录每个节点的邻居节点数
    end
end

% find the neb node 寻找邻节点
for node_i=1:num_node
    num_mem=0;
    for node_j=1:num_node
        if node_i~=node_j && sqrt((node_x(node_i)-node_x(node_j))^2+(node_y(node_i)-node_y(node_j))^2)<=R  %记录路由搜索范围内节点
            num_mem= num_mem+1;
            neb_node_x(node_i,num_mem) =node_x(node_j);
            neb_node_y(node_i,num_mem) =node_y(node_j);
            neb_node_id(node_i,num_mem) =node_j;
        end
    end
    num_neb(node_i)=num_mem;
end

total_neb=0;

for node_i=1:num_node
    total_neb=total_neb+num_neb(node_i);
end

if total_neb~=0
    connectedness =total_neb/num_node;
else    connectedness =0;
end
%---------------------------------
hop=num_node;
for i_hop_i=1:hop
    pre_hop_node_x(i_hop_i) =0;
    pre_hop_node_y(i_hop_i) =0;
    present_node_x(i_hop_i) =0;
    present_node_y(i_hop_i)= 0;
end

num_source_neb=0;
% 寻找源节点的邻居节点
for node_i=1:num_node
    if sqrt((node_x(node_i)-source_x )^2+(node_y(node_i)-source_y)^2)<=R
        num_source_neb=num_source_neb+1;
        neb_node_source_x(num_source_neb) =node_x(node_i);
        neb_node_source_y(num_source_neb) =node_y(node_i);
        source_neb_node_id(num_source_neb)=node_i;
    end
end

next_hop_node_coordinate=0;   % 下一跳坐标
min_mem_1=sqrt((source_x-des_x)^2 + (source_y -des_y)^2);
if num_source_neb~=0
    flag_no_void_source=0;
    for i_num_source_neb=1:num_source_neb
        if sqrt( (neb_node_source_x(i_num_source_neb)-des_x)^2+( neb_node_source_y(i_num_source_neb)-des_y)^2)< min_mem_1  % c1
            min_mem_1= sqrt( (neb_node_source_x(i_num_source_neb)-des_x)^2+ ( neb_node_source_y(i_num_source_neb)-des_y)^2);
            next_hop_node_coordinate=source_neb_node_id(i_num_source_neb);
            flag_no_void_source=1;
        end
    end
    
    if flag_no_void_source==0
        b_in= atan((source_y-0)/(source_x-0));
        if  b_in<0
            b_in=b_in +2*pi;
        end
        sita_min=3*pi;
        for i_num_souce_neb=1:num_source_neb
            b_a=atan((source_y-neb_node_source_y(i_num_source_neb))/(source_x-neb_node_source_x(i_num_source_neb)));    % norm() means adding 2*pi
            if  b_a<0
                b_a=b_a+2*pi;
            end
            sita_b=(b_a-b_in);
            if  sita_b<0
                sita_b=sita_b+2*pi;
            end
            if sita_b<sita_min
                sita_min=sita_b;
                a_min=source_neb_node_id(i_num_source_neb);
            end
        end
        next_hop_node_coordinate=a_min;
    end
end

present_node_x(1) = source_x;
present_node_y(1) = source_y;
hop_num=1;
% 开始转发
for i_hop=2:hop
    if next_hop_node_coordinate~=0
        pre_hop_node_x(i_hop) = present_node_x(i_hop-1);
        pre_hop_node_y(i_hop) = present_node_y(i_hop-1);
        present_node_x(i_hop) = node_x(next_hop_node_coordinate);
        present_node_y(i_hop) = node_y(next_hop_node_coordinate);
        min_mem_2 = sqrt((present_node_x(i_hop)-des_x)^2+(present_node_y(i_hop)-des_y)^2);
        if num_neb(next_hop_node_coordinate)~=0
            flag_no_void=0;
            for i_num_node_neb=1:num_neb(next_hop_node_coordinate)
              if pre_hop_node_x(i_hop)~=neb_node_x(next_hop_node_coordinate,i_num_node_neb)  
                if sqrt( (neb_node_x(next_hop_node_coordinate,i_num_node_neb)-des_x)^2+( neb_node_y(next_hop_node_coordinate,i_num_node_neb)-des_y)^2)< min_mem_2  % c3
                    min_mem_2=sqrt( (neb_node_x(next_hop_node_coordinate,i_num_node_neb)-des_x)^2+( neb_node_y(next_hop_node_coordinate,i_num_node_neb)-des_y)^2);
                    next_hop_node_coordinate_temp=neb_node_id(next_hop_node_coordinate,i_num_node_neb);
                    flag_no_void=1;
                end
              end  
            end
            % 未出现路由空洞问题
            if flag_no_void==1 
                next_hop_node_coordinate = next_hop_node_coordinate_temp;
                hop_num=hop_num+1;
            % 出现路由空洞问题
            elseif flag_no_void==0
                fprintf('在%d处出现路由空洞\n',hop_num+1);
                b_in= atan((des_y-present_node_y(i_hop))/(des_x-present_node_x(i_hop)));
                if b_in < 0
                    b_in=-b_in;
                elseif b_in > 0
                    b_in=2*pi-b_in;
                end
                sita_min=3*pi;
                flag_a_min_avail=0;
                for i_num_node_neb=1:num_neb(next_hop_node_coordinate)
                    if pre_hop_node_x(i_hop)~=neb_node_x(next_hop_node_coordinate,i_num_node_neb)
                        b_a=atan((des_y-neb_node_y(next_hop_node_coordinate,i_num_node_neb))/(des_x-neb_node_x(next_hop_node_coordinate,i_num_node_neb)));
                        if b_a < 0
                            b_a=-b_a;
                        elseif b_a > 0
                            b_a=2*pi-b_a;
                        end
                        if (b_a-b_in)<0
                            sita_b=(b_a-b_in)+2*pi;
                        else
                            sita_b=(b_a-b_in);
                        end
                        if sita_b<sita_min
                            sita_min=sita_b;
                            a_min=neb_node_id(next_hop_node_coordinate,i_num_node_neb);
                            flag_a_min_avail=1;
                        end
                    end
                end
                if flag_a_min_avail==1
                    next_hop_node_coordinate=a_min;
                    hop_num = hop_num+1;
                else
                    error('无法找到周界转发节点');
                end
            end
            
            if i_hop==hop && ( sqrt((present_node_x(i_hop)-des_x)^2+(present_node_y(i_hop)-des_y)^2) > R ) %
                plot(3,5)
                text(3,5,'路由转发失败')
                error('出现周界转发困境')
            end
            if sqrt((node_x(next_hop_node_coordinate)-des_x)^2+(node_y(next_hop_node_coordinate)-des_y)^2) <= R   % dd
                num_rout_track=i_hop;
                present_node_x(i_hop) = node_x(next_hop_node_coordinate);
                present_node_y(i_hop) = node_y(next_hop_node_coordinate);
                present_node_x(i_hop+1) = des_x ;
                present_node_y(i_hop+1) = des_y ;
                i_hop=hop;
                break
            end
        end
    end
end

plot(des_x,des_y,'sk')  
hold on  
plot(source_x,source_y,'pk')  
hold on  
plot(node_x,node_y,'.k')  
hold on  
plot(present_node_x(1:num_rout_track+1 ),present_node_y(1:num_rout_track+1 ),'x-r')  
	  
for k=1:num_rout_track+1  
text(present_node_x(k)+1,present_node_y(k),num2str(k) );  
end  
title('GPSR路径','fontsize',10);  
xlabel('X轴','fontsize',10);  
ylabel('Y轴','fontsize',10);  
legend('目标节点','源节点','传感器节点','GPSR路径'); 

