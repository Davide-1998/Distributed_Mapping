rosshutdown  % Delete the  global node if any

% Remember to run roscore in the vm to avoid problems
% Matlab connection to ROS and Gazebo

ip_addr = "192.168.1.97";
master_port = 11311;

setenv("ROS_MASTER_URI","http://" + ip_addr + ":" + master_port)
setenv('ROS_IP', ip_addr)
setenv('ROS_HOSTNAME',ip_addr)
rosinit(ip_addr, master_port);

% disp ("Available topics")
rostopic list

topicname = "/MyLidar/ScanResults";
sub = rossubscriber(topicname, "DataFormat", "struct");
% rosnode ping /rosout

LidarData = receive(sub, 3);
array_of_collisions = LidarData.Ranges;
res_step = LidarData.AngleIncrement;

%Cicle over the data in the ranges
infs = isinf(array_of_collisions);
keys=[];
values=[];
for k = 1:numel(array_of_collisions)
    if infs(k) == 0
        keys = [keys (k-1)*res_step];
        values = [values array_of_collisions(k)];
    end
end

valueToPlot = containers.Map(keys, values);
figure
polarplot(keys, values, '.')