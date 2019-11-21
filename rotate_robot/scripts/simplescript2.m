clc;
rosinit;
a = 'it works easily...';
t1 = rossubscriber('/zed/zed_node/left/image_rect_color');
t2 = rossubscriber('/zed/zed_node/right/image_rect_color');
disp(a);
run test3