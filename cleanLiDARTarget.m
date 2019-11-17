function [X_clean, bag_data] = cleanLiDARTarget(scan_num, tag_num, bag_data, X, target_len, opt)
    N = 4; % clean up using N std for x axis
    M = 3; % clean up using M std for y, z axis
    opt = optimizeCost(opt, X, target_len, 0.001);
    X_ref = opt.H_opt * X;
%     distance = sum(X_ref(1:3,:), 1); % L1
    L_infinity = max(abs(X_ref(1:3,:)));  % L infinity
    K = find(L_infinity < (target_len/2)*1.025);
    X_ref_clean_yz = X_ref(:, K); % clean up y and z axis
    L_infinity = max(X_ref_clean_yz(1:3,:));
    K_center = find(L_infinity < target_len/4);
    X_std = std(X_ref_clean_yz(:, K_center), 1, 2);
    Q = find(abs(X_ref_clean_yz(1,:)) < N*(X_std(1))); % clean up x with 2 std
    X_ref_clean = X_ref_clean_yz(:, Q);
    X_clean = inv(opt.H_opt) * X_ref_clean;
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.std = N*(X_std(1));
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.L_infinity = L_infinity;
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.L_1 = sum(X_ref(1:3,:), 1);
%     figure(200);
%     clf('reset')
%     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:))
%     xlabel('x') 
%     ylabel('y') 
%     zlabel('z') 
%     axis equal
%     hold on;
%     scatter3(X_ref_clean_yz(1,:), X_ref_clean_yz(2,:), X_ref_clean_yz(3,:))
%     scatter3(X_ref_clean(1,:), X_ref_clean(2,:), X_ref_clean(3,:))
%     axis equal
%     view(90,0)
%     hold off;    
end