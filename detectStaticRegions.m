function regions = detectStaticRegions(acc_data, gyro_data, fs, min_static_sec)

    if nargin < 4, min_static_sec = 25; end
    

    win_len = max(10, round(fs * 5));
    acc_var = movvar(vecnorm(acc_data, 2, 2), win_len);
    gyro_var = movvar(vecnorm(gyro_data, 2, 2), win_len);
    

    is_static = (acc_var < 0.008) & (gyro_var < 0.0005);
    is_static(isnan(is_static)) = false;
    

    d = diff([0; is_static; 0]);
    starts = find(d == 1); ends = find(d == -1) - 1;
    if isempty(starts), regions = []; return; end
    regions = [starts, ends];
    

    lengths = regions(:,2) - regions(:,1) + 1;
    min_len = round(min_static_sec * fs);
    regions = regions(lengths >= min_len, :);
    

    if size(regions, 1) > 1
        merged = regions(1, :);
        for i = 2:size(regions, 1)
            gap = regions(i, 1) - merged(end, 2);
            if gap < round(2 * fs) % 间隔<2s才考虑合并
                acc_prev = mean(acc_data(merged(end,1):merged(end,2), :));
                acc_curr = mean(acc_data(regions(i,1):regions(i,2), :));
                angle_diff = acosd(dot(acc_prev, acc_curr) / (norm(acc_prev)*norm(acc_curr)));
                if angle_diff < 5 % 方向变化<5°视为同一静止面
                    merged(end, 2) = regions(i, 2);
                    continue;
                end
            end
            merged = [merged; regions(i, :)];
        end
        regions = merged;
    end
end