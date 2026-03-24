function regions = detectStaticRegions(is_static, min_len)
    if nargin < 2, min_len = 100; end 
    % 将逻辑数组转换为 [start_idx, end_idx] 矩阵
    diff_vals = diff([0; is_static; 0]);
    start_indices = find(diff_vals == 1);
    end_indices = find(diff_vals == -1) - 1;

    if isempty(start_indices)
    regions = [];
    return;
    end


    regions = [start_indices, end_indices];

    lengths = regions(:,2) - regions(:,1) + 1;
    valid_idx = lengths >= min_len;
    regions = regions(valid_idx, :);
end