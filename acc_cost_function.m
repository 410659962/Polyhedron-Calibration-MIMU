function err = acc_cost_function(params, acc_raw_static, g_ref)

    b = params(1:3);
    s = params(4:6);
    m = params(7:12);
    

    T = eye(3);
    T(1,2) = -m(1); T(1,3) =  m(2);
    T(2,1) =  m(3); T(2,3) = -m(4);
    T(3,1) = -m(5); T(3,2) =  m(6);
    
    K = diag(s);

    acc_O = (acc_raw_static - b) / (T * K)';
    

    err = sum(acc_O.^2, 2) - g_ref^2;
end