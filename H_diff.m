function difference = H_diff(H1, H2)
    difference = H1 \ H2;
    logR = logm(difference(1:3,1:3));
    v = [-logR(1,2) logR(1,3) -logR(2,3) difference(1:3,4)'];
    difference = norm(v);
end