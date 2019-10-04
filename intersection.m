function cross = intersection(line1, line2)
    cross=[line1(1) -1; line2(1) -1]\[-line1(2);-line2(2)];
end