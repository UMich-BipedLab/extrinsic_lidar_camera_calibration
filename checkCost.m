function cost = checkCost(point, cons1, cons2)
    if point >= cons1 && point <= cons2
        cost = 0;
    else
        dis = min(abs(point-cons2), abs(point-cons1));
        cost = getCost(dis);
    end
end