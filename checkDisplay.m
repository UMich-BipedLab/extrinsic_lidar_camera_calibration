function display = checkDisplay(display)
    if strcmpi("display", display) || strcmpi("1", string(display))
        display = 1;
    else
        display = 0;
    end
end