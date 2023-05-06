function closest = closest_number(x)
    sequence1 = 0:0.25:16; % Define the sequence with step size of 0.25
    sequence2 = 0:0.25:12; % Define the sequence with step size of 0.25
    [~, idx1] = min(abs(sequence1 - x(1))); % Find the index of the closest number
    [~, idx2] = min(abs(sequence2 - x(2))); % Find the index of the closest number
    closest = [sequence1(idx1), sequence2(idx2)]; % Return the closest number in the sequence
end