% testing
q = PriorityQueue(false);
q.insert(3, 'h');
q.insert(1, 'a');
q.insert(2, 'b');
q.insert(4, 'e');

[pri, val] = q.pop();

subspace([1; -1], [-1; -1])