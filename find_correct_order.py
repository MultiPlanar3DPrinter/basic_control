from itertools import permutations

p = permutations([17, 22, 23, 27])
for k in p: 
    print(list(k))
