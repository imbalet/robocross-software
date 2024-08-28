import numpy as np
import os
import json

os.chdir(os.path.dirname(__file__))


# # a = np.ones((20, 20))
# a = [[1 for j in range(20)] for i in range(20)]
# for i in range(20):
#     a[i] [i] = 0
# # print(a)

# with open("m.json", 'w') as f:
#     json.dump(a, f)

with open("m.json") as f:
    a = json.load(f)
    print(a)
