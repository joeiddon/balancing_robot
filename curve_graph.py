import matplotlib.pyplot as plt
import ast

s = open("balancingRobotCurve.ino").read()
spds = next(l for l in s.split("\n") if '{' in l and '}' in l)
spds = '[' + spds[spds.index('{')+1:spds.index('}')] + ']'
spds = ast.literal_eval(spds)
plt.plot(range(len(spds)), spds)
plt.show()
