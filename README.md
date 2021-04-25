# ParticleSwarmOptimization
## EXAMPLE CODE:
```pythin
def Problem5(inPut):
    x = inPut[0]
    y = inPut[1]
    return -x * math.sin(4 * x)-1.1*y*math.sin(2*y)+1

def Constrain(inPut):
    x = inPut[0]
    y = inPut[1]
    return (x + y <= 22)

ParticleSwarmOptimization(8, 10, 10, 13, 3, 0.5, 1, 2, 2, 10, Problem5, Constrain).doRun()
```
