import nengo
import numpy as np

# input image: 5x5        -> motor angle
# leftleft  = [1 0 0 0 0] -> -1.0
# left      = [0 1 0 0 0] -> -0.5
# center    = [0 0 1 0 0] ->  0
# right     = [0 0 0 1 0] ->  0.5
# rightright= [0 0 0 0 1] ->  1.0

input = []
leftleft  = [1, 0, 0, 0, 0, -1.0]
left      = [0, 1, 0, 0, 0, -0.5]
center    = [0, 0, 1, 0, 0,  0]
right     = [0, 0, 0, 1, 0,  0.5]
rightright= [0, 0, 0, 0, 1,  1.0]

training_images = [
            leftleft[0:-1],
            left[0:-1],
            center[0:-1],
            right[0:-1],
            rightright[0:-1]
            ]
training_labels = [
            leftleft[-1],
            left[-1],
            center[-1],
            right[-1],
            rightright[-1]
            ]


print training_images
print training_labels


model = nengo.Network()
with model:
    image = nengo.Node([0,0,0,0,0])
    reservoir = nengo.Ensemble(n_neurons=500, dimensions=5,radius=1)
    
    nengo.Connection(image, reservoir)
    
    readout = nengo.Ensemble(n_neurons = 50, dimensions=1,radius=1)
    
    def matrixMultiply(reservoir):
        w=[ -1.0,  -0.5, 0,   0.5, 1.0]
        output = np.dot(reservoir,w)
        return output

    
    nengo.Connection(reservoir, readout,function=matrixMultiply)
