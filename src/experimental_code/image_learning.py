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
    
    readout = nengo.Ensemble(n_neurons = 50, dimensions=1,radius=2)
    
    def desired_func(reservoir):
        w=[ -1.0,  -0.5, 0,   0.5, 1.0]
        output = np.dot(reservoir,w)
        return output
    
    def init_func(x):
        return 0

    learn_conn=nengo.Connection(reservoir,readout, function = init_func, learning_rule_type=nengo.PES())    

    error = nengo.Ensemble(n_neurons=100, dimensions=1)
    nengo.Connection(image, error, function=desired_func, transform=-1)
    # for input-output pairs do target values as a list and then as an attribute the according inputs
    nengo.Connection(readout, error, transform=1)
    # in total error=readout-desired_func(image)
    
    # connect the error to the weights that should be learned
    nengo.Connection(error, learn_conn.learning_rule)

    stop_learn = nengo.Node(1)
    nengo.Connection(stop_learn, error.neurons, transform=-10*np.ones((100,1)))
