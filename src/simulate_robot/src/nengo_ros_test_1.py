import nengo 

  


model = nengo.Network() 

with model: 

    stim = nengo.Node([0]) 

    a = nengo.Ensemble(n_neurons=50, dimensions=1) 

    nengo.Connection(stim, a) 

     

    def output(t, x): 

        print(x) 

        # here you can publish the motor values to a ROS topic 

         

         

    b=nengo.Node(output,size_in=1) 

     

    nengo.Connection(a,b) 