import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# The plot function will be started as a thread, so we import the threading modules
import threading

# matplotlib is used to plot the data read from the topic
import matplotlib.pyplot as plt


class SinusoidalSubscriber(Node):

    def __init__(self):
        super().__init__('sinusoidal_subscriber')
        
        # A function invoked everytime a new data is published on a topic is called callback.
        # The callback is specified in the creation of the subscriber
        self.sub = self.create_subscription(Float64, 'sinusoidal_signal', self.sinu_sub, 10)
        
        # Flat to check when data are published
        self.first_data_arrived = False
        self.sinu_values = None
        
        # Time in seconds to store new data 
        self.logging_time = 15
        
        
    def sinu_sub( self, msg ):
        self.sinu_values = msg.data
        self.first_data_arrived = True
        
    def log_data(self):

        rate = self.create_rate(10)
        sinu_data = []
        
        # Wait the first data arrived	
        while( self.first_data_arrived == False ):
            rate.sleep()
        
        self.get_logger().info('Starting logging sine wave!')
        t = 0.0
        
        # Wait that the time to log is elapsed
        while ( t < self.logging_time ):
            sinu_data.append( self.sinu_values )            
       	    t = t+0.1 
       	    rate.sleep()
       	    
        self.destroy_node() 
        
        # Generate the plot
        plt.plot(sinu_data)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Sine wave data')
        plt.show()
        
        rclpy.shutdown() 
                
def main(args=None):
    
    rclpy.init(args=args) 
    sinusoidal_subscriber = SinusoidalSubscriber()
     
    # Start a new thread that logs the data running in parallel with the rest of the code
    t = threading.Thread(target=sinusoidal_subscriber.log_data, args=[])
    t.start()
        
    rclpy.spin(sinusoidal_subscriber) 

   
   
if __name__ == '__main__':
    main()

