import gymnasium as gym
import numpy as np
from cartpole_drl_ppo.cartpole_env import CartPoleROS2Env

#from cartpole_env import CartPoleROS2Env

def main(args=None):
    print("Main")
    env = CartPoleROS2Env()
    print("Env initialized!")

    # Initialize the environment
    

    for e in range(3):
        observation, info = env.reset()
        done = False
        total_reward = 0.0


        while not done:
            # Sample a random action
            action = env.action_space.sample()
            #print("Action: ", action)
            # Apply the action
            select_action = 0

            for i in range(10000):
                
                if ( select_action == 0):
                    select_action = 1
                    action = [15]   
                else:
                    select_action = 0
                    action = [-15] 
                observation, reward, done, _,  info = env.step(action)
                # Accumulate the reward
                total_reward += reward
            #print("total reward")
            
            #print(f"Observation: {observation}, Reward: {reward}, Done: {done}")

        print(f"Episode finished with total reward: {total_reward}")
        print("e: ", e)
    # Close the environment
    env.close()
    


if __name__ == '__main__':
    main()
