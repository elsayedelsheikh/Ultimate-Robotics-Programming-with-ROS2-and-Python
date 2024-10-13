

from cartpole_drl_ppo.cartpole_env import CartPoleROS2Env
from stable_baselines3 import PPO
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    
    package_name = 'cartpole_drl_ppo'  # Replace with your package name
    model_pkg_path = get_package_share_directory(package_name) + "/" 

    env = CartPoleROS2Env()
    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=1, progress_bar=True)
    model.save(model_pkg_path + "ppo_cartpole_ros2")

    env.close()
   
    
if __name__ == '__main__':
    main()
