import time
from stable_baselines3 import PPO
from cartpole_drl_ppo.cartpole_env import CartPoleROS2Env
from ament_index_python.packages import get_package_share_directory

def main(args=None):

    package_name = 'cartpole_drl_ppo'  # Replace with your package name
    model_pkg_path = get_package_share_directory(package_name) + "/" 

    model = PPO.load(model_pkg_path + "ppo_cartpole_ros2.zip")
    env = CartPoleROS2Env()
    obs, _ = env.reset()
    time.sleep(1)

    done = False
    while not done:        
        action, _ = model.predict(obs)
        obs, _, done, _,  info = env.step(action)
        time.sleep(0.01)

    env.close()


if __name__ == '__main__':
    main()
